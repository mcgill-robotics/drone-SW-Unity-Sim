using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

namespace DroneSim.Tests
{
    /// <summary>
    /// PX4 SITL's end of the HIL link: connects to the bridge, decodes what it sends and, as PX4
    /// does in lockstep, answers every HIL_SENSOR with HIL_ACTUATOR_CONTROLS. Runs on its own
    /// thread, as PX4 runs in its own process. Values are as sent: FRD, NED, SI units.
    /// </summary>
    public sealed class FakePx4 : IDisposable
    {
        public struct Sensor { public ulong timeUs; public Vector3 acc, gyro, mag; public float absPressure, diffPressure, pressureAlt; }
        public struct Gps { public double lat, lon, alt; public byte fixType; }
        public struct State { public Quaternion attitude; public Vector3 velocityNed; }

        /// <summary>Answer HIL_SENSOR. Off plays a PX4 that has stopped without closing the socket.</summary>
        public volatile bool answer = true;
        /// <summary>Wait this long before each answer, like a slow PX4.</summary>
        public volatile int answerDelayMs;

        readonly TcpClient client;
        readonly NetworkStream stream;
        readonly Thread thread;
        readonly object gate = new object();
        readonly float[] controls = new float[VehicleActuators.ChannelCount];
        bool armed;
        byte seq;
        int sensors, gpsFixes, states, heartbeats;
        ulong lastSensorTime, minStep = ulong.MaxValue, maxStep;
        Sensor sensor;
        Gps gps;
        State state;

        public FakePx4(int port)
        {
            client = new TcpClient { NoDelay = true };
            client.Connect(IPAddress.Loopback, port);
            stream = client.GetStream();
            thread = new Thread(Run) { IsBackground = true, Name = "fake-px4" };
            thread.Start();
        }

        public int Sensors { get { lock (gate) return sensors; } }
        public int GpsFixes { get { lock (gate) return gpsFixes; } }
        public int States { get { lock (gate) return states; } }
        public int Heartbeats { get { lock (gate) return heartbeats; } }
        public Sensor LastSensor { get { lock (gate) return sensor; } }
        public Gps LastGps { get { lock (gate) return gps; } }
        public State LastState { get { lock (gate) return state; } }
        /// <summary>Smallest and largest time step between consecutive HIL_SENSOR (us).</summary>
        public (ulong min, ulong max) SensorSteps { get { lock (gate) return (minStep, maxStep); } }

        /// <summary>What to answer from now on: armed flag and the 16 output channels.</summary>
        public void Command(bool isArmed, float[] values)
        {
            lock (gate)
            {
                armed = isArmed;
                Array.Copy(values, controls, controls.Length);
            }
        }

        void Run()
        {
            var parser = new MavlinkParser(Msg.CrcExtraFor);
            var buf = new byte[4096];
            try
            {
                int n;
                while ((n = stream.Read(buf, 0, buf.Length)) > 0) parser.Feed(buf, n, Handle);
            }
            catch (Exception) { } // closed by Dispose or by the bridge
        }

        void Handle(uint id, byte[] trimmed)
        {
            var p = new byte[255];
            Array.Copy(trimmed, p, trimmed.Length); // MAVLink 2 drops trailing zero bytes
            switch (id)
            {
                case Msg.HilSensor:
                    ulong t = BitConverter.ToUInt64(p, 0);
                    lock (gate)
                    {
                        if (sensors > 0)
                        {
                            minStep = Math.Min(minStep, t - lastSensorTime);
                            maxStep = Math.Max(maxStep, t - lastSensorTime);
                        }
                        lastSensorTime = t;
                        sensors++;
                        sensor = new Sensor
                        {
                            timeUs = t, acc = Vec(p, 8), gyro = Vec(p, 20), mag = Vec(p, 32),
                            absPressure = F(p, 44), diffPressure = F(p, 48), pressureAlt = F(p, 52)
                        };
                    }
                    if (answer) Answer(t);
                    break;
                case Msg.HilGps:
                    lock (gate)
                    {
                        gpsFixes++;
                        gps = new Gps
                        {
                            lat = BitConverter.ToInt32(p, 8) / 1e7, lon = BitConverter.ToInt32(p, 12) / 1e7,
                            alt = BitConverter.ToInt32(p, 16) / 1e3, fixType = p[34]
                        };
                    }
                    break;
                case Msg.HilStateQuaternion:
                    lock (gate)
                    {
                        states++;
                        state = new State
                        {
                            attitude = new Quaternion(F(p, 12), F(p, 16), F(p, 20), F(p, 8)),
                            velocityNed = new Vector3(BitConverter.ToInt16(p, 48), BitConverter.ToInt16(p, 50), BitConverter.ToInt16(p, 52)) / 100f
                        };
                    }
                    break;
                case Msg.Heartbeat:
                    lock (gate) heartbeats++;
                    break;
            }
        }

        void Answer(ulong timeUs)
        {
            if (answerDelayMs > 0) Thread.Sleep(answerDelayMs);
            byte[] frame;
            lock (gate)
            {
                // flags bit 0: lockstep, as PX4 sets it. mode 0x80: MAV_MODE_FLAG_SAFETY_ARMED.
                var w = new PayloadWriter(81).U64(timeUs).U64(1);
                foreach (float c in controls) w.F32(c);
                w.U8((byte)(armed ? 0x80 : 0));
                frame = MavlinkCodec.Pack(Msg.HilActuatorControls, Msg.HilActuatorControlsCrc, w.Bytes, seq++, 1, 1);
            }
            stream.Write(frame, 0, frame.Length);
        }

        static float F(byte[] p, int offset) => BitConverter.ToSingle(p, offset);
        static Vector3 Vec(byte[] p, int offset) => new Vector3(F(p, offset), F(p, offset + 4), F(p, offset + 8));

        public void Dispose()
        {
            try { client.Close(); } catch { }
            thread.Join(1000);
        }
    }
}
