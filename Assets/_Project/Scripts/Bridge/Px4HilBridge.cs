using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using Utils;

namespace Bridge
{
    /// <summary>
    /// Makes Unity the physics plant for PX4 SITL over the simulator MAVLink API.
    /// Listens on TCP 4560. Each physics step sends HIL_SENSOR (plus HIL_GPS and
    /// HIL_STATE_QUATERNION at lower rates), then in lockstep waits for HIL_ACTUATOR_CONTROLS
    /// before the next step. Frames: PX4 wants body FRD and world NED; Unity is x right, y up,
    /// z forward with z as north. Conversions are in the helpers at the bottom.
    ///
    /// Start PX4 with:  PX4_SIM_MODEL=none_standard_vtol make px4_sitl none   (or your airframe)
    /// </summary>
    [DefaultExecutionOrder(-10)]
    public class Px4HilBridge : MonoBehaviour, IResettable
    {
        [Header("Connection")]
        public int port = 4560;
        [Tooltip("Block each physics step until PX4 returns actuator controls")]
        public bool lockstep = true;
        [Tooltip("Give up waiting for PX4 after this long (ms) and step anyway")]
        public int lockstepTimeoutMs = 1000;

        [Header("Vehicle")]
        public ImuModel imu;
        public VehicleActuators actuators;
        public Wing wing;

        [Header("Rates (Hz)")]
        public float gpsRate = 10f;
        public float stateRate = 50f;

        [Header("GPS noise")]
        public float gpsPositionNoise = 0.3f;
        public float gpsVelocityNoise = 0.05f;

        [Header("Baro")]
        public float baroNoiseHpa = 0.02f;
        public float temperatureC = 25f;

        public bool Connected => client != null && client.Connected;
        public bool ReceivedControls { get; private set; }
        public ulong LastControlsTimeUs { get; private set; }

        private TcpListener listener;
        private TcpClient client;
        private NetworkStream stream;
        private Thread acceptThread, readThread;
        private volatile bool running;
        private readonly AutoResetEvent controlsEvent = new AutoResetEvent(false);
        private readonly object controlsLock = new object();
        private readonly float[] pendingControls = new float[16];
        private readonly float[] controls = new float[16];
        private bool pendingArmed, hasPending;

        private Rigidbody rb;
        private ulong timeUs;
        private byte seq;
        private double gpsAccum, stateAccum, heartbeatAccum;
        private const byte SysId = 1, CompId = 51;

        private void Start()
        {
            rb = SimSettings.Vehicle;
            if (imu == null && rb != null) imu = rb.GetComponentInChildren<ImuModel>();
            if (actuators == null && rb != null) actuators = rb.GetComponentInChildren<VehicleActuators>();
            if (wing == null && rb != null) wing = rb.GetComponentInChildren<Wing>();
            if (rb == null || imu == null || actuators == null)
            {
                Debug.LogError("[PX4] Bridge needs a vehicle rigidbody with ImuModel and VehicleActuators");
                enabled = false;
                return;
            }

            running = true;
            listener = new TcpListener(IPAddress.Any, port);
            listener.Start();
            acceptThread = new Thread(AcceptLoop) { IsBackground = true, Name = "px4-accept" };
            acceptThread.Start();
            Debug.Log($"[PX4] Waiting for PX4 SITL on TCP {port}");
        }

        private void AcceptLoop()
        {
            while (running)
            {
                try
                {
                    var c = listener.AcceptTcpClient();
                    c.NoDelay = true;
                    var old = client;
                    client = c;
                    stream = c.GetStream();
                    old?.Close();
                    Debug.Log("[PX4] SITL connected");
                    readThread = new Thread(() => ReadLoop(c)) { IsBackground = true, Name = "px4-read" };
                    readThread.Start();
                }
                catch (Exception) { if (running) Thread.Sleep(100); }
            }
        }

        private void ReadLoop(TcpClient c)
        {
            var parser = new MavlinkParser(Msg.CrcExtraFor);
            var buf = new byte[4096];
            var tmp = new float[16];
            try
            {
                var s = c.GetStream();
                while (running && c.Connected)
                {
                    int n = s.Read(buf, 0, buf.Length);
                    if (n <= 0) break;
                    parser.Feed(buf, n, (id, payload) =>
                    {
                        if (id != Msg.HilActuatorControls) return;
                        Msg.DecodeHilActuatorControls(payload, tmp, out ulong t, out ulong flags, out byte mode);
                        lock (controlsLock)
                        {
                            Array.Copy(tmp, pendingControls, 16);
                            pendingArmed = (mode & 0x80) != 0;
                            LastControlsTimeUs = t;
                            hasPending = true;
                        }
                        controlsEvent.Set();
                    });
                }
            }
            catch (Exception) { }
            if (client == c) { Debug.Log("[PX4] SITL disconnected"); ReceivedControls = false; }
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            timeUs += (ulong)(dt * 1e6);

            if (!Connected) { ApplyControls(); return; }

            heartbeatAccum += dt; gpsAccum += dt; stateAccum += dt;
            if (heartbeatAccum >= 1.0) { heartbeatAccum = 0; Send(Msg.Heartbeat, Msg.HeartbeatCrc, Msg.Heartbeat_(0, 0, 0, 0, 0)); }
            if (gpsAccum >= 1.0 / gpsRate) { gpsAccum = 0; SendGps(); }
            if (stateAccum >= 1.0 / stateRate) { stateAccum = 0; SendState(); }
            SendSensor();

            if (lockstep && ReceivedControls)
            {
                if (!controlsEvent.WaitOne(lockstepTimeoutMs))
                    Debug.LogWarning("[PX4] lockstep timeout, stepping without new controls");
            }
            else if (!ReceivedControls && hasPending)
            {
                ReceivedControls = true;
                Debug.Log("[PX4] first actuator controls received, lockstep engaged");
            }

            ApplyControls();
        }

        private void ApplyControls()
        {
            if (ManualControl.Instance != null && ManualControl.Instance.Active) return;
            lock (controlsLock)
            {
                if (hasPending) { Array.Copy(pendingControls, controls, 16); hasPending = false; }
                actuators.SetControls(controls, pendingArmed && Connected);
            }
        }

        private void SendSensor()
        {
            Vector3 accFrd = BodyToFrd(imu.Acceleration);
            Vector3 gyroFrd = RateToFrd(imu.AngularRate);

            Vector3 magWorld = GeoOrigin.Instance != null ? GeoOrigin.Instance.MagneticFieldUnity : new Vector3(0, -0.4f, 0.2f);
            Vector3 magFrd = BodyToFrd(Quaternion.Inverse(imu.Orientation) * magWorld);

            GeoOrigin.Instance.ToGeodetic(rb.position, out _, out _, out double altMsl);
            float absP = PressureAtAltitude((float)altMsl) + Stochastic.GenerateGaussian() * baroNoiseHpa;
            float tas = wing != null ? wing.Airspeed : rb.linearVelocity.magnitude;
            float diffP = 0.5f * 1.225f * tas * tas / 100f;

            Send(Msg.HilSensor, Msg.HilSensorCrc,
                Msg.HilSensor_(timeUs, accFrd, gyroFrd, magFrd, absP, diffP, (float)altMsl, temperatureC, 0x1FFF, 0));
        }

        private void SendGps()
        {
            Vector3 p = rb.position + Stochastic.GenerateWhiteNoiseVector(gpsPositionNoise);
            GeoOrigin.Instance.ToGeodetic(p, out double lat, out double lon, out double alt);
            Vector3 v = rb.linearVelocity + Stochastic.GenerateWhiteNoiseVector(gpsVelocityNoise);
            float vn = v.z, ve = v.x, vd = -v.y;
            float cog = Mathf.Atan2(ve, vn) * Mathf.Rad2Deg; if (cog < 0) cog += 360f;
            float yaw = rb.rotation.eulerAngles.y; // Unity yaw about up, clockwise from north = compass heading
            ushort yawCdeg = (ushort)Mathf.Clamp(Mathf.RoundToInt(yaw * 100f), 1, 36000);

            Send(Msg.HilGps, Msg.HilGpsCrc, Msg.HilGps_(timeUs,
                (int)(lat * 1e7), (int)(lon * 1e7), (int)(alt * 1000),
                100, 100, (ushort)(new Vector2(vn, ve).magnitude * 100f),
                (short)(vn * 100f), (short)(ve * 100f), (short)(vd * 100f),
                (ushort)(cog * 100f), 3, 12, 0, yawCdeg));
        }

        private void SendState()
        {
            GeoOrigin.Instance.ToGeodetic(rb.position, out double lat, out double lon, out double alt);
            Quaternion q = rb.rotation;
            Vector3 w = RateToFrd(rb.transform.InverseTransformDirection(rb.angularVelocity));
            Vector3 v = rb.linearVelocity;
            Vector3 accFrd = BodyToFrd(imu.Acceleration) / 9.80665f * 1000f;
            float tas = wing != null ? wing.Airspeed : v.magnitude;

            Send(Msg.HilStateQuaternion, Msg.HilStateQuaternionCrc, Msg.HilStateQuaternion_(timeUs,
                q.w, -q.z, -q.x, q.y, w.x, w.y, w.z,
                (int)(lat * 1e7), (int)(lon * 1e7), (int)(alt * 1000),
                (short)(v.z * 100f), (short)(v.x * 100f), (short)(-v.y * 100f),
                (ushort)(tas * 100f), (ushort)(tas * 100f),
                (short)accFrd.x, (short)accFrd.y, (short)accFrd.z));
        }

        private void Send(uint id, byte crc, byte[] payload)
        {
            var s = stream;
            if (s == null) return;
            try { s.Write(MavlinkCodec.Pack(id, crc, payload, seq++, SysId, CompId)); }
            catch (Exception) { }
        }

        /// <summary>Unity body (x right, y up, z forward) to FRD (x forward, y right, z down).</summary>
        public static Vector3 BodyToFrd(Vector3 v) => new Vector3(v.z, v.x, -v.y);
        /// <summary>Angular rate is a pseudovector: axis swap then handedness flip.</summary>
        public static Vector3 RateToFrd(Vector3 w) => new Vector3(-w.z, -w.x, w.y);

        public static float PressureAtAltitude(float hMeters)
            => 1013.25f * Mathf.Pow(1f - 2.25577e-5f * hMeters, 5.25588f);

        public void ResetState()
        {
            lock (controlsLock) { Array.Clear(controls, 0, 16); Array.Clear(pendingControls, 0, 16); hasPending = false; pendingArmed = false; }
        }

        private void OnDestroy()
        {
            running = false;
            try { listener?.Stop(); } catch { }
            try { client?.Close(); } catch { }
        }

        private void OnGUI()
        {
            GUI.Label(new Rect(10, 10, 600, 20), Connected
                ? $"PX4 SITL: connected  lockstep={(ReceivedControls ? "on" : "waiting")}  armed={actuators.armed}  t={timeUs / 1e6:F1}s"
                : $"PX4 SITL: waiting on TCP {port}");
        }
    }
}
