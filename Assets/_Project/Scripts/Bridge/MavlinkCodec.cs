using System;
using System.Collections.Generic;

namespace Bridge
{
    /// <summary>
    /// Minimal MAVLink v2 framing: enough to speak the PX4 simulator HIL API without a
    /// generated library. Only the five messages the bridge needs are defined.
    /// </summary>
    public static class MavlinkCodec
    {
        public const byte Stx = 0xFD;

        public static ushort CrcAccumulate(byte b, ushort crc)
        {
            byte tmp = (byte)(b ^ (byte)(crc & 0xff));
            tmp ^= (byte)(tmp << 4);
            return (ushort)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
        }

        /// <summary>Frame a payload. Trailing zero bytes are trimmed per the v2 spec.</summary>
        public static byte[] Pack(uint msgId, byte crcExtra, byte[] payload, byte seq, byte sysId, byte compId)
        {
            int len = payload.Length;
            while (len > 1 && payload[len - 1] == 0) len--;

            var frame = new byte[10 + len + 2];
            frame[0] = Stx;
            frame[1] = (byte)len;
            frame[2] = 0; // incompat flags
            frame[3] = 0; // compat flags
            frame[4] = seq;
            frame[5] = sysId;
            frame[6] = compId;
            frame[7] = (byte)(msgId & 0xff);
            frame[8] = (byte)((msgId >> 8) & 0xff);
            frame[9] = (byte)((msgId >> 16) & 0xff);
            Array.Copy(payload, 0, frame, 10, len);

            ushort crc = 0xFFFF;
            for (int i = 1; i < 10 + len; i++) crc = CrcAccumulate(frame[i], crc);
            crc = CrcAccumulate(crcExtra, crc);
            frame[10 + len] = (byte)(crc & 0xff);
            frame[11 + len] = (byte)(crc >> 8);
            return frame;
        }
    }

    /// <summary>Incremental parser for a MAVLink v2 byte stream. Yields (msgId, payload) pairs.</summary>
    public class MavlinkParser
    {
        private readonly List<byte> buf = new List<byte>(512);
        private readonly Func<uint, byte?> crcExtraLookup;

        public MavlinkParser(Func<uint, byte?> crcExtraLookup) => this.crcExtraLookup = crcExtraLookup;

        public void Feed(byte[] data, int count, Action<uint, byte[]> onMessage)
        {
            for (int i = 0; i < count; i++) buf.Add(data[i]);

            while (true)
            {
                int start = buf.IndexOf(MavlinkCodec.Stx);
                if (start < 0) { buf.Clear(); return; }
                if (start > 0) buf.RemoveRange(0, start);
                if (buf.Count < 12) return;

                int len = buf[1];
                bool signed = (buf[2] & 0x01) != 0;
                int total = 12 + len + (signed ? 13 : 0);
                if (buf.Count < total) return;

                uint msgId = (uint)(buf[7] | (buf[8] << 8) | (buf[9] << 16));
                byte? extra = crcExtraLookup(msgId);
                bool ok = false;
                if (extra.HasValue)
                {
                    ushort crc = 0xFFFF;
                    for (int i = 1; i < 10 + len; i++) crc = MavlinkCodec.CrcAccumulate(buf[i], crc);
                    crc = MavlinkCodec.CrcAccumulate(extra.Value, crc);
                    ushort got = (ushort)(buf[10 + len] | (buf[11 + len] << 8));
                    ok = crc == got;
                }

                if (ok)
                {
                    var payload = buf.GetRange(10, len).ToArray();
                    onMessage(msgId, payload);
                    buf.RemoveRange(0, total);
                }
                else if (extra.HasValue)
                {
                    buf.RemoveAt(0); // bad crc, resync
                }
                else
                {
                    buf.RemoveRange(0, total); // unknown message, skip whole frame
                }
            }
        }
    }

    /// <summary>Little-endian payload writer.</summary>
    public class PayloadWriter
    {
        private readonly byte[] b;
        private int o;
        public PayloadWriter(int size) => b = new byte[size];
        public byte[] Bytes => b;
        public PayloadWriter U8(byte v) { b[o++] = v; return this; }
        public PayloadWriter U16(ushort v) { b[o++] = (byte)v; b[o++] = (byte)(v >> 8); return this; }
        public PayloadWriter I16(short v) => U16((ushort)v);
        public PayloadWriter U32(uint v) { for (int i = 0; i < 4; i++) b[o++] = (byte)(v >> (8 * i)); return this; }
        public PayloadWriter I32(int v) => U32((uint)v);
        public PayloadWriter U64(ulong v) { for (int i = 0; i < 8; i++) b[o++] = (byte)(v >> (8 * i)); return this; }
        public PayloadWriter F32(float v) { var x = BitConverter.GetBytes(v); Array.Copy(x, 0, b, o, 4); o += 4; return this; }
    }

    /// <summary>Message ids and CRC extras from common.xml.</summary>
    public static class Msg
    {
        public const uint Heartbeat = 0; public const byte HeartbeatCrc = 50;
        public const uint HilActuatorControls = 93; public const byte HilActuatorControlsCrc = 47;
        public const uint HilSensor = 107; public const byte HilSensorCrc = 108;
        public const uint HilGps = 113; public const byte HilGpsCrc = 124;
        public const uint HilStateQuaternion = 115; public const byte HilStateQuaternionCrc = 4;

        public static byte? CrcExtraFor(uint id) => id switch
        {
            Heartbeat => HeartbeatCrc,
            HilActuatorControls => HilActuatorControlsCrc,
            HilSensor => HilSensorCrc,
            HilGps => HilGpsCrc,
            HilStateQuaternion => HilStateQuaternionCrc,
            _ => null
        };

        public static byte[] Heartbeat_(uint customMode, byte type, byte autopilot, byte baseMode, byte status)
            => new PayloadWriter(9).U32(customMode).U8(type).U8(autopilot).U8(baseMode).U8(status).U8(3).Bytes;

        /// <summary>HIL_SENSOR: body FRD, accel m/s^2, gyro rad/s, mag gauss, pressure hPa.</summary>
        public static byte[] HilSensor_(ulong tUs, UnityEngine.Vector3 acc, UnityEngine.Vector3 gyro, UnityEngine.Vector3 mag,
            float absPressure, float diffPressure, float pressureAlt, float temperature, uint fieldsUpdated, byte id)
            => new PayloadWriter(65).U64(tUs)
                .F32(acc.x).F32(acc.y).F32(acc.z)
                .F32(gyro.x).F32(gyro.y).F32(gyro.z)
                .F32(mag.x).F32(mag.y).F32(mag.z)
                .F32(absPressure).F32(diffPressure).F32(pressureAlt).F32(temperature)
                .U32(fieldsUpdated).U8(id).Bytes;

        /// <summary>HIL_GPS: lat/lon 1e7 deg, alt mm AMSL, eph/epv cm, vel cm/s, vn/ve/vd cm/s, cog cdeg.</summary>
        public static byte[] HilGps_(ulong tUs, int lat, int lon, int alt, ushort eph, ushort epv, ushort vel,
            short vn, short ve, short vd, ushort cog, byte fixType, byte sats, byte id, ushort yawCdeg)
            => new PayloadWriter(39).U64(tUs).I32(lat).I32(lon).I32(alt).U16(eph).U16(epv).U16(vel)
                .I16(vn).I16(ve).I16(vd).U16(cog).U8(fixType).U8(sats).U8(id).U16(yawCdeg).Bytes;

        /// <summary>HIL_STATE_QUATERNION: q wxyz NED->FRD, rates FRD, lat/lon 1e7, alt mm, vel NED cm/s, airspeed cm/s, acc mG.</summary>
        public static byte[] HilStateQuaternion_(ulong tUs, float qw, float qx, float qy, float qz,
            float p, float q, float r, int lat, int lon, int alt, short vx, short vy, short vz,
            ushort indAirspeed, ushort trueAirspeed, short xacc, short yacc, short zacc)
            => new PayloadWriter(64).U64(tUs).F32(qw).F32(qx).F32(qy).F32(qz).F32(p).F32(q).F32(r)
                .I32(lat).I32(lon).I32(alt).I16(vx).I16(vy).I16(vz).U16(indAirspeed).U16(trueAirspeed)
                .I16(xacc).I16(yacc).I16(zacc).Bytes;

        /// <summary>Decode HIL_ACTUATOR_CONTROLS. Payload may be zero-trimmed.</summary>
        public static void DecodeHilActuatorControls(byte[] payload, float[] controls, out ulong tUs, out ulong flags, out byte mode)
        {
            var p = new byte[81];
            Array.Copy(payload, p, Math.Min(payload.Length, 81));
            tUs = BitConverter.ToUInt64(p, 0);
            flags = BitConverter.ToUInt64(p, 8);
            for (int i = 0; i < 16; i++) controls[i] = BitConverter.ToSingle(p, 16 + 4 * i);
            mode = p[80];
        }
    }
}
