using System.Collections.Generic;
using Bridge;
using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    /// <summary>
    /// Golden frames were generated with pymavlink 2 (common.xml, MAVLink v2, sys 1, comp 51)
    /// from the same field values, so these pin our hand-written layouts to the reference encoder.
    /// </summary>
    public class MavlinkCodecTests
    {
        const byte Sys = 1, Comp = 51;

        static readonly byte[] HeartbeatGolden =
        {
            0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x03, 0x0D, 0x2F
        };

        static readonly byte[] HilSensorGolden =
        {
            0xFD, 0x3E, 0x00, 0x00, 0x01, 0x01, 0x33, 0x6B, 0x00, 0x00, 0x15, 0xCD, 0x5B, 0x07, 0x00, 0x00,
            0x00, 0x00, 0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0x4C, 0xBE, 0xCD, 0xCC, 0x1C, 0xC1, 0x0A, 0xD7,
            0x23, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x8F, 0xC2, 0xF5, 0xBC, 0x3D, 0x0A, 0x57, 0x3E, 0x42, 0x60,
            0xE5, 0xBB, 0x66, 0x66, 0xE6, 0x3E, 0x00, 0xA0, 0x77, 0x44, 0x00, 0x00, 0xA0, 0x3F, 0x00, 0x00,
            0x52, 0x43, 0x00, 0x00, 0xC8, 0x41, 0xFF, 0x1F, 0x0E, 0x9A
        };

        static readonly byte[] HilGpsGolden =
        {
            0xFD, 0x27, 0x00, 0x00, 0x02, 0x01, 0x33, 0x71, 0x00, 0x00, 0x15, 0xCD, 0x5B, 0x07, 0x00, 0x00,
            0x00, 0x00, 0x50, 0xAA, 0x95, 0x15, 0x70, 0x30, 0xC6, 0xC6, 0x50, 0x34, 0x03, 0x00, 0x64, 0x00,
            0x64, 0x00, 0x00, 0x02, 0x2C, 0x01, 0x70, 0xFE, 0x32, 0x00, 0x28, 0x23, 0x03, 0x0C, 0x00, 0x50,
            0x46, 0xA0, 0xEB
        };

        static readonly byte[] HilStateQuaternionGolden =
        {
            0xFD, 0x40, 0x00, 0x00, 0x03, 0x01, 0x33, 0x73, 0x00, 0x00, 0x15, 0xCD, 0x5B, 0x07, 0x00, 0x00,
            0x00, 0x00, 0x66, 0x66, 0x66, 0x3F, 0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0x4C, 0xBE, 0x9A, 0x99,
            0x99, 0x3E, 0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0x4C, 0xBE, 0x9A, 0x99, 0x99, 0x3E, 0x50, 0xAA,
            0x95, 0x15, 0x70, 0x30, 0xC6, 0xC6, 0x50, 0x34, 0x03, 0x00, 0x64, 0x00, 0x38, 0xFF, 0x1E, 0x00,
            0xDC, 0x05, 0xF0, 0x05, 0x0A, 0x00, 0xEC, 0xFF, 0x18, 0xFC, 0x07, 0xC0
        };

        static readonly byte[] HilActuatorControlsGolden =
        {
            0xFD, 0x51, 0x00, 0x00, 0x04, 0x01, 0x33, 0x5D, 0x00, 0x00, 0xB1, 0x68, 0xDE, 0x3A, 0x00, 0x00,
            0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00,
            0x80, 0x3E, 0x00, 0x00, 0x80, 0xBF, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xAC, 0x4F
        };

        [Test]
        public void Crc_MatchesMcrf4xxCheckValue()
        {
            ushort crc = 0xFFFF;
            foreach (char c in "123456789") crc = MavlinkCodec.CrcAccumulate((byte)c, crc);
            Assert.AreEqual(0x6F91, crc);
        }

        [Test]
        public void Heartbeat_MatchesPymavlink()
        {
            var frame = MavlinkCodec.Pack(Msg.Heartbeat, Msg.HeartbeatCrc, Msg.Heartbeat_(0, 0, 0, 0, 0), 0, Sys, Comp);
            CollectionAssert.AreEqual(HeartbeatGolden, frame);
        }

        [Test]
        public void HilSensor_MatchesPymavlink()
        {
            var payload = Msg.HilSensor_(123456789,
                new Vector3(0.1f, -0.2f, -9.8f), new Vector3(0.01f, 0.02f, -0.03f), new Vector3(0.21f, -0.007f, 0.45f),
                990.5f, 1.25f, 210f, 25f, 0x1FFF, 0);
            Assert.AreEqual(65, payload.Length);
            CollectionAssert.AreEqual(HilSensorGolden, MavlinkCodec.Pack(Msg.HilSensor, Msg.HilSensorCrc, payload, 1, Sys, Comp));
        }

        [Test]
        public void HilGps_MatchesPymavlink()
        {
            var payload = Msg.HilGps_(123456789, 362130000, -960090000, 210000, 100, 100, 512,
                300, -400, 50, 9000, 3, 12, 0, 18000);
            Assert.AreEqual(39, payload.Length);
            CollectionAssert.AreEqual(HilGpsGolden, MavlinkCodec.Pack(Msg.HilGps, Msg.HilGpsCrc, payload, 2, Sys, Comp));
        }

        [Test]
        public void HilStateQuaternion_MatchesPymavlink()
        {
            var payload = Msg.HilStateQuaternion_(123456789, 0.9f, 0.1f, -0.2f, 0.3f, 0.1f, -0.2f, 0.3f,
                362130000, -960090000, 210000, 100, -200, 30, 1500, 1520, 10, -20, -1000);
            Assert.AreEqual(64, payload.Length);
            CollectionAssert.AreEqual(HilStateQuaternionGolden,
                MavlinkCodec.Pack(Msg.HilStateQuaternion, Msg.HilStateQuaternionCrc, payload, 3, Sys, Comp));
        }

        [Test]
        public void HilActuatorControls_DecodesPymavlinkFrame()
        {
            var msgs = ParseAll(HilActuatorControlsGolden);
            Assert.AreEqual(1, msgs.Count);
            Assert.AreEqual(Msg.HilActuatorControls, msgs[0].id);

            var controls = new float[16];
            Msg.DecodeHilActuatorControls(msgs[0].payload, controls, out ulong t, out ulong flags, out byte mode);
            Assert.AreEqual(987654321UL, t);
            Assert.AreEqual(1UL, flags);
            Assert.AreEqual(0x80, mode);
            CollectionAssert.AreEqual(new[] { 0.5f, 0.25f, -1f, 1f }, new[] { controls[0], controls[1], controls[2], controls[3] });
            for (int i = 4; i < 16; i++) Assert.AreEqual(0f, controls[i]);
        }

        [Test]
        public void Decode_ZeroTrimmedPayload_FillsZeros()
        {
            // Disarmed PX4 sends all-zero controls; the v2 sender trims them away.
            var full = new PayloadWriter(81).U64(5).U64(0).Bytes;
            var frame = MavlinkCodec.Pack(Msg.HilActuatorControls, Msg.HilActuatorControlsCrc, full, 0, Sys, Comp);
            Assert.Less(frame.Length, 12 + 81);

            var msgs = ParseAll(frame);
            var controls = new float[16];
            for (int i = 0; i < 16; i++) controls[i] = 9f;
            Msg.DecodeHilActuatorControls(msgs[0].payload, controls, out ulong t, out _, out byte mode);
            Assert.AreEqual(5UL, t);
            Assert.AreEqual(0, mode);
            foreach (var c in controls) Assert.AreEqual(0f, c);
        }

        [Test]
        public void Parser_ResyncsAfterGarbageAndBadCrc()
        {
            var good = HilActuatorControlsGolden;
            var corrupt = (byte[])good.Clone();
            corrupt[20] ^= 0xFF;

            var stream = new List<byte> { 0x00, 0x12, MavlinkCodec.Stx, 0x01 };
            stream.AddRange(corrupt);
            stream.AddRange(good);

            var msgs = ParseAll(stream.ToArray());
            Assert.AreEqual(1, msgs.Count);
            Assert.AreEqual(Msg.HilActuatorControls, msgs[0].id);
        }

        [Test]
        public void Parser_HandlesByteAtATimeDelivery()
        {
            var parser = new MavlinkParser(Msg.CrcExtraFor);
            int count = 0;
            var one = new byte[1];
            for (int rep = 0; rep < 3; rep++)
                foreach (var b in HilActuatorControlsGolden)
                {
                    one[0] = b;
                    parser.Feed(one, 1, (id, p) => count++);
                }
            Assert.AreEqual(3, count);
        }

        [Test]
        public void Parser_SkipsUnknownMessages()
        {
            var unknown = MavlinkCodec.Pack(30, 39, new byte[28] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28 }, 0, Sys, Comp);
            var stream = new List<byte>(unknown);
            stream.AddRange(HilActuatorControlsGolden);
            var msgs = ParseAll(stream.ToArray());
            Assert.AreEqual(1, msgs.Count);
            Assert.AreEqual(Msg.HilActuatorControls, msgs[0].id);
        }

        static List<(uint id, byte[] payload)> ParseAll(byte[] bytes)
        {
            var parser = new MavlinkParser(Msg.CrcExtraFor);
            var msgs = new List<(uint, byte[])>();
            parser.Feed(bytes, bytes.Length, (id, p) => msgs.Add((id, p)));
            return msgs;
        }
    }
}
