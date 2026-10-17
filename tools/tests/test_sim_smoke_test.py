"""Tests for tools/sim_smoke_test.py. From the repo root: python3 -m unittest discover -s tools/tests -v

The golden frames are MavlinkCodecTests.cs's, generated with pymavlink, so the script's framing is
pinned to the same reference as the sim's.
"""
import socket
import struct
import sys
import threading
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))
import sim_smoke_test as st  # noqa: E402

HIL_SENSOR_GOLDEN = bytes([
    0xFD, 0x3E, 0x00, 0x00, 0x01, 0x01, 0x33, 0x6B, 0x00, 0x00, 0x15, 0xCD, 0x5B, 0x07, 0x00, 0x00,
    0x00, 0x00, 0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0x4C, 0xBE, 0xCD, 0xCC, 0x1C, 0xC1, 0x0A, 0xD7,
    0x23, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x8F, 0xC2, 0xF5, 0xBC, 0x3D, 0x0A, 0x57, 0x3E, 0x42, 0x60,
    0xE5, 0xBB, 0x66, 0x66, 0xE6, 0x3E, 0x00, 0xA0, 0x77, 0x44, 0x00, 0x00, 0xA0, 0x3F, 0x00, 0x00,
    0x52, 0x43, 0x00, 0x00, 0xC8, 0x41, 0xFF, 0x1F, 0x0E, 0x9A])

HIL_GPS_GOLDEN = bytes([
    0xFD, 0x27, 0x00, 0x00, 0x02, 0x01, 0x33, 0x71, 0x00, 0x00, 0x15, 0xCD, 0x5B, 0x07, 0x00, 0x00,
    0x00, 0x00, 0x50, 0xAA, 0x95, 0x15, 0x70, 0x30, 0xC6, 0xC6, 0x50, 0x34, 0x03, 0x00, 0x64, 0x00,
    0x64, 0x00, 0x00, 0x02, 0x2C, 0x01, 0x70, 0xFE, 0x32, 0x00, 0x28, 0x23, 0x03, 0x0C, 0x00, 0x50,
    0x46, 0xA0, 0xEB])

HIL_STATE_QUATERNION_GOLDEN = bytes([
    0xFD, 0x40, 0x00, 0x00, 0x03, 0x01, 0x33, 0x73, 0x00, 0x00, 0x15, 0xCD, 0x5B, 0x07, 0x00, 0x00,
    0x00, 0x00, 0x66, 0x66, 0x66, 0x3F, 0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0x4C, 0xBE, 0x9A, 0x99,
    0x99, 0x3E, 0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0x4C, 0xBE, 0x9A, 0x99, 0x99, 0x3E, 0x50, 0xAA,
    0x95, 0x15, 0x70, 0x30, 0xC6, 0xC6, 0x50, 0x34, 0x03, 0x00, 0x64, 0x00, 0x38, 0xFF, 0x1E, 0x00,
    0xDC, 0x05, 0xF0, 0x05, 0x0A, 0x00, 0xEC, 0xFF, 0x18, 0xFC, 0x07, 0xC0])

HIL_ACTUATOR_CONTROLS_GOLDEN = bytes([
    0xFD, 0x51, 0x00, 0x00, 0x04, 0x01, 0x33, 0x5D, 0x00, 0x00, 0xB1, 0x68, 0xDE, 0x3A, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00,
    0x80, 0x3E, 0x00, 0x00, 0x80, 0xBF, 0x00, 0x00, 0x80, 0x3F] + [0x00] * 48 + [0x80, 0xAC, 0x4F])


def parse_all(data):
    return list(st.Parser().feed(data))


class Framing(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(0x6F91, st.crc(b"123456789"))

    def test_actuator_controls_match_pymavlink(self):
        controls = [0.5, 0.25, -1.0, 1.0] + [0.0] * 12
        payload = st.hil_actuator_controls(987654321, controls, armed=True)
        self.assertEqual(HIL_ACTUATOR_CONTROLS_GOLDEN, st.pack(st.HIL_ACTUATOR_CONTROLS, payload, 4, 1, 51))

    def test_trailing_zeros_are_trimmed(self):
        frame = st.pack(st.HIL_ACTUATOR_CONTROLS, st.hil_actuator_controls(5, [0.0] * 16, armed=False), 0)
        self.assertEqual(9, frame[1], "8 time bytes, then the low byte of flags = 1")
        [(msg_id, payload)] = parse_all(frame)
        self.assertEqual((5, 1), struct.unpack_from("<QQ", st.padded(payload, 81)))

    def test_decodes_sensor(self):
        [(msg_id, payload)] = parse_all(HIL_SENSOR_GOLDEN)
        s = st.hil_sensor(payload)
        self.assertEqual(st.HIL_SENSOR, msg_id)
        self.assertEqual(123456789, s["time_us"])
        for got, want in zip(s["acc"] + s["gyro"] + s["mag"], [0.1, -0.2, -9.8, 0.01, 0.02, -0.03, 0.21, -0.007, 0.45]):
            self.assertAlmostEqual(want, got, places=5)
        self.assertAlmostEqual(990.5, s["abs_pressure"], places=3)
        self.assertAlmostEqual(210, s["pressure_alt"], places=3)

    def test_decodes_gps(self):
        [(_, payload)] = parse_all(HIL_GPS_GOLDEN)
        g = st.hil_gps(payload)
        self.assertEqual({"lat": 36.213, "lon": -96.009, "alt": 210.0, "fix_type": 3, "satellites": 12}, g)

    def test_decodes_state(self):
        [(_, payload)] = parse_all(HIL_STATE_QUATERNION_GOLDEN)
        s = st.hil_state(payload)
        for got, want in zip(s["attitude"], [0.9, 0.1, -0.2, 0.3]):
            self.assertAlmostEqual(want, got, places=5)
        self.assertEqual(210.0, s["alt"])
        self.assertEqual((1.0, -2.0, 0.3), s["velocity_ned"])

    def test_parser_resyncs_after_garbage_and_a_bad_crc(self):
        corrupt = bytearray(HIL_ACTUATOR_CONTROLS_GOLDEN)
        corrupt[20] ^= 0xFF
        msgs = parse_all(b"\x00\x12\xfd\x01" + bytes(corrupt) + HIL_ACTUATOR_CONTROLS_GOLDEN)
        self.assertEqual([st.HIL_ACTUATOR_CONTROLS], [m for m, _ in msgs])

    def test_parser_handles_a_byte_at_a_time_and_skips_unknown_ids(self):
        parser = st.Parser()
        unknown = bytearray(HIL_GPS_GOLDEN)
        unknown[7] = 30  # ATTITUDE, which the script doesn't know
        stream = bytes(unknown) + HIL_SENSOR_GOLDEN * 2
        ids = [m for i in range(len(stream)) for m, _ in parser.feed(stream[i:i + 1])]
        self.assertEqual([st.HIL_SENSOR] * 2, ids)


class FakeSim:
    """The sim's end, crudely: a vehicle at rest that climbs at 5 m/s^2 when armed with the lift
    channels above half throttle. Lockstep like the bridge: one HIL_SENSOR, then wait for controls."""

    def __init__(self, lift=(0, 1, 2, 3), gravity_sign=-1, motors_work=True, step_us=4000):
        self.lift, self.gravity_sign, self.motors_work, self.step_us = lift, gravity_sign, motors_work, step_us
        self.server = socket.socket()
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.port = self.server.getsockname()[1]
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        conn, _ = self.server.accept()
        parser, seq, alt, vz, t = st.Parser(), 0, 30.0, 0.0, 0
        controls, armed = [0.0] * 16, False
        try:
            for step in range(100000):
                t += self.step_us
                dt = self.step_us / 1e6
                if armed and self.motors_work and min(controls[c] for c in self.lift) > 0.5:
                    vz += 5 * dt
                alt += vz * dt
                out = []
                if step % 250 == 0:
                    out.append((st.HEARTBEAT, struct.pack("<IBBBBB", 0, 0, 0, 0, 0, 3)))
                if step % 25 == 0:
                    out.append((st.HIL_GPS, struct.pack("<QiiiHHHhhhHBBBH", t, 455048000, -735772000, int(alt * 1000),
                                                        100, 100, 0, 0, 0, int(-vz * 100), 0, 3, 12, 0, 1)))
                if step % 5 == 0:
                    out.append((st.HIL_STATE_QUATERNION, struct.pack("<Q7f3i3h2H3h", t, 1, 0, 0, 0, 0, 0, 0, 455048000,
                                                                     -735772000, int(alt * 1000), 0, 0, int(-vz * 100), 0, 0, 0, 0, 0)))
                out.append((st.HIL_SENSOR, struct.pack("<Q13fIB", t, 0.01, -0.02, self.gravity_sign * 9.81, 0.001, 0, 0,
                                                       0.2, -0.05, 0.4, 1009.5, 0, alt, 25, 0x1FFF, 0)))
                for msg_id, payload in out:
                    conn.sendall(st.pack(msg_id, payload, seq, 1, 51))
                    seq += 1
                answered = False
                while not answered:
                    data = conn.recv(4096)
                    if not data:
                        return
                    for msg_id, payload in parser.feed(data):
                        if msg_id == st.HIL_ACTUATOR_CONTROLS:
                            v = struct.unpack_from("<QQ16fB", st.padded(payload, 81))
                            controls, armed, answered = list(v[2:18]), bool(v[18] & 0x80), True
        except OSError:
            pass
        finally:
            conn.close()
            self.server.close()


class Smoke(unittest.TestCase):
    def run_smoke(self, sim):
        sock = st.connect("127.0.0.1", sim.port, timeout=5)
        lines = []
        try:
            st.smoke(st.Link(sock), set(sim.lift), log=lines.append)
            return 0, lines
        except st.Fail as e:
            return e.code, lines
        finally:
            sock.close()
            sim.thread.join(5)

    def test_a_working_sim_passes(self):
        code, lines = self.run_smoke(FakeSim())
        self.assertEqual(0, code, "\n".join(lines))
        self.assertTrue(all(line.startswith("ok") for line in lines))

    def test_upside_down_gravity_fails_the_sensor_checks(self):
        code, lines = self.run_smoke(FakeSim(gravity_sign=+1))
        self.assertEqual(2, code)
        self.assertIn("accelerometer", lines[-1])

    def test_motors_that_do_nothing_fail_the_climb(self):
        code, lines = self.run_smoke(FakeSim(motors_work=False))
        self.assertEqual(3, code)
        self.assertIn("climbs", lines[-1])

    def test_nothing_listening_is_exit_1(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        with self.assertRaises(st.Fail) as e:
            st.connect("127.0.0.1", port, timeout=0.5)
        self.assertEqual(1, e.exception.code)

    def test_lift_channels_come_from_the_spec(self):
        spec = {"rotors": [{"channel": 0}, {"channel": 4, "axis": [1, 0, 0]}, {"channel": 7, "axis": [0, 0, -2]}]}
        self.assertEqual({0, 7}, st.lift_channels(spec))


if __name__ == "__main__":
    unittest.main()
