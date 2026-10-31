#!/usr/bin/env python3
"""Smoke-test a running sim from PX4's side of the HIL link, without PX4.

Connects to the sim's PX4 port the way PX4 SITL does, answers every HIL_SENSOR with
HIL_ACTUATOR_CONTROLS as PX4 does in lockstep, and checks that:

  1. a vehicle at rest reads sensibly: gravity on the accelerometer, a still gyro, a plausible
     magnetometer and barometer, a 3D GPS fix, one HIL_SENSOR per physics step;
  2. full throttle on the lift rotors does nothing while disarmed;
  3. armed, it climbs.

    ./drone-sim.x86_64 -batchmode -nographics --no-ros --vehicle x500 &
    python3 tools/sim_smoke_test.py x500

The vehicle (a name in Assets/StreamingAssets/vehicles or a path to a .json file) says which
channels are lift rotors. Exit status: 0 pass, 1 no connection or the link dropped, 2 sensors
wrong, 3 actuators or physics wrong. Standard library only. CI runs it against the Linux build.
"""
import argparse
import json
import math
import socket
import struct
import sys
import time

import px4_airframe

HEARTBEAT, HIL_ACTUATOR_CONTROLS, HIL_SENSOR, HIL_GPS, HIL_STATE_QUATERNION = 0, 93, 107, 113, 115
CRC_EXTRA = {HEARTBEAT: 50, HIL_ACTUATOR_CONTROLS: 47, HIL_SENSOR: 108, HIL_GPS: 124, HIL_STATE_QUATERNION: 4}
CHANNELS = 16
STX = 0xFD


# MAVLink 2 framing, the same subset as MavlinkCodec.cs.

def crc(data, extra=None):
    c = 0xFFFF
    for b in bytes(data) + (bytes([extra]) if extra is not None else b""):
        t = (b ^ c) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        c = ((c >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return c


def pack(msg_id, payload, seq, sys_id=1, comp_id=1):
    """One MAVLink 2 frame. Trailing zero bytes of the payload are trimmed, as the spec says."""
    n = len(payload)
    while n > 1 and payload[n - 1] == 0:
        n -= 1
    header = struct.pack("<BBBBBBBHB", STX, n, 0, 0, seq & 0xFF, sys_id, comp_id, msg_id & 0xFFFF, msg_id >> 16)
    body = header + payload[:n]
    return body + struct.pack("<H", crc(body[1:], CRC_EXTRA[msg_id]))


class Parser:
    """Turns a byte stream into (msg_id, payload) pairs; skips garbage, bad CRCs and unknown ids."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        while True:
            start = self.buf.find(STX)
            if start < 0:
                self.buf.clear()
                return
            del self.buf[:start]
            if len(self.buf) < 12:
                return
            n = self.buf[1]
            total = 12 + n + (13 if self.buf[2] & 0x01 else 0)
            if len(self.buf) < total:
                return
            msg_id = self.buf[7] | self.buf[8] << 8 | self.buf[9] << 16
            extra = CRC_EXTRA.get(msg_id)
            if extra is None:
                del self.buf[:total]
            elif crc(self.buf[1:10 + n], extra) == struct.unpack_from("<H", self.buf, 10 + n)[0]:
                payload = bytes(self.buf[10:10 + n])
                del self.buf[:total]
                yield msg_id, payload
            else:
                del self.buf[:1]


def padded(payload, size):
    return payload + bytes(max(0, size - len(payload)))


def hil_sensor(p):
    t, *f = struct.unpack_from("<Q13f", padded(p, 65))
    return {"time_us": t, "acc": f[0:3], "gyro": f[3:6], "mag": f[6:9],
            "abs_pressure": f[9], "diff_pressure": f[10], "pressure_alt": f[11]}


def hil_gps(p):
    t, lat, lon, alt, _eph, _epv, _vel, _vn, _ve, _vd, _cog, fix, sats = struct.unpack_from("<QiiiHHHhhhHBB", padded(p, 39))
    return {"lat": lat / 1e7, "lon": lon / 1e7, "alt": alt / 1e3, "fix_type": fix, "satellites": sats}


def hil_state(p):
    v = struct.unpack_from("<Q7f3i3h", padded(p, 64))
    return {"attitude": v[1:5], "alt": v[10] / 1e3, "velocity_ned": tuple(c / 100 for c in v[11:14])}


def hil_actuator_controls(time_us, controls, armed):
    """Payload as PX4 sends it in lockstep: flags bit 0 set, mode 0x80 when armed."""
    return struct.pack("<QQ16fB", time_us, 1, *controls, 0x80 if armed else 0)


class Fail(Exception):
    def __init__(self, code, why):
        super().__init__(why)
        self.code = code


class Link:
    """PX4's end of the link: answers each HIL_SENSOR with the current controls and keeps the latest of everything."""

    def __init__(self, sock):
        self.sock = sock
        self.parser = Parser()
        self.seq = 0
        self.controls = [0.0] * CHANNELS
        self.armed = False
        self.sensors = []
        self.gps = None
        self.state = None
        self.heartbeats = 0

    def steps(self, count, timeout=30.0):
        """Plays PX4 for this many sim steps (HIL_SENSOR messages) and returns their sensor readings."""
        got = []
        deadline = time.monotonic() + timeout
        while len(got) < count:
            if time.monotonic() > deadline:
                raise Fail(1, f"only {len(got)} of {count} sim steps in {timeout:.0f} s")
            try:
                data = self.sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                raise Fail(1, "the sim closed the connection")
            for msg_id, payload in self.parser.feed(data):
                if msg_id == HIL_SENSOR:
                    s = hil_sensor(payload)
                    got.append(s)
                    out = hil_actuator_controls(s["time_us"], self.controls, self.armed)
                    self.sock.sendall(pack(HIL_ACTUATOR_CONTROLS, out, self.seq))
                    self.seq += 1
                elif msg_id == HIL_GPS:
                    self.gps = hil_gps(payload)
                elif msg_id == HIL_STATE_QUATERNION:
                    self.state = hil_state(payload)
                elif msg_id == HEARTBEAT:
                    self.heartbeats += 1
        self.sensors += got
        return got


def connect(host, port, timeout):
    deadline = time.monotonic() + timeout
    while True:
        try:
            sock = socket.create_connection((host, port), timeout=5)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(1.0)
            return sock
        except OSError:
            if time.monotonic() > deadline:
                raise Fail(1, f"nothing listening on {host}:{port} after {timeout:.0f} s")
            time.sleep(0.5)


def mean(vectors):
    return [sum(v[i] for v in vectors) / len(vectors) for i in range(3)]


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def check(ok, code, what, log):
    log(f"{'ok  ' if ok else 'FAIL'} {what}")
    if not ok:
        raise Fail(code, what)


def smoke(link, lift_channels, log=print):
    """The checks, on a connected link. Raises Fail."""
    # At rest. The first second covers a vehicle still dropping onto the ground.
    link.steps(250)
    rest = link.steps(250)
    times = [s["time_us"] for s in link.sensors]
    gaps = {b - a for a, b in zip(times, times[1:])}
    check(len(gaps) == 1 and 1000 <= next(iter(gaps)) <= 20000, 2,
          f"one HIL_SENSOR per physics step ({sorted(gaps)[:5]} us apart)", log)
    acc, gyro, mag = mean([s["acc"] for s in rest]), mean([s["gyro"] for s in rest]), mean([s["mag"] for s in rest])
    check(abs(acc[0]) < 0.3 and abs(acc[1]) < 0.3 and abs(acc[2] + 9.81) < 0.3, 2,
          f"accelerometer reads -g on z at rest ({acc[0]:.2f}, {acc[1]:.2f}, {acc[2]:.2f})", log)
    check(norm(gyro) < 0.02, 2, f"gyro still at rest ({norm(gyro):.4f} rad/s)", log)
    check(0.2 < norm(mag) < 0.7, 2, f"magnetometer sees an Earth-like field ({norm(mag):.3f} gauss)", log)
    pressure = rest[-1]["abs_pressure"]
    check(700 < pressure < 1100, 2, f"barometer plausible ({pressure:.1f} hPa)", log)
    check(link.gps is not None and link.gps["fix_type"] == 3, 2, f"GPS 3D fix ({link.gps})", log)
    check(link.state is not None and norm(link.state["velocity_ned"]) < 0.1, 2, f"at rest ({link.state})", log)
    check(link.heartbeats > 0, 2, f"heartbeats ({link.heartbeats})", log)

    # Disarmed at full throttle: nothing moves.
    link.controls = [1.0 if ch in lift_channels else 0.0 for ch in range(CHANNELS)]
    alt0 = link.state["alt"]
    link.steps(125)
    check(abs(link.state["velocity_ned"][2]) < 0.05 and abs(link.state["alt"] - alt0) < 0.02, 3,
          f"disarmed full throttle stays put ({link.state['velocity_ned'][2]:+.2f} m/s down)", log)

    # Armed: it climbs. Short, since the throttle is open loop.
    link.armed = True
    link.steps(100)
    climb = link.state["alt"] - alt0
    check(link.state["velocity_ned"][2] < -0.3 and climb > 0.03, 3,
          f"armed full throttle climbs ({-link.state['velocity_ned'][2]:.2f} m/s up, {climb:.2f} m)", log)

    link.controls, link.armed = [0.0] * CHANNELS, False
    link.steps(10)


def lift_channels(spec):
    return {r.get("channel", 0) for r in spec.get("rotors", []) if px4_airframe.unit(r.get("axis", [0, 0, -1]))[2] < -0.7}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("vehicle", help="spec name in Assets/StreamingAssets/vehicles, or a path to a .json file")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=4560)
    ap.add_argument("--connect-timeout", type=float, default=120, help="seconds to wait for the sim to listen")
    args = ap.parse_args()

    spec = json.loads(px4_airframe.resolve(args.vehicle).read_text(encoding="utf-8-sig"))
    channels = lift_channels(spec)
    try:
        if not channels:
            raise Fail(3, f"{args.vehicle} has no lift rotors")
        with connect(args.host, args.port, args.connect_timeout) as sock:
            print(f"connected to {args.host}:{args.port}, lift channels {sorted(channels)}", flush=True)
            smoke(Link(sock), channels, log=lambda line: print(line, flush=True))
    except Fail as e:
        print(f"FAIL: {e}", flush=True)
        sys.exit(e.code)
    print("PASS", flush=True)


if __name__ == "__main__":
    main()
