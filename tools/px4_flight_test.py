"""Fly the sim through PX4 SITL's MAVLink API port (UDP 14540) and check that each step works.

Start Unity (play mode) and PX4 SITL first, see the Setup page in the wiki. Then:

    pip install pymavlink
    python3 tools/px4_flight_test.py hover            # any vehicle: take off, hover, land
    python3 tools/px4_flight_test.py vtol             # quadplane: adds a transition and a loiter on the wing

Prints PX4's own estimate once a second. Exit status: 0 pass, 1 PX4 never set home,
2 PX4 refused to arm, 3 a flight step didn't complete (climb, transition or landing).
"""
import argparse
import math
import os
import sys
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

mav = mavutil.mavlink
VTOL_STATE = {0: "-", 1: "to-FW", 2: "to-MC", 3: "MC", 4: "FW"}
NAN = float("nan")


class Vehicle:
    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=245)
        self.state = {}

    def get(self, msg_type):
        return self.state.get(msg_type)

    @property
    def armed(self):
        hb = self.get("HEARTBEAT")
        return bool(hb and hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)

    @property
    def altitude(self):
        lp = self.get("LOCAL_POSITION_NED")
        return -lp.z if lp else 0.0

    @property
    def vtol_state(self):
        ext = self.get("EXTENDED_SYS_STATE")
        return ext.vtol_state if ext else 0

    def pump(self, seconds, label, until=None, every=1.0):
        """Read messages for up to `seconds`, reporting as we go. Returns True if `until` came true."""
        end = time.time() + seconds
        last = 0.0
        while time.time() < end:
            msg = self.m.recv_match(blocking=True, timeout=0.2)
            if msg is not None:
                self.state[msg.get_type()] = msg
                if msg.get_type() == "STATUSTEXT":
                    print(f"  PX4: {msg.text}", flush=True)
            if time.time() - last >= every:
                last = time.time()
                self.report(label)
            if until is not None and until():
                self.report(label)
                return True
        return False

    def report(self, label):
        lp, att, hud = self.get("LOCAL_POSITION_NED"), self.get("ATTITUDE"), self.get("VFR_HUD")
        s = f"[{label:7}] {VTOL_STATE.get(self.vtol_state, '?'):5} armed={self.armed!s:5}"
        if lp:
            s += f" alt={-lp.z:6.1f} N={lp.x:7.1f} E={lp.y:7.1f}"
        if hud:
            s += f" aspd={hud.airspeed:5.1f} thr={hud.throttle:3d}%"
        if att:
            s += f" roll={math.degrees(att.roll):6.1f} pitch={math.degrees(att.pitch):6.1f}"
        print(s, flush=True)

    def command(self, command, *params):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, command, 0, *p)
        deadline = time.time() + 5
        while time.time() < deadline:
            ack = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if ack and ack.command == command:
                return ack.result
        return None

    def connect(self):
        print("waiting for PX4 heartbeat...", flush=True)
        self.m.wait_heartbeat(timeout=60)
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            mav.MAV_DATA_STREAM_ALL, 10, 1)
        print("waiting for home position (EKF and GPS ready)...", flush=True)
        deadline = time.time() + 120
        while time.time() < deadline:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                         mav.MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0, 0, 0)
            home = self.m.recv_match(type="HOME_POSITION", blocking=True, timeout=2)
            if home:
                print(f"home: lat={home.latitude / 1e7:.6f} lon={home.longitude / 1e7:.6f} "
                      f"alt={home.altitude / 1e3:.1f}", flush=True)
                return home
        return None


def fail(code, why):
    print(f"FAIL: {why}", flush=True)
    sys.exit(code)


def take_off(v, home, alt):
    if v.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1) != mav.MAV_RESULT_ACCEPTED:
        v.pump(3, "armfail")
        fail(2, "PX4 refused to arm")
    v.command(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, NAN, NAN, NAN, home.altitude / 1e3 + alt)
    if not v.pump(60, "climb", until=lambda: v.altitude > alt - 1.0):
        fail(3, f"didn't reach {alt} m")
    v.pump(5, "hover")


def transition(v, target, label):
    v.command(mav.MAV_CMD_DO_VTOL_TRANSITION, target)
    if not v.pump(40, label, until=lambda: v.vtol_state == target):
        fail(3, f"transition {label} didn't complete")


def land(v):
    v.command(mav.MAV_CMD_NAV_LAND, 0, 0, 0, NAN, NAN, NAN, NAN)
    if not v.pump(120, "land", every=2.0, until=lambda: not v.armed):
        fail(3, "didn't land and disarm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("flight", choices=["hover", "vtol"])
    ap.add_argument("--alt", type=float, help="takeoff altitude above home, m (hover 5, vtol 40)")
    ap.add_argument("--hover-seconds", type=float, default=10)
    ap.add_argument("--fw-seconds", type=float, default=60, help="time on the wing for vtol")
    ap.add_argument("--url", default="udpin:0.0.0.0:14540", help="pymavlink connection string")
    args = ap.parse_args()
    alt = args.alt if args.alt is not None else (5.0 if args.flight == "hover" else 40.0)

    v = Vehicle(args.url)
    home = v.connect()
    if home is None:
        fail(1, "no home position after 120 s")

    take_off(v, home, alt)
    if args.flight == "hover":
        v.pump(args.hover_seconds, "hover")
    else:
        transition(v, mav.MAV_VTOL_STATE_FW, "to-FW")
        v.pump(args.fw_seconds, "FW", every=2.0)
        transition(v, mav.MAV_VTOL_STATE_MC, "to-MC")
        v.pump(8, "hover")
    land(v)
    print("PASS", flush=True)


if __name__ == "__main__":
    main()
