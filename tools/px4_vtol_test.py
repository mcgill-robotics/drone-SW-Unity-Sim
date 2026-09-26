"""VTOL transition test: take off as a quad, transition to fixed-wing, loiter, transition back, land.

Start Unity (Flight.unity, play mode) and PX4 SITL with PX4_SIM_MODEL=gazebo-classic_standard_vtol
first, see docs/running.md. Then:

    pip install pymavlink
    python3 tools/px4_vtol_test.py [altitude_m] [seconds_in_fixed_wing]

Prints PX4's view once a second: VTOL state, altitude, airspeed, attitude, throttle. After the
forward transition PX4 holds position the fixed-wing way, circling at NAV_LOITER_RAD.
"""
import math
import os
import sys
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

ALT = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
FW_SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
VTOL_STATE = {0: "undef", 1: "to-FW", 2: "to-MC", 3: "MC", 4: "FW"}

m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=245)
state = {}


def pump(duration, label, until=None, every=1.0):
    end = time.time() + duration
    last = 0
    while time.time() < end:
        msg = m.recv_match(blocking=True, timeout=0.2)
        if msg is not None:
            state[msg.get_type()] = msg
            if msg.get_type() == "STATUSTEXT":
                print(f"  PX4: {msg.text}", flush=True)
        if time.time() - last >= every:
            last = time.time()
            report(label)
        if until is not None and until():
            report(label)
            return True
    return False


def vtol_state():
    ext = state.get("EXTENDED_SYS_STATE")
    return ext.vtol_state if ext else 0


def report(label):
    lp, att, hud, hb = (state.get(k) for k in ("LOCAL_POSITION_NED", "ATTITUDE", "VFR_HUD", "HEARTBEAT"))
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None
    s = f"[{label:7}] {VTOL_STATE.get(vtol_state(), '?'):5} armed={armed!s:5}"
    if lp:
        s += f" alt={-lp.z:6.1f} N={lp.x:7.1f} E={lp.y:7.1f}"
    if hud:
        s += f" aspd={hud.airspeed:5.1f} gspd={hud.groundspeed:5.1f} thr={hud.throttle:3d}%"
    if att:
        s += f" roll={math.degrees(att.roll):6.1f} pitch={math.degrees(att.pitch):6.1f}"
    print(s, flush=True)


def cmd(command, *params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, command, 0, *p)
    deadline = time.time() + 5
    while time.time() < deadline:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == command:
            print(f"  ack {command}: result {ack.result}", flush=True)
            return ack.result
    print(f"  no ack for {command}", flush=True)
    return None


print("waiting for PX4 heartbeat on 14540...")
m.wait_heartbeat(timeout=60)
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
home = None
deadline = time.time() + 120
while home is None and time.time() < deadline:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 242, 0, 0, 0, 0, 0, 0)
    home = m.recv_match(type="HOME_POSITION", blocking=True, timeout=2)
if home is None:
    sys.exit("no home position")
print(f"home: lat={home.latitude / 1e7:.6f} lon={home.longitude / 1e7:.6f} alt={home.altitude / 1e3:.1f}")

if cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) != 0:
    pump(3, "armfail")
    sys.exit(2)

nan = float("nan")
cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, nan, nan, nan, home.altitude / 1e3 + ALT)
reached = pump(60, "climb", until=lambda: "LOCAL_POSITION_NED" in state and -state["LOCAL_POSITION_NED"].z > ALT - 1.5)
pump(5, "hover")
if not reached:
    print("did not reach takeoff altitude")

print("transition to fixed-wing...")
cmd(mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, mavutil.mavlink.MAV_VTOL_STATE_FW)
ok = pump(40, "to-FW", until=lambda: vtol_state() == mavutil.mavlink.MAV_VTOL_STATE_FW)
print("forward transition", "complete" if ok else "DID NOT COMPLETE")
pump(FW_SECONDS, "FW", every=2.0)

print("transition back to multicopter...")
cmd(mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, mavutil.mavlink.MAV_VTOL_STATE_MC)
ok = pump(40, "to-MC", until=lambda: vtol_state() == mavutil.mavlink.MAV_VTOL_STATE_MC)
print("back transition", "complete" if ok else "DID NOT COMPLETE")
pump(8, "hover")

print("landing...")
cmd(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, nan, nan, nan, nan)
pump(90, "land", every=2.0, until=lambda: "HEARTBEAT" in state and not (
    state["HEARTBEAT"].base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
pump(3, "landed")
