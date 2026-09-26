"""Smoke test for the sim: arm, take off, hover 10 s, land, through PX4 SITL's MAVLink API port.

Start Unity (play mode) and PX4 SITL first, see docs/running.md. Then:

    pip install pymavlink
    python3 tools/px4_smoke_test.py [altitude_m]

Prints PX4's own estimate of position and attitude once a second. A healthy run arms, climbs
to the altitude, holds it, lands and disarms by itself. Exits non-zero if PX4 never sets home
or refuses to arm.
"""
import math
import os
import sys
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

ALT = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=245)
state = {}


def pump(duration, every=1.0, label=""):
    end = time.time() + duration
    last = 0
    while time.time() < end:
        msg = m.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "STATUSTEXT":
            print(f"  PX4: {msg.text}")
        state[t] = msg
        if time.time() - last >= every:
            last = time.time()
            report(label)


def report(label):
    lp = state.get("LOCAL_POSITION_NED")
    att = state.get("ATTITUDE")
    hb = state.get("HEARTBEAT")
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None
    s = f"[{label:8}] armed={armed}"
    if lp:
        s += f" pos N={lp.x:6.2f} E={lp.y:6.2f} alt={-lp.z:6.2f}  v_down={lp.vz:5.2f}"
    if att:
        s += f"  roll={math.degrees(att.roll):6.1f} pitch={math.degrees(att.pitch):6.1f} yaw={math.degrees(att.yaw):6.1f}"
    print(s, flush=True)


def cmd(command, *params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, command, 0, *p)
    deadline = time.time() + 5
    while time.time() < deadline:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == command:
            print(f"  ack {command}: result {ack.result}")
            return ack.result
    print(f"  no ack for {command}")
    return None


print("waiting for PX4 heartbeat on 14540...")
m.wait_heartbeat(timeout=60)
print(f"heartbeat from system {m.target_system}")
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

print("waiting for home position (EKF + GPS ready)...")
deadline = time.time() + 120
while time.time() < deadline:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue
    state[msg.get_type()] = msg
    if msg.get_type() == "STATUSTEXT":
        print(f"  PX4: {msg.text}")
    if msg.get_type() == "HOME_POSITION":
        print(f"home: lat={msg.latitude / 1e7:.6f} lon={msg.longitude / 1e7:.6f} alt={msg.altitude / 1e3:.1f}")
        break
    if int(time.time()) % 5 == 0:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 242, 0, 0, 0, 0, 0, 0)
else:
    print("no home position after 120 s")
    sys.exit(1)

pump(5, label="ground")
home = state.get("HOME_POSITION")
takeoff_amsl = home.altitude / 1e3 + ALT

print("arming...")
if cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) != 0:
    pump(3, label="armfail")
    sys.exit(2)
pump(3, label="armed")

print(f"takeoff to {ALT} m...")
cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, float("nan"),
    float("nan"), float("nan"), takeoff_amsl)
pump(20, label="takeoff")
pump(10, label="hover")

print("landing...")
cmd(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, float("nan"), float("nan"), float("nan"), float("nan"))
pump(25, label="land")
hb = state.get("HEARTBEAT")
print("final armed:", bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None)
