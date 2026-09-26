# Roadmap

The goal: retire Gazebo. Unity is the physics plant for PX4 SITL, and the Jetson stack runs
against it unchanged.

## Done

- MAVLink HIL bridge on TCP 4560 with lockstep. HIL_SENSOR at 250 Hz, HIL_GPS at 10 Hz,
  HIL_STATE_QUATERNION at 50 Hz. Encoding is checked against pymavlink golden frames.
- Placeholder standard VTOL: four lift rotors, pusher, wing, two ailerons, elevator, wired to
  PX4's standard_vtol channels with PX4's output scaling.
- IMU, mag, baro, GPS with noise. Geo origin on the SUAS 2026 field.
- Wind with gusts. Reset from `R` or `/simulation/reset`, with optional randomization.
- Cameras, IMU, ground truth and `/clock` to ROS 2.
- Keyboard flying without PX4.
- Edit-mode tests for the codec, frame conversions, geo math and actuator mapping.

## Next: first flight

The code has not flown with PX4 yet. In order:

1. Connect PX4 SITL (see [running.md](running.md)). Check the status line reaches
   `lockstep=on`, and that QGC shows a sane attitude, heading and GPS fix with the vehicle
   sitting on the ground.
2. Arm in QGC. Rotors should sit at idle, not lift off.
3. Takeoff and hover in position mode. If it flips immediately, suspect a sign: rotor spin, a
   frame conversion, or rotor order. If it hovers but drifts or oscillates, tune the vehicle
   numbers.
4. Transition to fixed-wing, fly a short mission, back-transition, land.
5. Replay the same mission through the ROS 2 stack on uXRCE-DDS.

Write down what broke in each step. The answers belong in the docs, not in someone's head.

## Then: make it our airframe

- Mass, inertia, rotor positions, thrust curves and wing numbers from the CAD and bench tests,
  replacing the placeholder values in `SceneBuilder`.
- A PX4 airframe file for our real aircraft, and `StandardVtolChannels` plus the channel
  table in running.md updated to match.
- Ground contact: landing gear colliders instead of one box.

## Then: make it the competition

- Terrain, trees and the runways from [suas-2026-rules.md](suas-2026-rules.md), with the flight
  and search boundaries drawn from their lat/lon.
- Mannequins and tents as targets. Payload drop with a parachute model.
- Synthetic training images. The AUV team's Perception package does not compile on Unity 6.6,
  so the randomizers, batch runner and YOLO converter need a port or a rewrite.

## Housekeeping

- `Assets/OutdoorsScene.unity` is the HDRP template scene. Nothing uses it.
- CI: run the edit-mode tests on every push once there is a Unity license for a runner.
