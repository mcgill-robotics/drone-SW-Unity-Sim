# Roadmap

The goal: retire Gazebo. Unity is the physics plant for PX4 SITL, and the Jetson stack runs
against it unchanged.

## Done

- MAVLink HIL bridge on TCP 4560 with lockstep. HIL_SENSOR at 250 Hz, HIL_GPS at 10 Hz,
  HIL_STATE_QUATERNION at 50 Hz. Encoding is checked against pymavlink golden frames.
- Placeholder quadplane at 6.5 kg with 16 inch props: four lift rotors, pusher, wing, two
  ailerons, elevator, wired to PX4's standard_vtol channels with PX4's output scaling.
- PX4's X500 quad from its Gazebo model, as a like-for-like Gazebo replacement.
- IMU, mag, baro, GPS with noise. Geo origin on the SUAS 2026 field.
- Wind with gusts. Reset from `R` or `/simulation/reset`, with optional randomization.
- Cameras, IMU, ground truth and `/clock` to ROS 2.
- Keyboard flying without PX4.
- Edit-mode tests for the codec, frame conversions, geo math and actuator mapping.

## Next: first flight

Steps 1 to 4 work (2026-09-26, PX4 v1.17.0 on macOS): the X500 and the quadplane hover
(`tools/px4_smoke_test.py`), and the quadplane transitions, loiters on the wing and comes back
(`tools/px4_vtol_test.py`). Fixes that came out of it: the magnetometer needed noise, because PX4
drops a sensor whose reading never changes; the project needed Run In Background; and the pusher
needed 30 N to reach blend airspeed inside PX4's 6 s window. The wing flies nose-high and sags
in turns, which points at the placeholder wing. Next: step 5. In order:

1. Connect PX4 SITL (see [running.md](running.md)). Check the status line reaches
   `lockstep=on`, and that QGC shows a sane attitude, heading and GPS fix with the vehicle
   sitting on the ground.
2. Arm in QGC. Rotors should sit at idle, not lift off.
3. Takeoff and hover in position mode. If it flips immediately, suspect a sign: rotor spin, a
   frame conversion, or rotor order. If it hovers but drifts or oscillates, tune the vehicle
   numbers.
4. Transition to fixed-wing, fly a short mission, back-transition, land.
5. Run drone-2027's flight-check ladder against the X500, then the quadplane.

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
