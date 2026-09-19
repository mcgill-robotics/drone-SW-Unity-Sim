# drone-unity-sim

Unity simulator for the McGill Robotics drone team. A physics and camera plant for PX4 SITL, so our real autonomy stack can fly in a fake world.

## Why

Field time is scarce and the airframe is expensive to crash. Most of what we write only gets tested by flying. A sim lets us fly it hundreds of times a day, on a laptop, and break nothing.

The AUV team built the same thing for RoboSub in Unity. We borrow their reusable parts and build the aerial parts fresh.

## The one big idea: PX4 stays in the loop

The AUV sim skips the autopilot and applies forces straight from ROS. That works for them because their whole control stack is custom.

Ours flies PX4 on a Pixhawk, with ROS 2 on the Jetson talking to it over uXRCE-DDS. If the sim bypasses PX4, it tests nothing we wrote. So Unity plays the role Gazebo plays in the stock PX4 setup: it is the physics plant for PX4 SITL, and everything above the autopilot runs unchanged.

```
QGroundControl ──MAVLink UDP 14550──┐
                                    ▼
Unity ──HIL MAVLink TCP 4560──► PX4 SITL ──uXRCE-DDS UDP 8888──► ROS 2 (Jetson stack)
  │                                                                     ▲
  └────────── cameras, ground truth over ROS TCP connector ─────────────┘
```

- Unity sends HIL_SENSOR, HIL_GPS, HIL_STATE_QUATERNION each physics tick, receives HIL_ACTUATOR_CONTROLS, runs in lockstep.
- PX4 SITL with the standard VTOL airframe. Its EKF, mission logic, transition, and failsafes are real.
- The ROS 2 stack sees the same /fmu topics it sees on the real drone.
- Unity streams camera images and ground truth to ROS 2 for the vision team.

## What it is

- A VTOL fixed-wing rigidbody: lift rotors, a pusher, a wing. Simple dynamics, PX4 does the controlling.
- Sensors PX4 needs, with noise: IMU, mag, baro, GPS.
- Cameras published to ROS 2 with correct intrinsics.
- A flat world with a geo origin so lat/lon means something.
- Wind.
- A reset topic so runs can be scripted.
- Synthetic training images later. The AUV team's Perception package does not compile on Unity 6.6, so that pipeline needs a port or a replacement before it comes over.

Competition props, scoring, and the real flight area come later, as scenes and data on top of this. See [docs/suas-2026-rules.md](docs/suas-2026-rules.md) for what those will eventually need.

## Borrowed from auv-sim-unity

ROS publisher base and clock, IMU with bias random walk, sensor delay buffer, camera publisher with async GPU readback and CameraInfo, ground truth publisher, free and orbit cameras, synthetic data batch runner and YOLO converter.

Left behind: everything underwater.

## Stack

- Unity 6.6, HDRP.
- PX4 SITL, standard_vtol airframe.
- ROS 2 Humble, uXRCE-DDS agent, Unity ROS TCP connector for images.

## Docs

- [docs/running.md](docs/running.md): how to start Unity, PX4 SITL, and ROS 2 together.
- [docs/architecture.md](docs/architecture.md): how the pieces connect.
- [docs/suas-2026-rules.md](docs/suas-2026-rules.md): competition reference, not a design input yet.
