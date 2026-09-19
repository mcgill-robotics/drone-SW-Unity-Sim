# Architecture sketch

Working sketch. Expect it to change once the bridge is flying.

## Processes

| Process | Role | Talks to |
|---|---|---|
| Unity | Physics, rendering, sensors | PX4 SITL (MAVLink HIL, TCP 4560), ROS 2 (ROS TCP endpoint, TCP 10000) |
| PX4 SITL | Autopilot | Unity, QGC (UDP 14550), uXRCE agent (UDP 8888) |
| micro XRCE-DDS agent | Exposes PX4 uORB as ROS 2 topics under /fmu | PX4 SITL, ROS 2 graph |
| ROS 2 stack | Our code | /fmu topics, Unity camera topics |
| QGroundControl | Mission upload, monitoring | PX4 SITL |

Unity is the lockstep master. Each FixedUpdate it sends sensors, waits for actuator controls, then steps physics.

## Unity project layout

```
Assets/_Project/Scripts/
  Bridge/      MAVLink HIL client and lockstep
  Vehicle/     Rotor, Wing, AeroBody. Forces only, no control logic
  Sensors/     ROS publishers from the AUV sim: camera, IMU, ground truth, clock, delay buffer
  World/       GeoOrigin (lat/lon to local), Wind
  Core/        Settings, reset manager, HUD
  SynthData/   Perception randomizers, batch runner, YOLO converter
```

## Coordinate frames

- Unity: left-handed, Y up.
- PX4 HIL: NED, lat/lon/alt for GPS.
- ROS publishers: FLU body, ENU world, as the AUV sim does.

One GeoOrigin script owns the lat/lon/alt of the Unity origin and converts both ways. Nothing else does its own geo math.

## Vehicle model

- Rigidbody mass and inertia from the CAD.
- Rotor: thrust = k * omega^2 along the rotor axis at the mount point, with reaction torque and a first-order spin-up. Four lift rotors, one pusher.
- Wing: lift = 0.5 * rho * V^2 * S * CL(alpha), linear up to stall then dropped. Parasitic plus induced drag. Small pitch and roll damping.
- Wind: constant vector plus gusts, subtracted from the airspeed the wing sees.

Actuator channel order follows PX4's standard_vtol. Mixing stays on the PX4 side.

## Sensors to PX4

- HIL_SENSOR: accel, gyro, mag, baro. Noise added in Unity.
- HIL_GPS: lat, lon, alt, velocity, fix, sats. 10 Hz with delay.
- HIL_STATE_QUATERNION: ground truth, for PX4 logging.

## Sensors to ROS 2

- Cameras as compressed images with CameraInfo.
- Ground truth pose and twist.
- /clock.

## Reset

Keep the AUV sim's reset manager. A /simulation/reset topic snaps the vehicle back to the origin and can randomize mass, thrust, wind, and sensor latency.
