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
Assets/_Project/Scripts/        DroneSim assembly
  Bridge/      MAVLink v2 codec, HIL server on TCP 4560, lockstep
  Vehicle/     Rotor, Wing, ControlSurface, VehicleActuators, ManualControl. Forces only
  Sensors/     ImuModel, delay buffer, and ROS publishers (camera, IMU, ground truth)
  World/       GeoOrigin (lat/lon to local), Wind
  Core/        Settings, ROS clock and publisher base, reset manager
  UserCamera/  Orbit and free cameras
  Utils/       Gaussian noise, Gauss-Markov bias
Assets/Editor/DroneSim/         SceneBuilder generates Flight.unity (quadplane) and FlightX500.unity
Assets/Tests/EditMode/          Codec, frame, geo and actuator tests
```

Not written yet: SynthData (randomizers, batch runner, YOLO converter). See
[roadmap.md](roadmap.md). The only HUD is the bridge status line drawn with OnGUI.

## Coordinate frames

- Unity: left-handed, Y up.
- PX4 HIL: NED, lat/lon/alt for GPS.
- ROS publishers: FLU body, ENU world, as the AUV sim does.

One GeoOrigin script owns the lat/lon/alt of the Unity origin and converts both ways. Nothing else does its own geo math.

## Vehicle model

- Rigidbody mass and inertia from the CAD.
- Rotor: thrust = max thrust * throttle^2 along the rotor axis at the mount point, with reaction torque and a first-order spin-up. Four lift rotors, one pusher.
- Control surfaces: a moment about one body axis, scaled by dynamic pressure and deflection. Two ailerons (half the roll authority each) and an elevator.
- Wing: lift = 0.5 * rho * V^2 * S * CL(alpha), linear up to stall then dropped. Parasitic plus induced drag. Small pitch and roll damping.
- Wind: constant vector plus gusts, subtracted from the airspeed the wing sees.

Actuator channel order follows PX4's standard_vtol (see the table in [running.md](running.md)).
Mixing stays on the PX4 side. Rotor spin sign matches `CA_ROTORn_KM`: +1 is CCW from above.

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
