# Architecture

## Processes

```
QGroundControl ── MAVLink UDP 14550 ──┐
MAVLink scripts ─ MAVLink UDP 14540 ──┤
                                      ▼
Unity ──── HIL MAVLink, TCP 4560 ──► PX4 SITL ── uXRCE-DDS, UDP 8888 ──► ROS 2
  │                                                                       ▲
  └────────── cameras, ground truth, /clock (ROS TCP endpoint) ───────────┘
```

<!-- todo: cleaner diagram -->

| Process | Role |
|---|---|
| Unity | Physics, sensors, rendering |
| PX4 SITL | The real autopilot code, built for the host |
| QGroundControl | Parameters, monitoring, missions, logs |
| Micro XRCE-DDS agent | Exposes PX4's internal topics to ROS 2 as `/fmu/...` |
| ROS TCP endpoint | Exposes Unity's publishers to ROS 2 |

PX4 receives sensor readings and sends actuator outputs; it cannot tell Unity from Gazebo or from
real hardware. Everything above PX4 is unaware of the simulator.

## Interfaces

Every port and topic, and what takes its place on the real drone.

| Port or topic | Type | Between | Rate | Real drone |
|---|---|---|---|---|
| TCP 4560 | MAVLink HIL | PX4 SITL and Unity (Unity listens) | 250 Hz, lockstep | Sensors and actuators wired to the Pixhawk |
| UDP 14550 | MAVLink | PX4 and QGroundControl | | Telemetry radio |
| UDP 14540 | MAVLink | PX4 and scripts (`tools/px4_flight_test.py`) | | None |
| UDP 8888 | uXRCE-DDS | PX4 and the Micro XRCE-DDS agent | | Serial or Ethernet, Pixhawk to Jetson |
| TCP 10000 | ROS TCP | Unity and the ROS TCP endpoint (Unity connects) | | None, camera drivers publish on the Jetson |
| `/fmu/in/...`, `/fmu/out/...` | px4_msgs | ROS 2 and PX4, through the agent | Set by PX4 | Same |
| `/sim/forward_cam/image_raw/compressed` | sensor_msgs/CompressedImage, 1280x720 JPEG | Unity to ROS 2 | 10 Hz | Forward camera driver |
| `/sim/forward_cam/camera_info` | sensor_msgs/CameraInfo | Unity to ROS 2 | 10 Hz | Forward camera driver |
| `/sim/down_cam/image_raw/compressed` | sensor_msgs/CompressedImage, 1280x720 JPEG | Unity to ROS 2 | 10 Hz | Down camera driver |
| `/sim/down_cam/camera_info` | sensor_msgs/CameraInfo | Unity to ROS 2 | 10 Hz | Down camera driver |
| `/clock` | rosgraph_msgs/Clock | Unity to ROS 2 | 10 Hz | None, sim only |
| `/sim/ground_truth/pose` | geometry_msgs/PoseStamped, ENU relative to the Unity origin | Unity to ROS 2 | 50 Hz | None, sim only |
| `/sim/ground_truth/twist` | geometry_msgs/TwistStamped, body FLU | Unity to ROS 2 | 50 Hz | None, sim only |
| `/sim/ground_truth/gps` | sensor_msgs/NavSatFix | Unity to ROS 2 | 50 Hz | None, sim only |
| `/simulation/reset` | std_msgs/Bool, `true` resets the vehicle | ROS 2 to Unity | On demand | None, sim only |

- Camera rows are the defaults in `sim_config.json` ([Camera config](Development#camera-config)).
- Ports are the defaults. `--px4-port`, `--ros-ip` and `--ros-port` change them in a build
  ([Standalone build](Setup#standalone-build)).
- Flight code should only use topics the real drone has. Sim-only topics are for tests and
  debugging.

## PX4 bridge and lockstep

`Bridge/Px4HilBridge.cs` implements PX4's simulator MAVLink interface, the one Gazebo Classic
uses. Unity listens on TCP 4560 and PX4's `simulator_mavlink` module connects to it.
`Bridge/MavlinkCodec.cs` is a minimal MAVLink v2 encoder and parser for the messages involved.

Every physics step (250 Hz, Unity's fixed timestep):

1. Unity sends `HIL_SENSOR` (accelerometer, gyro, magnetometer, barometer, airspeed), plus
   `HIL_GPS` at 10 Hz and `HIL_STATE_QUATERNION` (ground truth, for PX4's logs) at 50 Hz.
2. Unity waits for `HIL_ACTUATOR_CONTROLS`: 16 channels, motors 0..1 and servos -1..1.
3. The controls go to the vehicle's actuators and the physics steps.

Simulated time advances only after PX4 answers, so a slow machine runs slower than real time
rather than feeding PX4 stale data. The step timestamp is simulated time, which PX4 uses as its
clock. If PX4 fails to answer for three consecutive 1 s timeouts, Unity stops waiting and steps
on its own until controls arrive again.

## Coordinate frames

| Where | World | Body |
|---|---|---|
| Unity | x east, y up, z north (left-handed) | x right, y up, z forward |
| PX4 | NED | FRD |
| ROS topics from Unity | ENU | FLU (REP 103) |

<!-- todo: image of the three axis sets on the drone -->

Each conversion exists in one place: `Px4HilBridge.BodyToFrd`, `RateToFrd` and `ToNedFrd` for
PX4; `GeoOrigin.ToEnu` and `ToEnuFlu` for ROS world data; the ROS TCP connector's `To<FLU>()` for
ROS body vectors. Angular rates are pseudovectors, so a change of handedness flips their sign
after the axis swap. Unit tests cover the PX4 and ENU conversions.

`World/GeoOrigin.cs` holds the latitude, longitude and altitude of the Unity origin and converts
positions to and from geodetic coordinates with a flat-earth approximation, which is adequate
within a few kilometres. PX4 takes its home from the GPS Unity sends, so the home location is
configured on the Unity side ([Setup](Setup#home-location)).

`World/WorldMagneticModel.cs` is a copy of the lookup table PX4 uses to check its compass, so
the simulated field agrees with PX4's expectation at any home location.

## Sensors

| Sensor | Source |
|---|---|
| IMU (`Sensors/ImuModel.cs`) | Acceleration and angular rate at the mount point given in the vehicle spec, with white noise and slowly wandering bias. Sent to PX4 only, like the Pixhawk's IMU on the real drone |
| Magnetometer, barometer, airspeed, GPS | Computed in the bridge from ground truth, with noise |
| Cameras (`Sensors/CameraPublisher.cs`) | One per camera mount in the vehicle spec, rendered only at publish time and only while the ROS endpoint is connected, read back asynchronously and published as JPEG or raw with a matching `CameraInfo`. Topics, resolution, rate and format are in `sim_config.json` ([Camera config](Development#camera-config)) |

Every channel sent to PX4 carries some noise. PX4 treats a reading that repeats the exact same
value as a failed sensor.

## Vehicle model

All forces act on one rigidbody. Mixing and control stay in PX4; Unity only turns channel
values into forces.

| Component | Model |
|---|---|
| `Rotor` | Thrust along the rotor axis at its mount point, plus a reaction torque about it. Thrust = max thrust x speed^2, speed following the command through a first-order lag with an optional idle speed when armed; or a thrust table looked up on PX4's throttle, with the lag on thrust and torque. Spin direction follows the sign of PX4's `CA_ROTORn_KM` (+1 is counter-clockwise from above) |
| `Wing` | Linear lift curve up to stall, parasitic and induced drag, sideforce, static stability and rate damping, applied at the aerodynamic centre. Airspeed subtracts the wind |
| `ControlSurface` | A moment with the roll, pitch and yaw proportions of PX4's `CA_SV_CSn_TRQ_R/P/Y`, proportional to dynamic pressure and deflection, behind a servo lag |
| `Wind` (`World/Wind.cs`) | Mean vector plus slow gusts |

The vehicle is built at startup by `VehicleLoader` from a JSON spec in
`Assets/StreamingAssets/vehicles/`: mass, CG and inertia on the rigidbody, a box collider, the
components above, the IMU and camera mounts, and placeholder shapes. The spec format, the checks
and the summary it logs are in [Vehicles](Vehicles). The X500's numbers come from PX4's Gazebo
model. The quadplane's mass (6.5 kg) and prop size (16 inch) match the team's aircraft; its
thrust, inertia and aerodynamics are placeholders sized to fly under PX4, marked `estimate` in
the spec.

## Scenes

`Assets/_Project/Scenes/Flight.unity` is the one scene, generated by
`Assets/Editor/DroneSim/SceneBuilder.cs` (Drone Sim > Build Flight Scene): a flat ground plane
with a runway, a `Managers` object carrying the scene-wide components (`SimSettings`,
`ROSSettings`, `GeoOrigin`, `Wind`, `VehicleLoader`, `Px4HilBridge`, `ROSConnection` and others),
and a chase camera. The vehicle is not in the scene; `VehicleLoader` builds it when the scene
starts. Edit the builder rather than the scene, rebuild it from the menu, and commit both. Field
values changed in the Inspector on `Managers` are lost at the next rebuild.
