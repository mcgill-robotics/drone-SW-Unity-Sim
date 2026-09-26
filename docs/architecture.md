# How it works

## Processes

| Process | Role | Connections |
|---|---|---|
| Unity | Physics, sensors, rendering | Serves PX4 on TCP 4560; client of the ROS TCP endpoint on TCP 10000 |
| PX4 SITL | The real autopilot code, built for the host | Unity; MAVLink on UDP 14550 (ground station) and 14540 (API); uXRCE-DDS on UDP 8888 |
| QGroundControl | Parameters, monitoring, logs, manual missions | PX4 over MAVLink |
| Micro XRCE-DDS agent | Bridges PX4's internal topics to ROS 2 as `/fmu/...` | PX4, ROS 2 |
| ROS TCP endpoint | Bridges Unity's publishers to ROS 2 | Unity, ROS 2 |

PX4 can't tell Unity from Gazebo or from real hardware: it gets sensor readings and sends
actuator outputs. Everything above PX4 is unaware of the simulator.

## The PX4 bridge

`Bridge/Px4HilBridge.cs` implements PX4's simulator MAVLink interface, the same one Gazebo
Classic uses. Unity listens on TCP 4560 and PX4's `simulator_mavlink` module connects to it.

Every physics step (250 Hz):

1. Unity sends `HIL_SENSOR` (accelerometer, gyro, magnetometer, barometer, airspeed), plus
   `HIL_GPS` at 10 Hz and `HIL_STATE_QUATERNION` (ground truth, for PX4's logs) at 50 Hz.
2. Unity waits for `HIL_ACTUATOR_CONTROLS`: 16 channels, motors 0..1 and servos -1..1.
3. The controls go to the vehicle's actuators and physics steps.

This is lockstep: simulated time only advances when PX4 has answered, so a slow machine runs
slower than real time instead of feeding PX4 stale data. The step timestamp is simulated time,
which PX4 uses as its clock. If PX4 goes quiet for three steps, Unity stops waiting until
controls arrive again.

`Bridge/MavlinkCodec.cs` is a minimal MAVLink v2 encoder and parser for just these messages.
Tests check its bytes against frames produced by pymavlink.

## Coordinate frames

| Where | Convention |
|---|---|
| Unity | Left-handed. x right (east), y up, z forward (north) |
| PX4 | World NED, body FRD (forward, right, down) |
| ROS topics from Unity | World ENU, body FLU, as REP 103 |

Conversions live in one place each: `Px4HilBridge.BodyToFrd`, `RateToFrd` and `ToNedFrd` for
PX4, and the ROS TCP connector's `To<FLU>()` for ROS. Angular rates are pseudovectors, so going
between left- and right-handed frames flips their sign after the axis swap. Tests pin each
conversion.

`World/GeoOrigin.cs` holds the latitude, longitude and altitude of the Unity origin and converts
positions to and from geodetic coordinates (flat earth, fine within a few km). The magnetic
field comes from `World/WorldMagneticModel.cs`, a copy of the lookup table PX4 uses to check its
compass, so the simulated field agrees with what PX4 expects at any home location.

## Sensors

- **IMU** (`Sensors/ImuModel.cs`): acceleration and angular rate at the sensor's mount point,
  with white noise and slowly wandering bias. PX4 and the ROS IMU topic read the same instance.
- **Magnetometer, barometer, airspeed, GPS**: computed in the bridge from ground truth, with
  noise. Every channel needs some noise, because PX4 treats a reading that never changes as a
  failed sensor.
- **Cameras** (`Sensors/CameraPublisher.cs`): rendered to a texture, read back without
  stalling the GPU, published as JPEG with a matching CameraInfo.

## Vehicle model

All forces are applied to one rigidbody. Mixing and control stay in PX4.

- **Rotor**: thrust = max thrust × speed², along the rotor axis at its mount point, plus a
  reaction torque. Speed follows the command through a first-order lag. Spin direction follows
  the sign of PX4's `CA_ROTORn_KM`: +1 is counter-clockwise seen from above.
- **Wing**: lift from a linear lift curve up to stall, parasitic and induced drag, sideforce,
  static stability and rate damping, at the aerodynamic centre. Airspeed subtracts the wind.
- **Control surface**: a moment about one body axis, proportional to dynamic pressure and
  deflection, behind a servo lag.
- **Wind** (`World/Wind.cs`): a mean vector plus slow gusts.

The numbers are in `SceneBuilder.cs`. The X500's come from PX4's Gazebo model. The quadplane's
mass (6.5 kg) and prop size (16 inch) are ours; its thrust, inertia and aerodynamics are
placeholders sized to fly in PX4, to be replaced by measured values.

### Quadplane channels

From PX4's `1040_gazebo-classic_standard_vtol` airframe. The same table is in code as
`StandardVtolChannels`.

| Channel | Actuator | Range | Notes |
|---|---|---|---|
| 0 | Lift rotor front-right | 0..1 | CCW |
| 1 | Lift rotor rear-left | 0..1 | CCW |
| 2 | Lift rotor front-left | 0..1 | CW |
| 3 | Lift rotor rear-right | 0..1 | CW |
| 4 | Pusher | 0..1 | |
| 5 | Left aileron | -1..1 | positive rolls left |
| 6 | Right aileron | -1..1 | positive rolls right |
| 7 | Elevator | -1..1 | positive pitches nose up |

No rudder. The X500 uses channels 0 to 3 in the same order. When the real aircraft gets its own
PX4 airframe, update `StandardVtolChannels`, the builder and this table together.

## Code layout

```
Assets/_Project/Scripts/     DroneSim assembly, namespace DroneSim
  Bridge/      PX4 bridge and MAVLink codec
  Vehicle/     Rotor, Wing, ControlSurface, VehicleActuators, ManualControl (keyboard flying)
  Sensors/     IMU model, state history for sensor latency, ROS publishers
  World/       GeoOrigin, WorldMagneticModel, Wind
  Core/        SimSettings, ROSSettings (topic names), ROS clock and publisher base, reset
  UserCamera/  Chase and free cameras
  Utils/       Gaussian noise, bias random walk
Assets/Editor/DroneSim/      SceneBuilder
Assets/Tests/EditMode/       Unit tests
```

Scene-wide objects (`SimSettings`, `ROSSettings`, `GeoOrigin`, `Wind`) are single components on
the `Managers` object, found through a static `Instance`.
