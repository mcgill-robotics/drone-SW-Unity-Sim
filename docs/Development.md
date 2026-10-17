# Development

## Code layout

```
Assets/_Project/Scripts/     DroneSim assembly, namespace DroneSim
  Bridge/      Px4HilBridge (PX4 connection, lockstep), MavlinkCodec
  Vehicle/     VehicleSpec (JSON format, checks, summary), VehicleBuilder, VehicleLoader,
               Rotor, Wing, ControlSurface, VehicleActuators, ManualControl (keyboard flying)
  Sensors/     ImuModel, SensorDelayBuffer, ROS publishers (cameras, ground truth)
  World/       GeoOrigin (home, GPS), WorldMagneticModel, Wind
  Core/        SimSettings, ROSSettings (sim-only topic names), SimConfig (sim_config.json), ROSClock,
               ROSPublisher base, SimulationResetManager, CommandLineOptions (player options)
  UserCamera/  Chase and free cameras
  Utils/       Noise generators
Assets/Editor/DroneSim/      SceneBuilder (Flight.unity)
Assets/StreamingAssets/      sim_config.json (cameras), vehicles/ (vehicle specs)
Assets/Tests/EditMode/       Unit tests
tools/                       px4_flight_test.py, px4_airframe.py
missions/                    QGroundControl mission for the quadplane
```

Scene-wide components (`SimSettings`, `ROSSettings`, `GeoOrigin`, `Wind`) live on the `Managers`
object and are reached through a static `Instance`. The vehicle is built at startup from its spec
and reached through `SimSettings.Vehicle`; `VehicleLoader.Loaded` holds the spec.

## Vehicle config

Everything about the aircraft (mass properties, rotors, wing, control surfaces, IMU and camera
mounts, PX4 airframe) is in its spec under `Assets/StreamingAssets/vehicles/`, documented in
[Vehicles](Vehicles). The scene is not involved; see [Scenes](Architecture#scenes) for what is.

## Camera config

`Assets/StreamingAssets/sim_config.json`, read at startup. In a build it is in
`drone-sim_Data/StreamingAssets/` (Windows, Linux) or
`drone-sim.app/Contents/Resources/Data/StreamingAssets/` (macOS), so it can be edited without
rebuilding. `--config PATH` uses another file ([Standalone build](Setup#standalone-build)).

One entry per camera, matched by `name` to a camera mount in the vehicle spec:

| Field | Meaning |
|---|---|
| `name` | Camera mount name in the spec: `ForwardCamera` or `DownCamera` in the shipped vehicles |
| `topic` | Image topic. `sensor_msgs/CameraInfo` goes on `camera_info` next to it |
| `frameId` | `frame_id` in the image and camera info headers |
| `width`, `height` | Resolution in pixels |
| `rate` | Publish rate in Hz |
| `verticalFov` | Vertical field of view in degrees. Intrinsics follow from it, no distortion |
| `format` | `jpeg`: `sensor_msgs/CompressedImage` on `<topic>/compressed`. `raw`: `sensor_msgs/Image` (rgba8) on `<topic>` |
| `jpegQuality` | 1 to 100, for `jpeg` |

A camera with no entry or a bad one isn't published, and the Unity console says why. Camera mounts
(position, tilt) are in the vehicle spec ([cameras](Vehicles#cameras)).

The topic names are placeholders under `/sim/`. Set them to what the real drone's camera drivers
publish once that's decided, so the same code runs on both.

## Actuator channels

PX4 sends 16 channels in `HIL_ACTUATOR_CONTROLS`, motors as 0..1 and servos as -1..1.
`VehicleActuators` hands each channel to the rotor or control surface whose `channel` in the spec
is that index (0-based; PX4 calls it `PWM_MAIN_FUNC<channel+1>`). PX4's side of the mapping comes
from the airframe: a generated airframe sets `PWM_MAIN_FUNC*` from the spec, so the two agree by
construction; with a stock airframe the spec's channels have to follow that airframe's output
layout ([PX4 airframe](Vehicles#px4-airframe)).

The shipped quadplane follows `1040_gazebo-classic_standard_vtol`:

| Channel | Actuator | Range | Notes |
|---|---|---|---|
| 0 | Lift rotor front-right | 0..1 | CCW |
| 1 | Lift rotor rear-left | 0..1 | CCW |
| 2 | Lift rotor front-left | 0..1 | CW |
| 3 | Lift rotor rear-right | 0..1 | CW |
| 4 | Pusher | 0..1 | |
| 5 | Left aileron | -1..1 | Positive rolls left |
| 6 | Right aileron | -1..1 | Positive rolls right |
| 7 | Elevator | -1..1 | Positive pitches nose up |

The airframe has no rudder. The X500 uses channels 0 to 3 in the same order (`none_iris`). The
same layout is in code as `StandardVtolChannels` in `Vehicle/VehicleActuators.cs`, which only the
keyboard controls (`ManualControl`, fields on `Managers`) and the tests read. A spec with a
different layout flies under PX4 without code changes; keyboard flying needs the lift rotor and
pusher channels on `ManualControl` updated to match.

## Tests

Edit-mode unit tests cover MAVLink encoding (checked against frames from pymavlink), frame
conversions, geodetic math, the magnetic field table, actuators, the camera config and the vehicle
spec (FRD to Unity, inertia, checks, defaults, thrust tables, hover throttle, and that the
shipped specs are valid and build).

- In Unity: Window > General > Test Runner > EditMode > Run All.
- From a terminal, with the editor closed, from the project folder:

  ```bash
  /Applications/Unity/Hub/Editor/6000.6.0f1/Unity.app/Contents/MacOS/Unity -batchmode -nographics \
    -projectPath . -runTests -testPlatform EditMode -testResults results.xml -logFile test.log
  ```

End-to-end: `tools/px4_flight_test.py` with PX4 connected ([Setup](Setup#flight-test-script)).

## CI

GitHub Actions, using game-ci's Unity images:

- `.github/workflows/tests.yml` runs the edit-mode tests on every push to `main` and every pull
  request.
- `.github/workflows/build.yml` builds Linux, Windows and macOS players. Run it from the Actions
  tab, or push a tag such as `v0.1.0`; a tag also creates a draft release with the three builds
  zipped.

Both need a Unity license in the repository secrets: `UNITY_EMAIL`, `UNITY_PASSWORD` and either
`UNITY_SERIAL` (Pro or Student license) or `UNITY_LICENSE` (contents of a Personal `.ulf` file).
Only one Unity job runs at a time so they don't compete for the license.

## ROS topics

Full list with types, rates and ports: [Interfaces](Architecture#interfaces).

- Camera topics, frames, resolution, rate and format are in `sim_config.json`
  ([Camera config](#camera-config)). The other topic and frame names are on `ROSSettings`.
- Clock and ground truth rates are on their publisher components. The ROS master switch is on
  `SimSettings`.
- `/fmu/...` topics come from PX4 through the XRCE-DDS agent, not from Unity.

## Settings

| Setting | Where |
|---|---|
| Cameras: topics, frames, resolution, rate, format | `Assets/StreamingAssets/sim_config.json` |
| Wind: mean vector, gust amplitude, gust period | `Wind` on `Managers` |
| Randomize mass, thrust and wind on reset (off by default) | `SimulationResetManager` on `Managers` |
| IMU noise and latency | `ImuModel` on the vehicle's `IMU` object. The vehicle is built at startup, so change the defaults in `Sensors/ImuModel.cs` or edit them in play mode |
| GPS, barometer, magnetometer and airspeed noise; lockstep timeout | `Px4HilBridge` on `Managers` |
| ROS endpoint address and port | `ROSConnection` on `Managers` |

## Keys

With the Game view focused:

| Key | Action |
|---|---|
| R | Reset the vehicle to its start pose |
| M | Toggle keyboard flying (PX4 controls are ignored while on) |
| Shift / Ctrl | Lift rotor collective up / down |
| W / S | Pitch nose down / nose up |
| A / D | Roll left / right |
| Q / E | Yaw left / right |
| Up / Down | Pusher throttle |
| Space | Cut all motors |
| Right drag | Orbit the chase camera |
| Scroll | Zoom |
