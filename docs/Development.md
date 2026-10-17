# Development

## Code layout

```
Assets/_Project/Scripts/     DroneSim assembly, namespace DroneSim
  Bridge/      Px4HilBridge (PX4 connection, lockstep), MavlinkCodec
  Vehicle/     Rotor, Wing, ControlSurface, VehicleActuators, ManualControl (keyboard flying)
  Sensors/     ImuModel, SensorDelayBuffer, ROS publishers (cameras, ground truth)
  World/       GeoOrigin (home, GPS), WorldMagneticModel, Wind
  Core/        SimSettings, ROSSettings (sim-only topic names), SimConfig (sim_config.json), ROSClock,
               ROSPublisher base, SimulationResetManager, CommandLineOptions (player options)
  UserCamera/  Chase and free cameras
  Utils/       Noise generators
Assets/Editor/DroneSim/      SceneBuilder
Assets/StreamingAssets/      sim_config.json (cameras)
Assets/Tests/EditMode/       Unit tests
tools/                       px4_flight_test.py
missions/                    QGroundControl mission for the quadplane
```

Scene-wide components (`SimSettings`, `ROSSettings`, `GeoOrigin`, `Wind`) live on the `Managers`
object and are reached through a static `Instance`.

## Vehicle config

Vehicle numbers (mass, inertia, rotor positions and thrust, wing coefficients, camera placement)
are in `Assets/Editor/DroneSim/SceneBuilder.cs`. To change a vehicle:

1. Edit the builder.
2. In Unity, run Drone Sim > Build Flight Scenes (or Build Quadplane Scene / Build X500 Scene).
3. Commit the builder and the regenerated scenes together.

<!-- todo: image of the Drone Sim menu -->

Changes made in the Inspector are overwritten by the next rebuild, so put them in the builder.

## Camera config

`Assets/StreamingAssets/sim_config.json`, read at startup. In a build it is in
`drone-sim_Data/StreamingAssets/` (Windows, Linux) or
`drone-sim.app/Contents/Resources/Data/StreamingAssets/` (macOS), so it can be edited without
rebuilding. `--config PATH` uses another file ([Standalone build](Setup#standalone-build)).

One entry per camera, matched to the camera object in the scene by `name`:

| Field | Meaning |
|---|---|
| `name` | Camera object in the scene: `ForwardCamera` or `DownCamera` |
| `topic` | Image topic. `sensor_msgs/CameraInfo` goes on `camera_info` next to it |
| `frameId` | `frame_id` in the image and camera info headers |
| `width`, `height` | Resolution in pixels |
| `rate` | Publish rate in Hz |
| `verticalFov` | Vertical field of view in degrees. Intrinsics follow from it, no distortion |
| `format` | `jpeg`: `sensor_msgs/CompressedImage` on `<topic>/compressed`. `raw`: `sensor_msgs/Image` (rgba8) on `<topic>` |
| `jpegQuality` | 1 to 100, for `jpeg` |

A camera with no entry or a bad one isn't published, and the Unity console says why. Camera mounts
(position, tilt) are in `SceneBuilder.cs` with the rest of the vehicle.

The topic names are placeholders under `/sim/`. Set them to what the real drone's camera drivers
publish once that's decided, so the same code runs on both.

## Actuator channels

The quadplane follows PX4's `1040_gazebo-classic_standard_vtol` airframe. The same mapping is in
code as `StandardVtolChannels` in `Vehicle/VehicleActuators.cs`.

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

The airframe has no rudder. The X500 uses channels 0 to 3 in the same order. When the team's
aircraft gets its own PX4 airframe, update `StandardVtolChannels`, the builder and this table
together.

## Tests

Edit-mode unit tests cover MAVLink encoding (checked against frames from pymavlink), frame
conversions, geodetic math, the magnetic field table and actuators.

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
| IMU noise and latency | `ImuModel` on the vehicle's `IMU` object |
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
