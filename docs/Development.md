# Development

## Code layout

```
Assets/_Project/Scripts/     DroneSim assembly, namespace DroneSim
  Bridge/      Px4HilBridge (PX4 connection, lockstep), MavlinkCodec
  Vehicle/     VehicleSpec (JSON format, checks, summary), VehicleBuilder, VehicleLoader,
               Rotor, Wing, ControlSurface, VehicleActuators, ManualControl (keyboard flying)
  Sensors/     ImuModel, SensorDelayBuffer, ROS publishers (cameras, lidar, ground truth),
               Vlp16Model (lidar geometry and point layout)
  World/       GeoOrigin (home, GPS), WorldMagneticModel, Wind
  Core/        SimSettings, ROSSettings (sim-only topic names), SimConfig (sim_config.json), ROSClock,
               ROSPublisher base, SimulationResetManager, CommandLineOptions (player options)
  UserCamera/  Chase and free cameras
  Utils/       Noise generators
Assets/Editor/DroneSim/      SceneBuilder (Flight.unity)
Assets/StreamingAssets/      sim_config.json (cameras, lidars), vehicles/ (vehicle specs)
Assets/Tests/EditMode/       Unit tests
Assets/Tests/PlayMode/       Flight scene and PX4 bridge tests (FakePx4 plays PX4's side)
tools/                       px4_flight_test.py, px4_airframe.py, sim_smoke_test.py,
                             coverage_summary.py; tests/ has their unit tests
missions/                    QGroundControl mission for the quadplane
```

Scene-wide components (`SimSettings`, `ROSSettings`, `GeoOrigin`, `Wind`) live on the `Managers`
object and are reached through a static `Instance`. The vehicle is built at startup from its spec
and reached through `SimSettings.Vehicle`; `VehicleLoader.Loaded` holds the spec.

## Vehicle config

Everything about the aircraft (mass properties, rotors, wing, control surfaces, IMU, camera and
lidar mounts, PX4 airframe) is in its spec under `Assets/StreamingAssets/vehicles/`, documented in
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

## Lidar config

The `lidars` list in the same `sim_config.json`. One entry per lidar, matched by `name` to a lidar
mount in the vehicle spec. Each is a simulated Velodyne VLP-16 publishing one
`sensor_msgs/PointCloud2` per revolution.

| Field | Meaning |
|---|---|
| `name` | Lidar mount name in the spec: `Lidar` in the shipped vehicles |
| `topic` | Point cloud topic. `/velodyne_points` is what the Velodyne ROS 2 driver publishes |
| `frameId` | `frame_id` in the header. The driver's default is `velodyne` |
| `rotationHz` | Spin rate, 5 to 20, default 10. Also the publish rate. The lasers fire at a fixed rate, so the azimuth step follows from it: 0.1 degrees at 5 Hz, 0.2 at 10 Hz, 0.4 at 20 Hz |
| `minRange`, `maxRange` | Metres, default 0.5 and 100. A laser whose first hit is nearer or farther gives no point |
| `rangeNoiseStdDev` | Standard deviation of the Gaussian range noise in metres, default 0.015 |

A lidar with no entry or a bad one isn't published, and the Unity console says why.

Each point is 22 bytes: `x`, `y`, `z`, `intensity` (float32), `ring` (uint16, 0 is the lowest
laser) and `time` (float32), in the sensor frame with ROS axes (x forward, y left, z up).

The lidar spins continuously, so the cloud of a moving vehicle is distorted as on the real sensor:
each point is in the sensor frame of the moment its laser fired. `time` is that moment in seconds
from the start of the revolution (0 to just under 0.1 at 10 Hz), and the header stamp is the start
of the revolution, so `stamp + time` is when a point was measured. Code that corrects the
distortion needs both. Check the stamp convention against the real driver's settings: it can be
set to stamp a scan with its first or its last packet.

What the simulation leaves out:

- The pose is sampled once per physics step (4 ms), not once per firing, so the distortion from
  the vehicle's motion comes in steps of about 14 degrees of azimuth at 10 Hz.
- `intensity` is a constant 100. Unity has no reflectivity to measure.
- It sees colliders, not meshes. An object without a collider is invisible to it.
- The vehicle carrying the lidar blocks the lasers that hit its collider, which is a box, not the
  real airframe's shape.

The sim publishes no TF. To view the cloud in RViz2, set the fixed frame to `velodyne`. While the
ROS endpoint is not connected the lidar does not spin, except in the editor, where selecting the
`Lidar` object under `Vehicle` in play mode draws the last scan in the Scene view.

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

| Suite | Covers | Needs |
|---|---|---|
| Edit mode, `Assets/Tests/EditMode/` | MAVLink encoding (checked against frames from pymavlink), frame conversions, geodetic math, the magnetic field table, actuators, the camera and lidar config, the lidar's ray geometry and point packing, the vehicle spec (FRD to Unity, inertia, checks, defaults, thrust tables, hover throttle, shipped specs valid and building) | Unity |
| Play mode, `Assets/Tests/PlayMode/` | `Flight.unity` with each shipped vehicle and no PX4: builds, starts without errors, settles upright on the ground, and its lidar sees the ground but not the vehicle. The PX4 bridge over a real socket with `FakePx4` on the other end: sensors at rest, one HIL_SENSOR per step, lockstep, PX4 stopping and restarting, arming and lift-off | Unity |
| Tools, `tools/tests/` | `px4_airframe.py` (rotor geometry, output functions, hover throttle, checks, constants shared with the C# code), `sim_smoke_test.py`, `coverage_summary.py` | Python 3 |
| `tools/sim_smoke_test.py` | A running sim or player, from PX4's side of the link: sensors at rest, disarmed stays put, armed climbs | Python 3 |
| `tools/px4_flight_test.py` | Takeoff, hover, transition and landing through PX4 ([Setup](Setup#flight-test-script)) | PX4 SITL, pymavlink |

The play-mode tests and the smoke test pick up every spec in `StreamingAssets/vehicles/`, so a new
vehicle is tested without touching them. Any logged error or exception fails a Unity test.

Unity tests:

- In Unity: Window > General > Test Runner > EditMode or PlayMode > Run All.
- From a terminal, with the editor closed, from the project folder (`PlayMode` for play mode):

  ```bash
  /Applications/Unity/Hub/Editor/6000.6.0f1/Unity.app/Contents/MacOS/Unity -batchmode \
    -projectPath . -runTests -testPlatform EditMode -testResults results.xml -logFile test.log
  ```

Coverage: add `-enableCodeCoverage -coverageOptions "generateHtmlReport;assemblyFilters:+DroneSim"`
to the command above (or use Window > Analysis > Code Coverage), then
`python3 tools/coverage_summary.py CodeCoverage` for the table per file.

Tool tests: `python3 -m unittest discover -s tools/tests -v`

Smoke test, against a player or the editor in play mode with no PX4 running:

```bash
./drone-sim.x86_64 -batchmode -nographics --no-ros --vehicle quadplane &
python3 tools/sim_smoke_test.py quadplane
```

## CI

GitHub Actions, using game-ci's Unity images:

| Workflow | Runs on | Does |
|---|---|---|
| `tests.yml` | Pull requests, pushes to `main` that touch the Unity project | Edit- and play-mode tests. Line coverage of the `DroneSim` assembly per file in the job summary, HTML report as the `coverage` artifact |
| `tools.yml` | Pull requests and pushes to `main` that touch `tools/`, vehicle specs or vehicle scripts | Tool tests, no Unity |
| `build.yml` | Manual (Actions tab, Build, Run workflow) or a tag such as `v0.1.0` | Linux, Windows and macOS players, then the smoke test on the Linux player for each vehicle. A tag also drafts a release with the three builds zipped, once the smoke test passes |
| `wiki.yml` | Pushes to `main` that touch `docs/` | Publishes `docs/` to the wiki |

The Unity jobs need a Unity license in the repository secrets: `UNITY_EMAIL`, `UNITY_PASSWORD` and
either `UNITY_SERIAL` (Pro or Student license) or `UNITY_LICENSE` (contents of a Personal `.ulf`
file). Only one Unity job runs at a time so they don't compete for the license.

The Unity test job is still called `Edit-mode tests`, though it runs play mode too: that name is the
required status check in `main`'s ruleset. Rename both together.

## ROS topics

Full list with types, rates and ports: [Interfaces](Architecture#interfaces).

- Camera topics, frames, resolution, rate and format are in `sim_config.json`
  ([Camera config](#camera-config)), and so are the lidar's topic, frame and spin rate
  ([Lidar config](#lidar-config)). The other topic and frame names are on `ROSSettings`.
- Clock and ground truth rates are on their publisher components. The ROS master switch is on
  `SimSettings`.
- `/fmu/...` topics come from PX4 through the XRCE-DDS agent, not from Unity.

## Settings

| Setting | Where |
|---|---|
| Cameras: topics, frames, resolution, rate, format | `Assets/StreamingAssets/sim_config.json` |
| Lidar: topic, frame, spin rate, range, noise | `Assets/StreamingAssets/sim_config.json` |
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
