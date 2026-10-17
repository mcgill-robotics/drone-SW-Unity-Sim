# Setup

## Requirements

| Component | Version | Needed for |
|---|---|---|
| Unity | 6000.6.0f1 (install through Unity Hub) | The sim |
| PX4-Autopilot | v1.17.0, built for SITL | Flight control |
| Python 3 with `pymavlink` | | `tools/px4_flight_test.py` |
| QGroundControl | | Optional: monitoring, parameters, missions |
| ROS 2 with Micro XRCE-DDS agent v2.4.x and `ros_tcp_endpoint` | | Optional: ROS 2 topics |

The sim has been run on macOS (Apple silicon) only. PX4 SITL also builds on Ubuntu; Windows
needs WSL2 for PX4. Neither has been tried with this project.

## Unity project

```bash
git clone https://github.com/mcgill-robotics/drone-SW-Unity-Sim.git
```

Add the folder in Unity Hub and open it with 6000.6.0f1. The first import takes several minutes.

## PX4 SITL

Build once, with the same PX4 release as the flight controller.

```bash
git clone --recursive -b v1.17.0 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

macOS:

```bash
xcode-select --install
brew install cmake ninja
python3 -m venv .venv && .venv/bin/pip install -r Tools/setup/requirements.txt
source .venv/bin/activate
ulimit -S -n 2048
make px4_sitl
```

If the build fails with C++17 errors in Gazebo code, CMake has found Homebrew's protobuf. The sim
does not use Gazebo, so disable it:

```bash
make px4_sitl CMAKE_ARGS="-DCMAKE_DISABLE_FIND_PACKAGE_Protobuf=TRUE"
```

Ubuntu:

```bash
bash ./Tools/setup/ubuntu.sh --no-sim-tools
make px4_sitl
```

## Running

Start Unity first, then PX4. QGroundControl, ROS 2 and scripts can connect at any time.

1. In Unity, open the scene for the vehicle you want and press Play. The overlay in the top left
   of the Game view reads `PX4 SITL: waiting on TCP 4560`.

   | Scene | Vehicle | `PX4_SIM_MODEL` |
   |---|---|---|
   | `Assets/_Project/Scenes/FlightX500.unity` | PX4's X500 quadcopter | `none_iris` |
   | `Assets/_Project/Scenes/Flight.unity` | Quadplane | `gazebo-classic_standard_vtol` |

   <!-- todo: image of the waiting overlay -->

2. Start PX4 from the `PX4-Autopilot` directory with the airframe that matches the scene:

   ```bash
   PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4 -d
   ```

   `-d` runs PX4 without its interactive shell; drop it to get the `pxh>` prompt.

3. The overlay changes to `PX4 SITL: connected lockstep=on`. Wait for `Ready for takeoff!` in the
   PX4 output before arming.

   <!-- todo: image of the connected overlay -->

Notes:

- Run the `px4` binary directly, as above. `make px4_sitl gazebo-classic_standard_vtol` would
  also launch Gazebo.
- The `gz_*` airframes do not work: they talk to Gazebo over its own transport, not MAVLink.
- If Unity runs on another machine, set `PX4_SIM_HOSTNAME=<unity host>` in PX4's environment.
- Unity's Run In Background is enabled, so the sim keeps stepping while another window has focus.

## Standalone build

A built player runs the sim without the Unity editor. Builds for Linux, Windows and macOS are
attached to each release on GitHub, or can be built from the Actions tab (Build, Run workflow).
Start the player instead of pressing Play, then start PX4 as above.

Options go after the executable name:

| Option | Default | Effect |
|---|---|---|
| `--scene NAME` | `FlightX500` | Vehicle scene: `FlightX500` or `Flight` (quadplane) |
| `--px4-port PORT` | 4560 | TCP port PX4 SITL connects to |
| `--ros-ip IP` | 127.0.0.1 | ROS TCP endpoint address |
| `--ros-port PORT` | 10000 | ROS TCP endpoint port |
| `--no-ros` | | Don't connect or publish to ROS |
| `--config PATH` | `StreamingAssets/sim_config.json` | Camera config file, see [Camera config](Development#camera-config) |

```bash
./drone-sim.x86_64 --scene Flight --ros-ip 192.168.1.20                            # Linux
drone-sim.app/Contents/MacOS/drone-unity-sim --scene Flight                        # macOS
```

On macOS, clear the quarantine flag on a downloaded build first: `xattr -cr drone-sim.app`.
The `PX4_HOME_*` variables ([Home location](#home-location)) work the same way for a player.
Unity's own player options also apply, for example `-screen-fullscreen 0 -screen-width 1280
-screen-height 720` for a window.

## Flying

### QGroundControl

Open QGC. PX4 SITL sends MAVLink to UDP 14550 on localhost, where QGC listens by default. For
QGC on another machine, set PX4 parameter `MAV_0_BROADCAST` to 1 or add a UDP link in QGC.

`missions/vtol_flyaround.plan` is a demo mission for `Flight.unity` at the default home: VTOL
takeoff to 50 m, a lap of about 1.2 km in fixed-wing flight, VTOL landing at home. In QGC:
Plan view, File, Open, then Upload; then Start Mission in the Fly view.

PX4 writes flight logs (`.ulg`) to `PX4-Autopilot/build/px4_sitl_default/rootfs/log/`.

### Flight test script

Flies the vehicle through PX4's MAVLink API port (UDP 14540), so it can run alongside QGC.
From the project folder, with Unity playing and PX4 connected:

```bash
pip install pymavlink
python3 tools/px4_flight_test.py hover   # any vehicle: take off to 5 m, hover, land
python3 tools/px4_flight_test.py vtol    # quadplane: take off to 40 m, transition, 60 s fixed wing, transition back, land
```

The script prints PX4's state estimate once a second and ends with `PASS`, or with `FAIL` and a
non-zero exit code naming the step that failed. `--help` lists the options.

### Keyboard, without PX4

Press `M` in the Game view to toggle keyboard flying. Keys are listed in
[Development](Development#keys).

## Home location

The default home is PX4's stock SITL location in Zurich (47.397742, 8.545594, 488 m).

PX4 derives its home from the GPS that Unity sends, so `PX4_HOME_LAT`, `PX4_HOME_LON` and
`PX4_HOME_ALT` must be set in Unity's environment, not PX4's. Either:

- Export them and launch the editor from the same terminal. An editor launched from the Dock or
  Unity Hub does not see them.

  ```bash
  export PX4_HOME_LAT=45.5048 PX4_HOME_LON=-73.5772 PX4_HOME_ALT=30   # Montreal
  /Applications/Unity/Hub/Editor/6000.6.0f1/Unity.app/Contents/MacOS/Unity -projectPath .   # from the project folder
  ```

- Or edit the fields on the `GeoOrigin` component of the `Managers` object in the scene. This
  edit is lost when the scenes are rebuilt (see [Vehicle config](Development#vehicle-config)).

The simulated magnetic field follows the home location automatically. drone-2027's `check_lap`
uses Montreal waypoints and needs the Montreal home above.

## ROS 2

Optional. The sim runs without it. Two separate bridges are involved:

- PX4's topics (`/fmu/...`), for code that commands PX4, go through the Micro XRCE-DDS agent.
  Use agent v2.4.x; v3 does not work with PX4 v1.17's client.

  ```bash
  MicroXRCEAgent udp4 -p 8888
  ```

- Unity's topics (cameras, ground truth, `/clock`) go through the ROS TCP endpoint. Unity
  connects to `127.0.0.1:10000`; change the address on the `ROSConnection` component of
  `Managers` if the endpoint runs elsewhere.

  ```bash
  ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
  ```

On macOS, run Unity and PX4 natively and ROS 2 in a Linux container with UDP 8888 and TCP 10000
reachable from the host. Every port and topic is listed in [Interfaces](Architecture#interfaces).

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| PX4 stops after `selected parameter backup file` and never connects | A PX4 startup command hung on PX4's local socket, or an old PX4 is still running. Kill everything and relaunch: `pkill -9 -f px4_sitl_default/bin/px4; pkill -9 -f px4-; rm -f /tmp/px4_lock-0 /tmp/px4-sock-0` |
| Preflight checks fail after switching vehicles | Parameters reset when the airframe changes, but the stored mission does not. Stop PX4 and delete `build/px4_sitl_default/rootfs/dataman` |
| Unity console: can't listen on TCP 4560 | Another Unity instance or simulator holds the port |
| PX4 logs `poll timeout` | Unity is not stepping: play mode is paused or stopped |
| Unity console: `lockstep paused` | PX4 sent no controls for three consecutive 1 s timeouts. Unity steps on its own until controls arrive again. Usually PX4 was killed without closing the socket |
| QGC stays disconnected | Check that PX4 is running and nothing else holds UDP 14550 |
| drone-2027 checks lose the XRCE-DDS agent in Docker on macOS | Docker Desktop's UDP forwarding breaks when the agent restarts, and each check starts and stops its own agent. Run the agent as the container's main process and pass `--no-agent` to each check |
