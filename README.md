# drone-unity-sim

A Unity simulator for the drone team's PX4 aircraft. Unity simulates the physics, sensors and
cameras; PX4 SITL connects to it over the same MAVLink HIL interface it uses with Gazebo and flies
the vehicle. QGroundControl, ROS 2 (over uXRCE-DDS) and MAVLink scripts talk to PX4 unchanged.

Two vehicles are included: PX4's X500 quadcopter and a quadplane standing in for the team's
aircraft.

Documentation is in the [wiki](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki):
[Setup](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki/Setup),
[Architecture](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki/Architecture),
[Development](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki/Development).

## Quick start

Requires Unity 6000.6.0f1 and a PX4 v1.17.0 SITL build (see
[Setup](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki/Setup)).

1. Open the project in Unity, open `Assets/_Project/Scenes/FlightX500.unity` and press Play.
2. From the `PX4-Autopilot` directory:

   ```bash
   PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4 -d
   ```

3. The overlay in the Game view changes to `PX4 SITL: connected lockstep=on`.
4. Fly it with QGroundControl, or from the project folder:

   ```bash
   pip install pymavlink
   python3 tools/px4_flight_test.py hover
   ```

For the quadplane, play `Flight.unity` and start PX4 with
`PX4_SIM_MODEL=gazebo-classic_standard_vtol`.

## Layout

| Path | Contents |
|---|---|
| `Assets/_Project/Scripts/` | The sim: PX4 bridge, vehicle physics, sensors, ROS publishers |
| `Assets/Editor/DroneSim/SceneBuilder.cs` | Generates both scenes and holds every vehicle number |
| `Assets/StreamingAssets/sim_config.json` | Camera topics, frames, resolution, rate and format, read at startup (editable in builds) |
| `Assets/Tests/EditMode/` | Unit tests |
| `tools/px4_flight_test.py` | Scripted hover or VTOL flight through PX4, with pass/fail exit codes |
| `missions/` | QGroundControl mission for the quadplane |
