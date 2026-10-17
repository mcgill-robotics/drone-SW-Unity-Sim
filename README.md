# drone-unity-sim

A Unity simulator for the drone team's PX4 aircraft. Unity simulates the physics, sensors and
cameras; PX4 SITL connects to it over the same MAVLink HIL interface it uses with Gazebo and flies
the vehicle. QGroundControl, ROS 2 (over uXRCE-DDS) and MAVLink scripts talk to PX4 unchanged.

Vehicles are described in JSON specs (`Assets/StreamingAssets/vehicles/`) and built when the sim
starts. Two are included: PX4's X500 quadcopter and a quadplane standing in for the team's
aircraft. `tools/px4_airframe.py` generates the matching PX4 airframe from a spec.

Documentation is in the [wiki](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki):
[Setup](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki/Setup),
[Architecture](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki/Architecture),
[Development](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki/Development).
The pages live in `docs/` and are published to the wiki when they change on `main`, so edit them
there in the same pull request as the code they describe.

## Quick start

Requires Unity 6000.6.0f1 and a PX4 v1.17.0 SITL build (see
[Setup](https://github.com/mcgill-robotics/drone-SW-Unity-Sim/wiki/Setup)).

1. Open the project in Unity, open `Assets/_Project/Scenes/Flight.unity` and press Play. It flies
   the X500 by default.
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

For the quadplane, set Vehicle to `quadplane` on the `VehicleLoader` component of `Managers` (or
pass `--vehicle quadplane` to a built player) and start PX4 with
`PX4_SIM_MODEL=gazebo-classic_standard_vtol`.

## Layout

| Path | Contents |
|---|---|
| `Assets/_Project/Scripts/` | The sim: PX4 bridge, vehicle physics, sensors, ROS publishers |
| `Assets/StreamingAssets/vehicles/` | Vehicle specs: mass properties, rotors, wing, control surfaces, sensor mounts |
| `Assets/Editor/DroneSim/SceneBuilder.cs` | Generates the flight scene |
| `Assets/StreamingAssets/sim_config.json` | Camera topics, frames, resolution, rate and format, read at startup (editable in builds) |
| `Assets/Tests/EditMode/` | Unit tests |
| `tools/px4_flight_test.py` | Scripted hover or VTOL flight through PX4, with pass/fail exit codes |
| `tools/px4_airframe.py` | Generates a PX4 SITL airframe from a vehicle spec |
| `missions/` | QGroundControl mission for the quadplane |
