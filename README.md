# drone-unity-sim

A Unity simulator for PX4. Unity does the physics, sensors and cameras; PX4 SITL runs the real
flight code on top, the same way it would with Gazebo. Anything that talks to PX4 (QGroundControl,
a ROS 2 stack over uXRCE-DDS, MAVLink scripts) works against it unchanged.

```
QGroundControl ── MAVLink UDP 14550 ──┐
MAVLink scripts ─ MAVLink UDP 14540 ──┤
                                      ▼
Unity ──── HIL MAVLink, TCP 4560 ──► PX4 SITL ── uXRCE-DDS, UDP 8888 ──► ROS 2
  │                                                                       ▲
  └──────── cameras, IMU, ground truth, /clock (ROS TCP endpoint) ────────┘
```

## Quick start

1. Open the project in Unity 6.6 and play `Assets/_Project/Scenes/FlightX500.unity`.
2. Build PX4 v1.17.0 SITL once, then start it (details in [docs/running.md](docs/running.md)):
   ```bash
   PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4 -d
   ```
3. The overlay in Unity switches to `PX4 SITL: connected lockstep=on`.
4. Fly it: open QGroundControl, or run `python3 tools/px4_flight_test.py hover`.

For the quadplane, play `Flight.unity` and use `PX4_SIM_MODEL=gazebo-classic_standard_vtol`.

## What's here

| Path | What |
|---|---|
| `Assets/_Project/Scripts/` | The sim, one `DroneSim` assembly: PX4 bridge, vehicle physics, sensors, ROS publishers |
| `Assets/Editor/DroneSim/SceneBuilder.cs` | Generates both scenes and holds every vehicle number. Edit this, not the scenes |
| `Assets/Tests/EditMode/` | Unit tests: MAVLink encoding, frame conversions, geo math, magnetic model, actuators |
| `tools/px4_flight_test.py` | Scripted hover or VTOL flight through PX4 with pass/fail |
| `missions/` | Example QGroundControl mission for the quadplane |
| `docs/` | [running.md](docs/running.md) to set up and run, [architecture.md](docs/architecture.md) for how it works |

## Status

Works: PX4 v1.17.0 SITL in lockstep with both vehicles. The X500 hovers, and the quadplane takes
off, transitions to wing-borne flight, flies missions, transitions back and lands. Tested on
macOS (Apple silicon).

Not done yet:

- The quadplane's mass, thrust and aerodynamics are placeholders, not our aircraft. It flies
  nose-high on the wing and loses a few metres in the transition.
- The world is a flat plane with a runway. No terrain, obstacles or targets.
- No synthetic training-data pipeline.
- No CI. Tests run in the editor.

Parts of the ROS side (publisher base, clock, IMU noise model, camera readback, reset) come from
the AUV team's auv-sim-unity.
