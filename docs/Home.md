# drone-SW-Unity-Sim

A Unity simulator for the drone team's PX4 aircraft. Unity simulates the physics, sensors and
cameras. PX4 SITL, the real flight code built for your computer, connects to Unity over the same
MAVLink HIL interface it uses with Gazebo and flies the vehicle. QGroundControl, ROS 2 and MAVLink
scripts talk to PX4 exactly as they would with the real aircraft.

Vehicles are JSON specs built at startup. Two are included: PX4's X500 quadcopter, with the
numbers from PX4's own Gazebo model, and a quadplane standing in for the team's aircraft.

<!-- todo: image of the X500 flying in the sim -->

## Pages

| Page | Contents |
|---|---|
| [Setup](Setup) | Requirements, building PX4, running the sim, flying it, ROS 2, troubleshooting |
| [Architecture](Architecture) | Processes, every port and topic, lockstep, coordinate frames, sensors, vehicle model |
| [Vehicles](Vehicles) | The vehicle spec: frame, every field and unit, thrust tables, load summary, adding a vehicle, generating the PX4 airframe |
| [Development](Development) | Code layout, camera config, actuator channels, tests, CI, keys |

Repository: https://github.com/mcgill-robotics/drone-SW-Unity-Sim
