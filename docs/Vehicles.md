# Vehicles

A vehicle is a JSON spec in `Assets/StreamingAssets/vehicles/`. `VehicleLoader` reads it when
the scene starts, checks it, builds the aircraft from it and logs a summary. `tools/px4_airframe.py`
turns the same file into a PX4 airframe so PX4's model of the aircraft matches the sim's. Two specs
are included: `x500.json` (PX4's X500 quadcopter, numbers from PX4's Gazebo model) and
`quadplane.json` (a placeholder for the team's aircraft).

The code is in `Assets/_Project/Scripts/Vehicle/`: `VehicleSpec.cs` (format, checks, summary),
`VehicleBuilder.cs` (spec to GameObject), `VehicleLoader.cs`.

## Selecting a vehicle

- Editor: the `Vehicle` field on the `VehicleLoader` component of `Managers` in
  `Assets/_Project/Scenes/Flight.unity`. Default `x500`.
- Built player: `--vehicle x500`, `--vehicle quadplane` or `--vehicle path/to/spec.json`
  ([Standalone build](Setup#standalone-build)). A value ending in `.json` or containing a slash is
  a path; anything else is a name in `StreamingAssets/vehicles`.

Start PX4 with an airframe that matches the vehicle: the stock one ([Running](Setup#running)) or
one generated from the spec ([PX4 airframe](#px4-airframe)).

## Frame, datum and units

SI units throughout: m, kg, kg m^2, N, N m, s. Angles in degrees where a field name ends in
`Deg`.

Positions and directions are in the aircraft body frame FRD: x forward, y right, z down. They are
measured from a datum the spec picks, such as the CAD origin; the spec does not say where the
datum is, so use the same one for every field and say which in `description`. The vehicle's
Unity transform sits at the datum, and the aircraft is spawned with the bottom of its collider
10 cm above the ground.

A vector field is a 3-element array `[x, y, z]`. Every vector in the spec must have exactly three
values.

## Fields

Fields not listed as optional are required. Where a default is given, leaving the field out uses
the default.

### Top level

| Field | Meaning |
|---|---|
| `name` | Vehicle name. Also names the generated PX4 airframe (`unity_<name>`) |
| `description` | Free text: what the vehicle is, where the datum is, what the numbers are based on |
| `massProperties` | [Mass properties](#massproperties) |
| `collider` | [Collider](#collider) |
| `rotors` | List of [rotors](#rotors), at least one |
| `wings` | List of [wings](#wings), optional. The quadplane has one; the X500 none |
| `controlSurfaces` | List of [control surfaces](#controlsurfaces), optional |
| `imu` | [IMU mount](#imu) |
| `cameras` | List of [camera mounts](#cameras), optional |
| `visuals` | List of [visual shapes](#visuals), optional |
| `px4` | [PX4 airframe settings](#px4) |

### massProperties

| Field | Unit | Meaning |
|---|---|---|
| `mass` | kg | Takeoff mass. Must be positive |
| `cg` | m | Centre of gravity, FRD from the datum |
| `inertia` | kg m^2 | `ixx`, `iyy`, `izz`, `ixy`, `ixz`, `iyz` about the CG, in the FRD axes. The diagonal terms must be positive |
| `source` | | [Source](#sources) of these numbers |

The products of inertia are the integrals as CAD tools list them: `ixy` = sum of m x y, and so on.
The inertia tensor's off-diagonal terms are the negatives of these; the sim does that sign flip
(`InertiaSpec.Matrix` in `VehicleSpec.cs`), so paste the CAD values as they are. If CAD reports
inertia about its origin rather than the CG, translate it to the CG first.

The load summary warns when the principal moments break the triangle inequality
(for example `ixx + iyy < izz`), which no real body does.

### collider

The box PhysX uses for ground contact. It is not the aerodynamic shape and does not need to be
tight.

| Field | Unit | Meaning |
|---|---|---|
| `center` | m | Box centre, FRD from the datum |
| `size` | m | Extent along x, y, z. All three positive |

### rotors

One entry per motor. A rotor is a lift rotor when its axis points up (FRD z component below -0.7)
and a forward rotor when it points forward (x above 0.7); the summary and the hover throttle use
that split.

| Field | Unit | Default | Meaning |
|---|---|---|---|
| `name` | | | Object name in the scene and comment in the generated airframe |
| `channel` | | | PX4 output channel, 0-based: `PWM_MAIN_FUNC<channel+1>`. 0 to 15, unique across rotors and control surfaces. See [Actuator channels](Development#actuator-channels) |
| `position` | m | | Thrust application point, FRD from the datum |
| `axis` | | `[0, 0, -1]` | Thrust direction, FRD. `[0, 0, -1]` for a lift rotor, `[1, 0, 0]` for a pusher. Normalised on load |
| `direction` | | | `ccw` or `cw`, seen from the side the thrust points to: from above for a lift rotor, from in front for a pusher. Sets the sign of the reaction torque and of PX4's `CA_ROTORn_KM` |
| `propDiameter` | m | | Propeller diameter. Only the drawn disc uses it |
| `maxThrust` | N | | Thrust at full speed. Thrust is `maxThrust x speed^2`. Required unless `thrustTable` is set, and ignored when it is |
| `thrustTable` | | | CSV file next to the spec, see [Thrust tables](#thrust-tables). Replaces `maxThrust` |
| `momentRatio` | m | 0.016 | Reaction torque per newton of thrust, PX4's `CA_ROTORn_KM` without the sign. Ignored when the thrust table has a torque column |
| `timeConstant` | s | 0.05 | First-order spin-up lag. Must be positive |
| `idle` | | 0 | Speed when armed at zero throttle, as a fraction of full speed. 0 to 0.5. The Gazebo X500 idles at 0.15 |
| `source` | | | [Source](#sources) |

Speed is `idle + (1 - idle) x throttle`, where throttle is what PX4 commands (0..1), so with
`idle` set the motor produces thrust the moment it is armed, as the Gazebo motor model does.

### Thrust tables

For a motor with test-stand or datasheet data. A CSV next to the spec, named in the rotor's
`thrustTable`:

```
# throttle, thrust (N), torque (N m)
0.0, 0.0, 0.0
0.25, 2.1, 0.03
0.5, 8.3, 0.12
0.75, 18.0, 0.27
1.0, 31.5, 0.49
```

- Columns: throttle 0..1, thrust in N, and optionally reaction torque in N m. Every row has the
  same number of columns.
- Throttle increases row by row; at least two rows. Lines that do not start with a digit, `.` or
  `-` (comments, a header) are skipped.
- Thrust and torque are interpolated linearly between rows and clamped at the ends. Without a
  torque column, torque is thrust x `momentRatio`.
- The table's largest thrust is the rotor's max thrust in the summary and the airframe generator.

A table with a torque column also sets `CA_ROTORn_KM` in the generated airframe, from the
torque/thrust ratio at the highest-thrust row.

### wings

Whole-airframe aerodynamics as one lifting body, applied at the aerodynamic centre; the fields
match the `Wing` component ([Vehicle model](Architecture#vehicle-model)). Only `aeroCenter`,
`area`, `span` and `chord` are required; every coefficient defaults to the placeholder
quadplane's value, which is plausible for a conventional airframe, not measured.

| Field | Unit | Default | Meaning |
|---|---|---|---|
| `aeroCenter` | m | | Aerodynamic centre, FRD from the datum |
| `area` | m^2 | | Reference wing area |
| `span` | m | | Wing span |
| `chord` | m | | Mean chord |
| `cl0` | | 0.3 | Lift coefficient at zero angle of attack |
| `clAlpha` | /rad | 5 | Lift slope |
| `stallAlphaDeg` | deg | 15 | Stall angle |
| `clMaxPostStall` | | 0.6 | Lift coefficient after stall |
| `cd0` | | 0.04 | Parasitic drag coefficient |
| `oswald` | | 0.8 | Oswald efficiency for induced drag |
| `cyBeta` | /rad | -0.3 | Sideforce per sideslip |
| `cmAlpha` | /rad | -0.6 | Pitch stiffness |
| `cnBeta` | /rad | 0.08 | Yaw stiffness (weathercock) |
| `clBeta` | /rad | -0.05 | Roll from sideslip (dihedral effect) |
| `clp`, `cmq`, `cnr` | | -0.5, -8, -0.1 | Roll, pitch and yaw rate damping |
| `source` | | | [Source](#sources) |

The summary reports wing loading and the stall speed computed from `cl0 + clAlpha x stallAlpha`.

### controlSurfaces

Each surface is a moment proportional to dynamic pressure and deflection. The deflection is the
PX4 channel value (-1..1) behind a servo lag.

| Field | Unit | Default | Meaning |
|---|---|---|---|
| `name` | | | Object name in the scene and comment in the generated airframe |
| `channel` | | | PX4 output channel, 0-based, unique across rotors and control surfaces |
| `type` | | | PX4 control surface type, the `CA_SV_CSn_TYPE` name: `left_aileron`, `right_aileron`, `elevator`, `rudder`, `left_elevon`, `right_elevon`, `left_v_tail`, `right_v_tail`, `left_flap`, `right_flap`, `airbrake`, `custom`, `left_a_tail`, `right_a_tail`, `single_channel_aileron`, `steering_wheel`, `left_spoiler`, `right_spoiler` |
| `torque` | | | `[roll, pitch, yaw]` effect of a positive deflection, PX4's `CA_SV_CSn_TRQ_R/P/Y`. Positive is roll right, pitch up, yaw right. `[-0.5, 0, 0]` for a left aileron, `[0.5, 0, 0]` for the right one, `[0, 1, 0]` for an elevator |
| `momentCoefficient` | m^3 | | Moment per unit of `torque` and deflection, divided by dynamic pressure: moment = q x `momentCoefficient` x deflection x `torque`. Must be positive |
| `timeConstant` | s | 0.05 | Servo lag. Must be positive |
| `source` | | | [Source](#sources) |

### imu

| Field | Unit | Meaning |
|---|---|---|
| `position` | m | IMU location, FRD from the datum. Accelerations are computed here, so an offset from the CG shows up in the readings as it does on the real aircraft |

### cameras

One entry per onboard camera. Topics, resolution, rate and format are not here; they are in
`sim_config.json`, matched by `name` ([Camera config](Development#camera-config)). A camera with no
`sim_config.json` entry is not published.

| Field | Unit | Meaning |
|---|---|---|
| `name` | | Must match a camera entry in `sim_config.json`: `ForwardCamera` or `DownCamera` in the shipped config |
| `position` | m | Lens position, FRD from the datum |
| `pitchDownDeg` | deg | Tilt below the nose direction: 0 looks forward, 90 straight down |

### visuals

Primitive shapes for the Game view only. They have no collider and no mass. Optional; without
them only the rotor discs are drawn.

| Field | Unit | Meaning |
|---|---|---|
| `name` | | Object name |
| `shape` | | `box`, `capsule` or `cylinder` |
| `position` | m | Shape centre, FRD from the datum |
| `size` | m | Box only: extent along x, y, z |
| `yawDeg` | deg | Box only: rotation about the vertical axis |
| `axis` | | Capsule and cylinder only: long axis, `x`, `y` or `z`. Default `z` |
| `diameter`, `length` | m | Capsule and cylinder only |

### px4

| Field | Meaning |
|---|---|
| `baseAirframe` | Stock PX4 SITL airframe file the generated airframe sources for its tuning, for example `10016_none_iris` or `1040_gazebo-classic_standard_vtol`. Pick the stock vehicle closest in type (multicopter, standard VTOL, tailsitter) |
| `autostartId` | `SYS_AUTOSTART` id for the generated airframe; its filename starts with it. Must be positive and must not collide with a stock airframe. The shipped specs use 22001 and 22002 |

### Sources

`massProperties`, each rotor, each wing and each control surface take a `source` string saying
where the numbers came from: `cad`, `measured`, `datasheet`, `estimate`, or a longer note such as
`PX4 Gazebo x500 model`. The load summary lists every section whose source is `estimate` or
missing under `Estimated, not measured`, so the list shrinks as real numbers replace guesses.

## Checks and the load summary

On load the spec is checked (`VehicleSpec.Problems`). If anything is wrong, the Unity console
lists every problem in one error and no vehicle is built; the PX4 bridge then disables itself
with its own error, so PX4 cannot connect. Checks include: positive mass and inertia diagonal, 3-element vectors,
channels in range and unique, `ccw`/`cw`, `maxThrust` or a readable `thrustTable`, known control
surface types, `px4` fields set.

A valid spec logs a summary like this one for the X500:

```
[Vehicle] loaded .../StreamingAssets/vehicles/x500.json
Vehicle x500: 2 kg, 4 rotors, 0 control surfaces, no wing
  Lift: 4 rotors, 34.2 N max, thrust/weight 1.74, hover throttle 71%
  CG is 0 cm forward and 0 cm right of the lift rotor centre (rotor arm 0.246 m)
```

| Line | Meaning |
|---|---|
| Lift | Number of lift rotors, their total max thrust, thrust/weight, and the throttle PX4 has to command for the lift rotors to carry the weight if they share it equally (`idle` included) |
| CG | Horizontal offset of the CG from the centroid of the lift rotors, and the mean rotor arm |
| Forward thrust | One line per forward rotor: max thrust and its ratio to weight |
| Wing | Span, area, wing loading and the stall speed at sea level |
| Estimated, not measured | Sections whose `source` is `estimate` or missing |

Warnings, logged separately: thrust/weight under 1.5 (aim for 1.8 or more), hover throttle over
80%, CG more than 10% of the rotor arm from the lift rotor centre, inertia that breaks the triangle
inequality, no lift rotors.

## Example

A trimmed spec showing each section; the pusher's thrust table is for illustration. See
`x500.json` and `quadplane.json` for complete ones.

```json
{
  "name": "quadplane",
  "description": "Placeholder standard VTOL standing in for the team's aircraft.",
  "massProperties": {
    "mass": 6.5,
    "cg": [0, 0, 0],
    "inertia": { "ixx": 0.4333, "iyy": 0.65, "izz": 0.975, "ixy": 0, "ixz": 0, "iyz": 0 },
    "source": "estimate"
  },
  "collider": { "center": [0, 0, 0], "size": [1.2, 2.0, 0.15] },
  "rotors": [
    { "name": "RotorFR", "channel": 0, "position": [0.4, 0.5, -0.03], "axis": [0, 0, -1], "direction": "ccw",
      "propDiameter": 0.4064, "maxThrust": 32, "momentRatio": 0.016, "timeConstant": 0.05, "idle": 0, "source": "estimate" },
    { "name": "Pusher", "channel": 4, "position": [-0.62, 0, 0], "axis": [1, 0, 0], "direction": "ccw",
      "propDiameter": 0.4064, "thrustTable": "pusher_thrust.csv", "timeConstant": 0.05, "idle": 0, "source": "datasheet" }
  ],
  "wings": [
    { "aeroCenter": [0.02, 0, 0], "area": 0.45, "span": 2.0, "chord": 0.22, "source": "estimate" }
  ],
  "controlSurfaces": [
    { "name": "AileronLeft", "channel": 5, "type": "left_aileron", "torque": [-0.5, 0, 0], "momentCoefficient": 0.03, "source": "estimate" },
    { "name": "Elevator", "channel": 7, "type": "elevator", "torque": [0, 1, 0], "momentCoefficient": 0.02, "source": "estimate" }
  ],
  "imu": { "position": [0, 0, 0] },
  "cameras": [
    { "name": "DownCamera", "position": [0.1, 0, 0.08], "pitchDownDeg": 90 }
  ],
  "visuals": [
    { "name": "Fuselage", "shape": "capsule", "position": [0, 0, 0], "axis": "x", "diameter": 0.15, "length": 1.1 },
    { "name": "Wing", "shape": "box", "position": [0.05, 0, 0], "size": [0.22, 2.0, 0.03] }
  ],
  "px4": { "baseAirframe": "1040_gazebo-classic_standard_vtol", "autostartId": 22001 }
}
```

## Adding a vehicle

1. Copy the closest spec in `Assets/StreamingAssets/vehicles/` to `<name>.json` and set `name`
   to match. Put thrust table CSVs in the same folder.
2. Fill in the sections from CAD, the test stand and the datasheets, with `source` on each.
3. Select it ([Selecting a vehicle](#selecting-a-vehicle)), press Play and read the summary and
   warnings in the console. Fix any problems it lists.
4. Generate and install the PX4 airframe ([below](#px4-airframe)) and fly
   `tools/px4_flight_test.py hover` ([Flight test script](Setup#flight-test-script)).
5. Commit the spec and its tables. The scene does not change.

Unity needs a `.meta` file next to each new file under `Assets/`; it creates one when the editor
has focus. Commit it with the spec.

## PX4 airframe

PX4 has its own model of the aircraft in its control allocation parameters (`CA_ROTOR*`,
`CA_SV_CS*`) and output mapping (`PWM_MAIN_FUNC*`). The stock SITL airframes describe PX4's
stock vehicles, not the sim's: `gazebo-classic_standard_vtol` puts its rotors at
+-0.15/0.25 m from the CG where the quadplane's are at +-0.4/0.5 m. The quadplane flies anyway,
because PX4's rate loops absorb the difference, but PX4 allocates roll and pitch with the wrong
lever arms.

`tools/px4_airframe.py` writes an airframe that sources the spec's `px4.baseAirframe` for its
tuning and then sets, from the spec:

- `CA_ROTOR_COUNT`, and for each rotor `CA_ROTORn_PX/PY/PZ` (position relative to the CG),
  `CA_ROTORn_AX/AY/AZ` (axis) and `CA_ROTORn_KM` (`momentRatio`, negative for `cw`).
- `CA_SV_CS_COUNT`, and for each control surface `CA_SV_CSn_TYPE` and `CA_SV_CSn_TRQ_R/P/Y`.
- `PWM_MAIN_FUNC1..16`: motor n on the rotor's channel, servo n on the surface's channel, 0
  elsewhere.
- `MPC_THR_HOVER`: the hover throttle from the summary, clamped to PX4's 0.1 to 0.8 range.

Print it:

```bash
python3 tools/px4_airframe.py quadplane
```

Install it into a PX4 SITL build from the project folder, then start PX4 with it from the
`PX4-Autopilot` directory:

```bash
python3 tools/px4_airframe.py quadplane --px4 path/to/PX4-Autopilot
PX4_SIM_MODEL=unity_quadplane ./build/px4_sitl_default/bin/px4 -d
```

The file lands in `build/px4_sitl_default/etc/init.d-posix/airframes/<autostartId>_unity_<name>`,
next to the stock airframes, and PX4 picks it up at the next start; no rebuild. The vehicle argument
is a spec name or a path to a `.json` file, as for `--vehicle`.

PX4's build regenerates `build/px4_sitl_default/etc` from `ROMFS/` whenever a ROMFS source file
changes (it deletes the directory and re-extracts it, see `ROMFS/CMakeLists.txt`), which removes
the installed file. Run the install again after a PX4 rebuild that touched ROMFS, or after `make
clean`.

Both vehicles fly `tools/px4_flight_test.py` on their stock airframes and on the generated ones.
The generated file is the starting point for the team's aircraft's own PX4 airframe, which will
carry its tuning once the aircraft has been flown.
