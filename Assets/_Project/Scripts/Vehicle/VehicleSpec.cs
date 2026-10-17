using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// A vehicle described in a JSON file, in the terms the mechanical team works in, so the
    /// simulated aircraft changes without touching code. Shipped specs are in
    /// StreamingAssets/vehicles/NAME.json; VehicleBuilder turns one into a vehicle at startup and
    /// tools/px4_airframe.py turns the same file into the matching PX4 airframe.
    ///
    /// Units are SI. Positions and directions are in the aircraft body frame FRD (x forward,
    /// y right, z down), measured from a datum the spec chooses, such as the CAD origin. The
    /// vehicle's transform sits at the datum.
    /// </summary>
    [Serializable]
    public class VehicleSpec
    {
        public string name;
        public string description;
        public MassSpec massProperties = new MassSpec();
        public BoxSpec collider = new BoxSpec();
        public RotorSpec[] rotors = new RotorSpec[0];
        public WingSpec[] wings = new WingSpec[0];
        public SurfaceSpec[] controlSurfaces = new SurfaceSpec[0];
        public MountSpec imu = new MountSpec();
        public CameraMountSpec[] cameras = new CameraMountSpec[0];
        public VisualSpec[] visuals = new VisualSpec[0];
        public Px4Spec px4 = new Px4Spec();

        /// <summary>Directory the spec was loaded from; thrust tables are relative to it.</summary>
        [NonSerialized] public string directory = "";

        public const float Gravity = 9.81f;

        /// <summary>A name in StreamingAssets/vehicles, or a path to a .json file.</summary>
        public static string Resolve(string nameOrPath)
        {
            if (nameOrPath.EndsWith(".json", StringComparison.OrdinalIgnoreCase) || nameOrPath.Contains("/") || nameOrPath.Contains("\\"))
                return Path.GetFullPath(nameOrPath);
            return Path.Combine(Application.streamingAssetsPath, "vehicles", nameOrPath + ".json");
        }

        public static VehicleSpec Load(string path)
        {
            var spec = Parse(File.ReadAllText(path));
            spec.directory = Path.GetDirectoryName(path);
            return spec;
        }

        public static VehicleSpec Parse(string json)
        {
            var spec = new VehicleSpec();
            JsonUtility.FromJsonOverwrite(json, spec);
            return spec;
        }

        public IEnumerable<RotorSpec> LiftRotors => rotors.Where(r => r.IsLift);

        /// <summary>Everything that stops this spec from building. Empty when it can be used.</summary>
        public List<string> Problems()
        {
            var p = new List<string>();
            if (string.IsNullOrEmpty(name)) p.Add("name is empty");
            if (massProperties.mass <= 0f) p.Add("massProperties.mass must be positive");
            if (!Is3(massProperties.cg)) p.Add("massProperties.cg needs 3 values");
            var i = massProperties.inertia;
            if (i.ixx <= 0f || i.iyy <= 0f || i.izz <= 0f) p.Add("massProperties.inertia ixx, iyy and izz must be positive");
            else
            {
                // Large products can make a tensor with a positive diagonal that no body has; Unity
                // rejects it and keeps the collider's inertia.
                var pm = PrincipalInertia();
                if (!(pm.x > 0f)) p.Add($"massProperties.inertia has principal moments {pm.x:0.####}, {pm.y:0.####}, {pm.z:0.####}, not all positive; check the signs of ixy, ixz and iyz");
            }
            if (!Is3(collider.center)) p.Add("collider.center needs 3 values");
            if (!Is3(collider.size) || collider.size.Any(s => s <= 0f)) p.Add("collider.size needs 3 positive values");

            var channels = new Dictionary<int, string>();
            void Channel(int ch, string who)
            {
                if (ch < 0 || ch >= VehicleActuators.ChannelCount) p.Add($"{who}: channel must be 0 to {VehicleActuators.ChannelCount - 1}");
                else if (channels.TryGetValue(ch, out string other)) p.Add($"{who}: channel {ch} is also used by {other}");
                else channels[ch] = who;
            }

            if (rotors.Length == 0) p.Add("no rotors");
            foreach (var r in rotors)
            {
                string who = $"rotor {r.name}";
                Channel(r.channel, who);
                if (!Is3(r.position)) p.Add($"{who}: position needs 3 values");
                if (!Is3(r.axis) || ToVector(r.axis).sqrMagnitude < 1e-6f) p.Add($"{who}: axis needs 3 values, not all zero");
                if (r.direction != "ccw" && r.direction != "cw") p.Add($"{who}: direction must be ccw or cw");
                if (string.IsNullOrEmpty(r.thrustTable) && r.maxThrust <= 0f) p.Add($"{who}: needs maxThrust or a thrustTable");
                if (r.momentRatio < 0f) p.Add($"{who}: momentRatio can't be negative");
                if (r.timeConstant <= 0f) p.Add($"{who}: timeConstant must be positive");
                if (r.idle < 0f || r.idle > 0.5f) p.Add($"{who}: idle must be 0 to 0.5");
                if (!string.IsNullOrEmpty(r.thrustTable))
                {
                    if (r.idle != 0f) p.Add($"{who}: idle must be 0 with a thrustTable; the table's first row is the thrust armed at zero throttle");
                    try { r.LoadTable(directory); }
                    catch (Exception e) { p.Add($"{who}: thrustTable {r.thrustTable}: {e.Message}"); }
                }
            }
            foreach (var s in controlSurfaces)
            {
                string who = $"control surface {s.name}";
                Channel(s.channel, who);
                if (SurfaceSpec.Px4Type(s.type) < 0) p.Add($"{who}: unknown type {s.type} (one of {string.Join(", ", SurfaceSpec.Types)})");
                if (!Is3(s.torque)) p.Add($"{who}: torque needs 3 values (roll, pitch, yaw)");
                if (s.momentCoefficient <= 0f) p.Add($"{who}: momentCoefficient must be positive");
                if (s.timeConstant <= 0f) p.Add($"{who}: timeConstant must be positive");
            }
            if (wings.Length > 1) p.Add("at most one wing: it models the whole airframe, tail included");
            foreach (var w in wings)
            {
                if (!Is3(w.aeroCenter)) p.Add("wing: aeroCenter needs 3 values");
                if (w.area <= 0f || w.span <= 0f || w.chord <= 0f) p.Add("wing: area, span and chord must be positive");
            }
            if (!Is3(imu.position)) p.Add("imu.position needs 3 values");
            foreach (var c in cameras)
                if (string.IsNullOrEmpty(c.name) || !Is3(c.position)) p.Add($"camera {c.name}: needs a name and 3 position values");
            foreach (var v in visuals)
            {
                string who = $"visual {v.name}";
                if (!Is3(v.position)) p.Add($"{who}: position needs 3 values");
                if (v.shape == "box")
                {
                    if (!Is3(v.size) || v.size.Any(s => s <= 0f)) p.Add($"{who}: size needs 3 positive values");
                }
                else if (v.shape == "capsule" || v.shape == "cylinder")
                {
                    if (v.diameter <= 0f || v.length <= 0f) p.Add($"{who}: diameter and length must be positive");
                    if (!string.IsNullOrEmpty(v.axis) && v.axis != "x" && v.axis != "y" && v.axis != "z") p.Add($"{who}: axis must be x, y or z");
                }
                else p.Add($"{who}: shape must be box, capsule or cylinder");
            }
            if (string.IsNullOrEmpty(px4.baseAirframe)) p.Add("px4.baseAirframe is empty");
            if (px4.autostartId <= 0) p.Add("px4.autostartId must be positive");
            return p;
        }

        /// <summary>
        /// Numbers that tell whether the aircraft is plausible, plus warnings. Assumes the lift
        /// rotors share the weight equally.
        /// </summary>
        public string Summary(out List<string> warnings)
        {
            warnings = new List<string>();
            var sb = new StringBuilder();
            float m = massProperties.mass, weight = m * Gravity;
            sb.AppendLine($"Vehicle {name}: {m:0.###} kg, {rotors.Length} rotors, {controlSurfaces.Length} control surfaces, {(wings.Length > 0 ? "wing" : "no wing")}");

            var lift = LiftRotors.ToList();
            if (lift.Count > 0)
            {
                float maxLift = lift.Sum(r => r.MaxThrust(directory));
                float tw = maxLift / weight;
                float hover = HoverThrottle();
                sb.AppendLine($"  Lift: {lift.Count} rotors, {maxLift:0.#} N max, thrust/weight {tw:0.00}, hover throttle {(float.IsNaN(hover) ? "out of reach" : $"{hover * 100f:0}%")}");
                if (tw < 1.5f) warnings.Add($"lift thrust/weight is {tw:0.00}; PX4 needs headroom to control attitude, aim for 1.8 or more");
                if (float.IsNaN(hover)) warnings.Add($"a lift rotor can't make its {weight / lift.Count:0.#} N share of the weight; the vehicle can't hover");
                else if (hover > 0.8f) warnings.Add($"hover throttle is {hover * 100f:0}%, which leaves little control margin");

                // Centre of gravity against the centre of the lift rotors, in the horizontal plane.
                Vector2 centroid = Vector2.zero;
                foreach (var r in lift) centroid += new Vector2(r.position[0], r.position[1]);
                centroid /= lift.Count;
                float arm = lift.Average(r => (new Vector2(r.position[0], r.position[1]) - centroid).magnitude);
                Vector2 off = new Vector2(massProperties.cg[0], massProperties.cg[1]) - centroid;
                sb.AppendLine($"  CG is {off.x * 100f:0.#} cm forward and {off.y * 100f:0.#} cm right of the lift rotor centre (rotor arm {arm:0.###} m)");
                if (arm > 0f && off.magnitude > 0.1f * arm)
                    warnings.Add($"CG is {off.magnitude * 100f:0.#} cm from the lift rotor centre, over 10% of the rotor arm; hover will need uneven motor outputs");
            }
            else warnings.Add("no lift rotors (rotor axis pointing up); the vehicle can't hover");

            foreach (var r in rotors.Where(r => r.IsForward))
                sb.AppendLine($"  Forward thrust ({r.name}): {r.MaxThrust(directory):0.#} N, {r.MaxThrust(directory) / weight:0.00} of weight");

            foreach (var w in wings)
            {
                float clMax = w.cl0 + w.clAlpha * w.stallAlphaDeg * Mathf.Deg2Rad;
                float stall = Mathf.Sqrt(2f * weight / (Wing.SeaLevelDensity * w.area * clMax));
                sb.AppendLine($"  Wing: {w.span:0.##} m span, {w.area:0.###} m^2, loading {m / w.area:0.#} kg/m^2, stall speed {stall:0.#} m/s at CL {clMax:0.##}");
            }

            var p = PrincipalInertia();
            if (p.x + p.y < p.z || p.y + p.z < p.x || p.z + p.x < p.y)
                warnings.Add("the inertia values break the triangle inequality, so no real body has them; check the CAD export");

            var estimates = Sources().Where(s => s.source == "" || s.source == "estimate").Select(s => s.what).ToList();
            if (estimates.Count > 0) sb.AppendLine($"  Estimated, not measured: {string.Join(", ", estimates)}");
            return sb.ToString().TrimEnd();
        }

        /// <summary>
        /// Throttle (0..1, as PX4 commands it) at which the lift rotors carry the weight, or NaN
        /// if they can't.
        /// </summary>
        public float HoverThrottle()
        {
            var lift = LiftRotors.ToList();
            if (lift.Count == 0) return float.NaN;
            float perRotor = massProperties.mass * Gravity / lift.Count;
            var needed = lift.Select(r => r.ThrottleFor(perRotor, directory)).ToList();
            // LINQ's Max skips NaN, so check for a rotor that falls short first.
            return needed.Any(float.IsNaN) ? float.NaN : needed.Max();
        }

        IEnumerable<(string what, string source)> Sources()
        {
            yield return ("mass properties", massProperties.source ?? "");
            foreach (var r in rotors) yield return ($"rotor {r.name}", r.source ?? "");
            foreach (var w in wings) yield return ("wing", w.source ?? "");
            foreach (var s in controlSurfaces) yield return ($"control surface {s.name}", s.source ?? "");
        }

        /// <summary>Principal moments of inertia (ascending).</summary>
        public Vector3 PrincipalInertia()
        {
            Frames.Eigen(massProperties.inertia.Matrix(), out double[] values, out _);
            var v = values.OrderBy(x => x).ToArray();
            return new Vector3((float)v[0], (float)v[1], (float)v[2]);
        }

        public static bool Is3(float[] a) => a != null && a.Length == 3;
        public static Vector3 ToVector(float[] a) => new Vector3(a[0], a[1], a[2]);
    }

    [Serializable]
    public class MassSpec
    {
        public float mass;
        [Tooltip("Centre of gravity, FRD from the datum")] public float[] cg = { 0f, 0f, 0f };
        public InertiaSpec inertia = new InertiaSpec();
        public string source;
    }

    /// <summary>
    /// Inertia about the centre of gravity, in the FRD axes, kg m^2. The products are the
    /// integrals ixy = sum(m x y) as CAD tools list them; the tensor's off-diagonal terms are
    /// their negatives.
    /// </summary>
    [Serializable]
    public class InertiaSpec
    {
        public float ixx, iyy, izz, ixy, ixz, iyz;

        public double[,] Matrix() => new double[,]
        {
            { ixx, -ixy, -ixz },
            { -ixy, iyy, -iyz },
            { -ixz, -iyz, izz }
        };
    }

    [Serializable]
    public class BoxSpec
    {
        public float[] center = { 0f, 0f, 0f };
        [Tooltip("Extent along x, y, z (FRD)")] public float[] size;
    }

    [Serializable]
    public class RotorSpec
    {
        public string name;
        [Tooltip("PX4 output channel, 0-based (PWM_MAIN_FUNC<channel+1>)")] public int channel;
        public float[] position;
        [Tooltip("Thrust direction, FRD. [0,0,-1] for a lift rotor, [1,0,0] for a pusher")]
        public float[] axis = { 0f, 0f, -1f };
        [Tooltip("ccw or cw, seen from the side the thrust points to (from above for a lift rotor)")]
        public string direction;
        public float propDiameter;
        [Tooltip("Thrust at full throttle (N); thrust is maxThrust x speed^2. Ignored with a thrustTable")]
        public float maxThrust;
        [Tooltip("Reaction torque per newton of thrust (m), PX4's |CA_ROTORn_KM|")]
        public float momentRatio = 0.016f;
        [Tooltip("Spin-up time constant (s)")] public float timeConstant = 0.05f;
        [Tooltip("Speed when armed at zero throttle, as a fraction of full. 0 with a thrustTable")] public float idle;
        [Tooltip("CSV next to the spec: throttle as PX4 commands it (0..1), thrust (N), optional torque (N m)")]
        public string thrustTable;
        public string source;

        [NonSerialized] public float[] tableThrottle, tableThrust, tableTorque;

        public Vector3 AxisFrd => VehicleSpec.ToVector(axis).normalized;
        public bool IsLift => AxisFrd.z < -0.7f;
        public bool IsForward => AxisFrd.x > 0.7f;
        public int Spin => direction == "cw" ? -1 : 1;

        /// <summary>Reads thrustTable once. Throws with a readable message if the file is bad.</summary>
        public void LoadTable(string directory)
        {
            if (string.IsNullOrEmpty(thrustTable) || tableThrottle != null) return;
            var rows = new List<float[]>();
            foreach (string raw in File.ReadAllLines(Path.Combine(directory ?? "", thrustTable)))
            {
                string line = raw.Trim();
                if (line.Length == 0 || line.StartsWith("#") || !(char.IsDigit(line[0]) || line[0] == '.' || line[0] == '-')) continue;
                rows.Add(line.Split(',').Select(c => float.Parse(c.Trim(), CultureInfo.InvariantCulture)).ToArray());
            }
            if (rows.Count < 2) throw new FormatException("needs at least 2 rows of throttle,thrust");
            if (rows.Any(r => r.Length < 2 || r.Length != rows[0].Length)) throw new FormatException("every row needs the same columns: throttle,thrust[,torque]");
            for (int i = 1; i < rows.Count; i++)
                if (rows[i][0] <= rows[i - 1][0]) throw new FormatException("throttle must increase row by row");
            tableThrottle = rows.Select(r => r[0]).ToArray();
            tableThrust = rows.Select(r => r[1]).ToArray();
            tableTorque = rows[0].Length > 2 ? rows.Select(r => r[2]).ToArray() : null;
        }

        public float MaxThrust(string directory)
        {
            if (string.IsNullOrEmpty(thrustTable)) return maxThrust;
            LoadTable(directory);
            return tableThrust.Max();
        }

        /// <summary>Throttle PX4 has to command for this thrust, or NaN if it is out of reach.</summary>
        public float ThrottleFor(float thrust, string directory)
        {
            if (string.IsNullOrEmpty(thrustTable))
            {
                if (thrust > maxThrust) return float.NaN;
                float speed = Mathf.Sqrt(thrust / maxThrust);
                return Mathf.Clamp01((speed - idle) / (1f - idle));
            }
            // The table is already in PX4's throttle, so idle doesn't apply.
            LoadTable(directory);
            if (thrust > tableThrust.Max()) return float.NaN;
            return Mathf.Clamp01(Rotor.Interpolate(tableThrust, tableThrottle, thrust));
        }
    }

    /// <summary>
    /// Whole-airframe aerodynamics; the fields match the Wing component. The defaults are the
    /// placeholder quadplane's and only plausible, not measured.
    /// </summary>
    [Serializable]
    public class WingSpec
    {
        [Tooltip("Aerodynamic centre, FRD from the datum")] public float[] aeroCenter = { 0f, 0f, 0f };
        public float area, span, chord;
        public float cl0 = 0.3f, clAlpha = 5f, stallAlphaDeg = 15f, clMaxPostStall = 0.6f;
        public float cd0 = 0.04f, oswald = 0.8f, cyBeta = -0.3f;
        public float cmAlpha = -0.6f, cnBeta = 0.08f, clBeta = -0.05f;
        public float clp = -0.5f, cmq = -8f, cnr = -0.1f;
        public string source;
    }

    [Serializable]
    public class SurfaceSpec
    {
        public string name;
        public int channel;
        [Tooltip("PX4 control surface type, e.g. left_aileron, elevator")] public string type;
        [Tooltip("Roll, pitch, yaw effect of a positive deflection: PX4's CA_SV_CSn_TRQ_R/P/Y")]
        public float[] torque;
        [Tooltip("Moment per unit torque and deflection, divided by dynamic pressure (m^3)")]
        public float momentCoefficient;
        [Tooltip("Servo time constant (s)")] public float timeConstant = 0.05f;
        public string source;

        /// <summary>CA_SV_CSn_TYPE values, in order from 1.</summary>
        public static readonly string[] Types =
        {
            "left_aileron", "right_aileron", "elevator", "rudder", "left_elevon", "right_elevon",
            "left_v_tail", "right_v_tail", "left_flap", "right_flap", "airbrake", "custom",
            "left_a_tail", "right_a_tail", "single_channel_aileron", "steering_wheel", "left_spoiler", "right_spoiler"
        };

        /// <summary>PX4's CA_SV_CSn_TYPE for a type name, or -1.</summary>
        public static int Px4Type(string type)
        {
            int i = Array.IndexOf(Types, type);
            return i < 0 ? -1 : i + 1;
        }
    }

    [Serializable]
    public class MountSpec
    {
        public float[] position = { 0f, 0f, 0f };
    }

    [Serializable]
    public class CameraMountSpec
    {
        [Tooltip("Matches the camera's entry in sim_config.json")] public string name;
        public float[] position;
        [Tooltip("Tilt below the nose direction: 0 looks forward, 90 straight down")] public float pitchDownDeg;
    }

    /// <summary>A primitive shape for looks only. Box size is the extent along x, y, z.</summary>
    [Serializable]
    public class VisualSpec
    {
        public string name;
        public string shape;
        public float[] position = { 0f, 0f, 0f };
        public float[] size;
        public float yawDeg;
        [Tooltip("Long axis of a capsule or cylinder: x, y or z")] public string axis = "z";
        public float diameter, length;
    }

    [Serializable]
    public class Px4Spec
    {
        [Tooltip("Stock PX4 SITL airframe whose tuning the generated airframe starts from")]
        public string baseAirframe;
        [Tooltip("SYS_AUTOSTART id for the generated airframe")] public int autostartId;
    }
}
