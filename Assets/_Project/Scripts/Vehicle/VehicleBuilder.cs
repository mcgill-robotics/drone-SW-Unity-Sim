using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Builds a vehicle GameObject from a VehicleSpec: rigidbody, collider, rotors, control
    /// surfaces, wing, IMU, ground truth, cameras and placeholder visuals. The transform sits at
    /// the spec's datum. Components look up their neighbours in Awake, which runs as soon as they
    /// are added, so the order below matters: rigidbody first, wing before surfaces, actuators last.
    /// </summary>
    public static class VehicleBuilder
    {
        /// <summary>
        /// Builds the vehicle with its datum at position. Place it there from the start: a rigidbody
        /// created inside the ground and moved out afterwards gets thrown through it. Expects a spec
        /// with no Problems; if building throws anyway, the half-built vehicle is destroyed.
        /// </summary>
        public static GameObject Build(VehicleSpec spec, Vector3 position)
        {
            var root = new GameObject("Vehicle");
            try
            {
                Populate(root, spec, position);
                return root;
            }
            catch
            {
                Object.DestroyImmediate(root);
                throw;
            }
        }

        static void Populate(GameObject root, VehicleSpec spec, Vector3 position)
        {
            root.transform.position = position;
            var rb = root.AddComponent<Rigidbody>();
            rb.mass = spec.massProperties.mass;
            rb.linearDamping = 0f;
            rb.angularDamping = 0.05f;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;

            var col = root.AddComponent<BoxCollider>();
            col.center = Frames.FrdToUnity(spec.collider.center);
            col.size = Abs(Frames.FrdToUnity(spec.collider.size));

            rb.centerOfMass = Frames.FrdToUnity(spec.massProperties.cg);
            Frames.InertiaToUnity(spec.massProperties.inertia, out Vector3 principal, out Quaternion inertiaRotation);
            rb.inertiaTensor = principal;
            rb.inertiaTensorRotation = inertiaRotation;

            foreach (var w in spec.wings) AddWing(root, w);
            root.AddComponent<SensorDelayBuffer>();

            foreach (var v in spec.visuals) AddVisual(root, v);
            foreach (var r in spec.rotors) AddRotor(root, r, spec.directory);
            foreach (var s in spec.controlSurfaces) AddSurface(root, s);

            var imu = new GameObject("IMU");
            imu.transform.SetParent(root.transform, false);
            imu.transform.localPosition = Frames.FrdToUnity(spec.imu.position);
            imu.AddComponent<ImuModel>();

            var gt = new GameObject("GroundTruth");
            gt.transform.SetParent(root.transform, false);
            gt.AddComponent<GroundTruthPublisher>().PublishRate = 50f;

            foreach (var c in spec.cameras) AddCamera(root, c);

            root.AddComponent<VehicleActuators>();
        }

        /// <summary>
        /// Height to place the datum at so the collider's lowest point starts 10 cm up, clear of the
        /// ground and the 4 cm runway strip; the vehicle drops onto it.
        /// </summary>
        public static float SpawnHeight(VehicleSpec spec)
        {
            Vector3 c = Frames.FrdToUnity(spec.collider.center), s = Abs(Frames.FrdToUnity(spec.collider.size));
            return -(c.y - s.y / 2f) + 0.1f;
        }

        static void AddWing(GameObject root, WingSpec s)
        {
            var w = root.AddComponent<Wing>();
            w.aeroCenter = Frames.FrdToUnity(s.aeroCenter);
            w.area = s.area; w.span = s.span; w.chord = s.chord;
            w.cl0 = s.cl0; w.clAlpha = s.clAlpha; w.stallAlphaDeg = s.stallAlphaDeg; w.clMaxPostStall = s.clMaxPostStall;
            w.cd0 = s.cd0; w.oswald = s.oswald; w.cyBeta = s.cyBeta;
            w.cmAlpha = s.cmAlpha; w.cnBeta = s.cnBeta; w.clBeta = s.clBeta;
            w.clp = s.clp; w.cmq = s.cmq; w.cnr = s.cnr;
        }

        static void AddRotor(GameObject root, RotorSpec s, string directory)
        {
            var go = new GameObject(s.name);
            go.transform.SetParent(root.transform, false);
            go.transform.localPosition = Frames.FrdToUnity(s.position);
            go.transform.localRotation = Quaternion.FromToRotation(Vector3.up, Frames.FrdToUnity(s.AxisFrd));

            var disc = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            disc.name = "Disc";
            Object.DestroyImmediate(disc.GetComponent<Collider>());
            disc.transform.SetParent(go.transform, false);
            disc.transform.localScale = new Vector3(s.propDiameter, 0.005f, s.propDiameter);

            var r = go.AddComponent<Rotor>();
            r.channel = s.channel;
            r.spinDirection = s.Spin;
            r.timeConstant = s.timeConstant;
            r.idleSpeed = s.idle;
            r.maxThrust = s.MaxThrust(directory);
            if (!string.IsNullOrEmpty(s.thrustTable))
            {
                r.tableThrottle = s.tableThrottle;
                r.tableThrust = s.tableThrust;
                r.tableTorque = s.tableTorque;
            }
            r.maxTorque = s.tableTorque != null ? Mathf.Max(s.tableTorque) : r.maxThrust * s.momentRatio;
        }

        static void AddSurface(GameObject root, SurfaceSpec s)
        {
            var go = new GameObject(s.name);
            go.transform.SetParent(root.transform, false);
            var cs = go.AddComponent<ControlSurface>();
            cs.channel = s.channel;
            cs.torque = VehicleSpec.ToVector(s.torque);
            cs.momentCoefficient = s.momentCoefficient;
            cs.timeConstant = s.timeConstant;
        }

        static void AddCamera(GameObject root, CameraMountSpec s)
        {
            var go = new GameObject(s.name);
            go.transform.SetParent(root.transform, false);
            go.transform.localPosition = Frames.FrdToUnity(s.position);
            go.transform.localRotation = Quaternion.Euler(s.pitchDownDeg, 0f, 0f);
            var cam = go.AddComponent<Camera>();
            cam.nearClipPlane = 0.1f;
            cam.farClipPlane = 5000f;
            go.AddComponent<CameraPublisher>();
        }

        static void AddVisual(GameObject root, VisualSpec v)
        {
            var type = v.shape == "box" ? PrimitiveType.Cube : v.shape == "capsule" ? PrimitiveType.Capsule : PrimitiveType.Cylinder;
            var go = GameObject.CreatePrimitive(type);
            go.name = string.IsNullOrEmpty(v.name) ? v.shape : v.name;
            Object.DestroyImmediate(go.GetComponent<Collider>());
            go.transform.SetParent(root.transform, false);
            go.transform.localPosition = Frames.FrdToUnity(v.position);
            if (v.shape == "box")
            {
                go.transform.localRotation = Quaternion.Euler(0f, v.yawDeg, 0f);
                go.transform.localScale = Abs(Frames.FrdToUnity(v.size));
            }
            else
            {
                // Unity's capsule and cylinder run along their local y, 2 units long.
                go.transform.localRotation = v.axis == "x" ? Quaternion.Euler(90f, 0f, 0f)
                    : v.axis == "y" ? Quaternion.Euler(0f, 0f, 90f) : Quaternion.identity;
                go.transform.localScale = new Vector3(v.diameter, v.length / 2f, v.diameter);
            }
        }

        static Vector3 Abs(Vector3 v) => new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
    }
}
