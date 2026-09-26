using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
using Bridge;

namespace DroneSim.Editor
{
    /// <summary>
    /// Builds the flight scenes from code so the whole setup is reviewable and reproducible.
    /// Menu: Drone Sim > Build Flight Scenes. Also runnable headless via -executeMethod.
    ///
    /// Flight.unity flies the quadplane (PX4 airframe gazebo-classic_standard_vtol).
    /// FlightX500.unity flies PX4's X500 quad (PX4 airframe none_iris), the vehicle drone-2026
    /// and drone-2027 tested against in Gazebo.
    /// </summary>
    public static class SceneBuilder
    {
        const string QuadplaneScenePath = "Assets/_Project/Scenes/Flight.unity";
        const string X500ScenePath = "Assets/_Project/Scenes/FlightX500.unity";
        /// <summary>Lift and pusher props are 16 inch.</summary>
        const float PropDiameter = 16f * 0.0254f;
        /// <summary>Expected takeoff weight is 6 to 7 kg.</summary>
        const float QuadplaneMass = 6.5f;
        /// <summary>
        /// Per lift rotor, for a hover thrust-to-weight of about 2. A guess for 16 inch props until
        /// the motors and battery are picked; replace with bench-test numbers.
        /// </summary>
        const float LiftRotorMaxThrust = 32f;
        /// <summary>Reaction torque per newton of thrust, the ratio PX4's Gazebo models use.</summary>
        const float RotorMomentRatio = 0.016f;

        [MenuItem("Drone Sim/Build Flight Scenes")]
        public static void Build()
        {
            BuildQuadplaneScene();
            BuildX500Scene();
        }

        [MenuItem("Drone Sim/Build Quadplane Scene")]
        public static void BuildQuadplaneScene() => BuildScene(QuadplaneScenePath, BuildQuadplane, spawnHeight: 0.3f);

        [MenuItem("Drone Sim/Build X500 Scene")]
        public static void BuildX500Scene() => BuildScene(X500ScenePath, BuildX500, spawnHeight: 0.25f);

        static void BuildScene(string path, System.Func<GameObject> buildVehicle, float spawnHeight)
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

            // Ground: a 4 km plane with a checker-ish material so motion is visible from altitude.
            var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
            ground.name = "Ground";
            ground.transform.localScale = new Vector3(400f, 1f, 400f);
            ground.isStatic = true;
            ground.GetComponent<Renderer>().sharedMaterial =
                GetOrCreateMaterial("Assets/_Project/Materials/Ground.mat", new Color(0.35f, 0.45f, 0.25f));

            // Runway strip for a visual reference.
            var runway = GameObject.CreatePrimitive(PrimitiveType.Cube);
            runway.name = "Runway";
            runway.transform.position = new Vector3(0f, 0.02f, 60f);
            runway.transform.localScale = new Vector3(12f, 0.04f, 150f);
            runway.isStatic = true;
            runway.GetComponent<Renderer>().sharedMaterial =
                GetOrCreateMaterial("Assets/_Project/Materials/Runway.mat", new Color(0.3f, 0.3f, 0.32f));

            // Sun.
            var light = GameObject.Find("Directional Light");
            if (light != null) light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);

            // Managers.
            var managers = new GameObject("Managers");
            managers.AddComponent<ROSSettings>();
            var settings = managers.AddComponent<SimSettings>();
            managers.AddComponent<ROSClock>();
            managers.AddComponent<SimulationResetManager>();
            managers.AddComponent<GeoOrigin>();
            managers.AddComponent<Wind>();
            managers.AddComponent<ManualControl>();
            var ros = managers.AddComponent<ROSConnection>();
            ros.RosIPAddress = "127.0.0.1";
            ros.RosPort = 10000;
            ros.ConnectOnStart = true;

            // Vehicle.
            var vehicle = buildVehicle();
            vehicle.transform.position = new Vector3(0f, spawnHeight, 0f);
            settings.vehicle = vehicle.GetComponent<Rigidbody>();

            var bridge = managers.AddComponent<Px4HilBridge>();
            bridge.imu = vehicle.GetComponentInChildren<ImuModel>();
            bridge.actuators = vehicle.GetComponent<VehicleActuators>();
            bridge.wing = vehicle.GetComponent<Wing>();

            // User camera.
            var cam = GameObject.Find("Main Camera");
            if (cam != null)
            {
                cam.transform.position = new Vector3(-6f, 3f, -8f);
                // Draw after the onboard cameras so the Game view shows the chase view. In play
                // mode the onboard cameras render to textures, but in edit mode they would win.
                cam.GetComponent<Camera>().depth = 10;
                var orbit = cam.AddComponent<OrbitFollowCamera>();
                orbit.distance = 10f;
            }

            System.IO.Directory.CreateDirectory("Assets/_Project/Scenes");
            EditorSceneManager.SaveScene(scene, path);
            AssetDatabase.SaveAssets();
            Debug.Log($"[SceneBuilder] Saved {path}");
        }

        /// <summary>Reuse the material if it exists so rebuilding doesn't rewrite the asset.</summary>
        static Material GetOrCreateMaterial(string path, Color color)
        {
            var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (mat != null) return mat;
            mat = new Material(Shader.Find("HDRP/Lit")) { color = color };
            AssetDatabase.CreateAsset(mat, path);
            return mat;
        }

        /// <summary>
        /// A placeholder standard VTOL: 2 m span wing, four lift rotors on booms, a rear pusher,
        /// two ailerons and an elevator. Channels follow PX4's standard_vtol airframe, see
        /// StandardVtolChannels. That airframe has no rudder; the wing's cnBeta gives weathercock
        /// stability instead.
        /// </summary>
        static GameObject BuildQuadplane()
        {
            var root = new GameObject("Vehicle");
            var rb = root.AddComponent<Rigidbody>();
            rb.mass = QuadplaneMass;
            rb.linearDamping = 0f;
            rb.angularDamping = 0.05f;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.inertiaTensor = new Vector3(0.6f, 0.9f, 0.4f) * (QuadplaneMass / 6f); // placeholder scaled from 6 kg
            rb.inertiaTensorRotation = Quaternion.identity;

            var col = root.AddComponent<BoxCollider>();
            col.size = new Vector3(2.0f, 0.15f, 1.2f);

            var wing = root.AddComponent<Wing>();
            root.AddComponent<VehicleActuators>();
            root.AddComponent<SensorDelayBuffer>();

            // Visual meshes.
            Mesh(root, "Fuselage", PrimitiveType.Capsule, new Vector3(0, 0, 0), Quaternion.Euler(90, 0, 0), new Vector3(0.15f, 0.55f, 0.15f));
            Mesh(root, "WingMesh", PrimitiveType.Cube, new Vector3(0, 0, 0.05f), Quaternion.identity, new Vector3(2.0f, 0.03f, 0.22f));
            Mesh(root, "Tail", PrimitiveType.Cube, new Vector3(0, 0.02f, -0.55f), Quaternion.identity, new Vector3(0.6f, 0.02f, 0.15f));
            Mesh(root, "Fin", PrimitiveType.Cube, new Vector3(0, 0.12f, -0.55f), Quaternion.identity, new Vector3(0.02f, 0.2f, 0.15f));
            Mesh(root, "BoomL", PrimitiveType.Cube, new Vector3(-0.5f, 0, 0), Quaternion.identity, new Vector3(0.03f, 0.03f, 0.9f));
            Mesh(root, "BoomR", PrimitiveType.Cube, new Vector3(0.5f, 0, 0), Quaternion.identity, new Vector3(0.03f, 0.03f, 0.9f));

            // Lift rotors, PX4 quad-x order: 0 front-right, 1 rear-left, 2 front-left, 3 rear-right.
            // Spin +1 = CCW from above, matching the sign of PX4's CA_ROTORn_KM for that rotor.
            AddRotor(root, "RotorFR", StandardVtolChannels.RotorFrontRight, new Vector3(0.5f, 0.03f, 0.4f), Vector3.up, +1);
            AddRotor(root, "RotorRL", StandardVtolChannels.RotorRearLeft, new Vector3(-0.5f, 0.03f, -0.4f), Vector3.up, +1);
            AddRotor(root, "RotorFL", StandardVtolChannels.RotorFrontLeft, new Vector3(-0.5f, 0.03f, 0.4f), Vector3.up, -1);
            AddRotor(root, "RotorRR", StandardVtolChannels.RotorRearRight, new Vector3(0.5f, 0.03f, -0.4f), Vector3.up, -1);
            foreach (var r in root.GetComponentsInChildren<Rotor>())
            {
                r.maxThrust = LiftRotorMaxThrust;
                r.maxTorque = LiftRotorMaxThrust * RotorMomentRatio;
            }
            var pusher = AddRotor(root, "Pusher", StandardVtolChannels.Pusher, new Vector3(0f, 0f, -0.62f), Vector3.forward, +1);
            pusher.maxThrust = 15f;
            pusher.maxTorque = 0.05f;

            // Each aileron carries half the roll authority, as PX4's CA_SV_CS0/1_TRQ_R = -0.5/+0.5.
            AddSurface(root, "AileronLeft", StandardVtolChannels.AileronLeft, ControlSurface.Axis.Roll, 0.015f, invert: true);
            AddSurface(root, "AileronRight", StandardVtolChannels.AileronRight, ControlSurface.Axis.Roll, 0.015f);
            AddSurface(root, "Elevator", StandardVtolChannels.Elevator, ControlSurface.Axis.Pitch, 0.02f);

            AddSensors(root, downCamera: new Vector3(0, -0.08f, 0.1f), forwardCamera: new Vector3(0, 0.05f, 0.35f));
            return root;
        }

        /// <summary>
        /// PX4's X500 quad, with the numbers from its Gazebo model (PX4-gazebo-models x500):
        /// 2.0 kg, motors on a 0.174 m square, 8.55 N max thrust each (motorConstant 8.54858e-6 at
        /// 1000 rad/s), moment 0.016 N m per N, idle at 15% speed (SIM_GZ_EC_MIN 150 of 1000).
        /// Motor order is PX4 quad-x, the same as the quadplane's lift rotors.
        /// </summary>
        static GameObject BuildX500()
        {
            const float maxThrust = 8.54858e-6f * 1000f * 1000f;
            const float propDiameter = 0.28f;

            var root = new GameObject("Vehicle");
            var rb = root.AddComponent<Rigidbody>();
            rb.mass = 2.0f;
            rb.linearDamping = 0f;
            rb.angularDamping = 0.05f;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.inertiaTensor = new Vector3(0.02167f, 0.04f, 0.02167f); // Unity x = pitch, y = yaw, z = roll
            rb.inertiaTensorRotation = Quaternion.identity;

            // Body plate down to the bottom of the landing skids.
            var col = root.AddComponent<BoxCollider>();
            col.center = new Vector3(0f, -0.1f, 0f);
            col.size = new Vector3(0.35f, 0.26f, 0.35f);

            root.AddComponent<VehicleActuators>();
            root.AddComponent<SensorDelayBuffer>();

            Mesh(root, "Body", PrimitiveType.Cube, new Vector3(0, 0.007f, 0), Quaternion.identity, new Vector3(0.2f, 0.05f, 0.2f));
            Mesh(root, "ArmA", PrimitiveType.Cube, new Vector3(0, 0.03f, 0), Quaternion.Euler(0, 45, 0), new Vector3(0.02f, 0.02f, 0.5f));
            Mesh(root, "ArmB", PrimitiveType.Cube, new Vector3(0, 0.03f, 0), Quaternion.Euler(0, -45, 0), new Vector3(0.02f, 0.02f, 0.5f));
            Mesh(root, "SkidL", PrimitiveType.Cube, new Vector3(-0.132f, -0.22f, 0), Quaternion.identity, new Vector3(0.015f, 0.015f, 0.25f));
            Mesh(root, "SkidR", PrimitiveType.Cube, new Vector3(0.132f, -0.22f, 0), Quaternion.identity, new Vector3(0.015f, 0.015f, 0.25f));

            AddRotor(root, "RotorFR", StandardVtolChannels.RotorFrontRight, new Vector3(0.174f, 0.06f, 0.174f), Vector3.up, +1, propDiameter);
            AddRotor(root, "RotorRL", StandardVtolChannels.RotorRearLeft, new Vector3(-0.174f, 0.06f, -0.174f), Vector3.up, +1, propDiameter);
            AddRotor(root, "RotorFL", StandardVtolChannels.RotorFrontLeft, new Vector3(-0.174f, 0.06f, 0.174f), Vector3.up, -1, propDiameter);
            AddRotor(root, "RotorRR", StandardVtolChannels.RotorRearRight, new Vector3(0.174f, 0.06f, -0.174f), Vector3.up, -1, propDiameter);
            foreach (var r in root.GetComponentsInChildren<Rotor>())
            {
                r.maxThrust = maxThrust;
                r.maxTorque = maxThrust * RotorMomentRatio;
                r.timeConstant = 0.02f; // Gazebo: 0.0125 s up, 0.025 s down
                r.idleSpeed = 0.15f;
            }

            // Forward camera under the body plate, tilted 20 degrees down so the props stay out of frame.
            AddSensors(root, downCamera: new Vector3(0, -0.05f, 0.05f), forwardCamera: new Vector3(0, -0.06f, 0.1f), forwardPitchDeg: 20f);
            return root;
        }

        /// <summary>IMU at the body origin, ground truth, and the down and forward cameras.</summary>
        static void AddSensors(GameObject root, Vector3 downCamera, Vector3 forwardCamera, float forwardPitchDeg = 0f)
        {
            var imuGo = new GameObject("IMU");
            imuGo.transform.SetParent(root.transform, false);
            imuGo.AddComponent<ImuModel>();
            imuGo.AddComponent<IMUPublisher>();

            var gt = new GameObject("GroundTruth");
            gt.transform.SetParent(root.transform, false);
            gt.AddComponent<GroundTruthPublisher>().PublishRate = 50f;

            AddCamera(root, "DownCamera", downCamera, Quaternion.Euler(90, 0, 0), "/sim/down_cam/image_raw", "down_camera_optical_frame", 60f);
            AddCamera(root, "ForwardCamera", forwardCamera, Quaternion.Euler(forwardPitchDeg, 0, 0), "/sim/forward_cam/image_raw", "forward_camera_optical_frame", 70f);
        }

        static void Mesh(GameObject parent, string name, PrimitiveType type, Vector3 pos, Quaternion rot, Vector3 scale)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            Object.DestroyImmediate(go.GetComponent<Collider>());
            go.transform.SetParent(parent.transform, false);
            go.transform.localPosition = pos;
            go.transform.localRotation = rot;
            go.transform.localScale = scale;
        }

        static Rotor AddRotor(GameObject parent, string name, int channel, Vector3 pos, Vector3 axis, int spin, float diameter = PropDiameter)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent.transform, false);
            go.transform.localPosition = pos;
            go.transform.localRotation = Quaternion.FromToRotation(Vector3.up, axis);
            var disc = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            disc.name = "Disc";
            Object.DestroyImmediate(disc.GetComponent<Collider>());
            disc.transform.SetParent(go.transform, false);
            disc.transform.localScale = new Vector3(diameter, 0.005f, diameter);
            var r = go.AddComponent<Rotor>();
            r.channel = channel;
            r.spinDirection = spin;
            return r;
        }

        static void AddSurface(GameObject parent, string name, int channel, ControlSurface.Axis axis, float coeff, bool invert = false)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent.transform, false);
            var s = go.AddComponent<ControlSurface>();
            s.channel = channel;
            s.axis = axis;
            s.momentCoefficient = coeff;
            s.invert = invert;
        }

        static void AddCamera(GameObject parent, string name, Vector3 pos, Quaternion rot, string topic, string frame, float fov)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent.transform, false);
            go.transform.localPosition = pos;
            go.transform.localRotation = rot;
            var cam = go.AddComponent<Camera>();
            cam.fieldOfView = fov;
            cam.nearClipPlane = 0.1f;
            cam.farClipPlane = 5000f;
            var pub = go.AddComponent<CameraPublisher>();
            pub.topic = topic;
            pub.frameId = frame;
            pub.width = 1280;
            pub.height = 720;
            pub.PublishRate = 10f;
        }
    }
}
