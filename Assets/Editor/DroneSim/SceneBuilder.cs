using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
using Bridge;

namespace DroneSim.Editor
{
    /// <summary>
    /// Builds the Flight scene from code so the whole setup is reviewable and reproducible.
    /// Menu: Drone Sim > Build Flight Scene. Also runnable headless via -executeMethod.
    /// </summary>
    public static class SceneBuilder
    {
        const string ScenePath = "Assets/_Project/Scenes/Flight.unity";

        [MenuItem("Drone Sim/Build Flight Scene")]
        public static void Build()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

            // Ground: a 4 km plane with a checker-ish material so motion is visible from altitude.
            var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
            ground.name = "Ground";
            ground.transform.localScale = new Vector3(400f, 1f, 400f);
            ground.isStatic = true;
            var groundMat = new Material(Shader.Find("HDRP/Lit"));
            groundMat.color = new Color(0.35f, 0.45f, 0.25f);
            AssetDatabase.CreateAsset(groundMat, "Assets/_Project/Materials/Ground.mat");
            ground.GetComponent<Renderer>().sharedMaterial = groundMat;

            // Runway strip for a visual reference.
            var runway = GameObject.CreatePrimitive(PrimitiveType.Cube);
            runway.name = "Runway";
            runway.transform.position = new Vector3(0f, 0.02f, 60f);
            runway.transform.localScale = new Vector3(12f, 0.04f, 150f);
            runway.isStatic = true;
            var runwayMat = new Material(Shader.Find("HDRP/Lit"));
            runwayMat.color = new Color(0.3f, 0.3f, 0.32f);
            AssetDatabase.CreateAsset(runwayMat, "Assets/_Project/Materials/Runway.mat");
            runway.GetComponent<Renderer>().sharedMaterial = runwayMat;

            // Sun.
            var light = GameObject.Find("Directional Light");
            if (light != null) light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);

            // Managers.
            var managers = new GameObject("Managers");
            managers.AddComponent<ROSSettings>();
            var settings = managers.AddComponent<SimSettings>();
            managers.AddComponent<ROSClock>();
            managers.AddComponent<SimulationResetManager>();
            var geo = managers.AddComponent<GeoOrigin>();
            managers.AddComponent<Wind>();
            managers.AddComponent<ManualControl>();
            var ros = managers.AddComponent<ROSConnection>();
            ros.RosIPAddress = "127.0.0.1";
            ros.RosPort = 10000;
            ros.ConnectOnStart = true;

            // Vehicle.
            var vehicle = BuildVehicle();
            vehicle.transform.position = new Vector3(0f, 0.3f, 0f);
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
                var orbit = cam.AddComponent<OrbitFollowCamera>();
                orbit.distance = 10f;
            }

            System.IO.Directory.CreateDirectory("Assets/_Project/Scenes");
            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();
            Debug.Log($"[SceneBuilder] Saved {ScenePath}");
        }

        /// <summary>
        /// A placeholder standard VTOL: 2 m span wing, four lift rotors on booms, a rear pusher,
        /// and three control surfaces. Channels follow PX4's standard_vtol SITL mapping:
        /// 0-3 lift rotors, 4 pusher, 5 aileron, 6 elevator, 7 rudder.
        /// </summary>
        static GameObject BuildVehicle()
        {
            var root = new GameObject("Vehicle");
            var rb = root.AddComponent<Rigidbody>();
            rb.mass = 6f;
            rb.linearDamping = 0f;
            rb.angularDamping = 0.05f;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.inertiaTensor = new Vector3(0.6f, 0.9f, 0.4f);
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
            AddRotor(root, "RotorFR", 0, new Vector3(0.5f, 0.03f, 0.4f), Vector3.up, +1);
            AddRotor(root, "RotorRL", 1, new Vector3(-0.5f, 0.03f, -0.4f), Vector3.up, +1);
            AddRotor(root, "RotorFL", 2, new Vector3(-0.5f, 0.03f, 0.4f), Vector3.up, -1);
            AddRotor(root, "RotorRR", 3, new Vector3(0.5f, 0.03f, -0.4f), Vector3.up, -1);
            var pusher = AddRotor(root, "Pusher", 4, new Vector3(0f, 0f, -0.62f), Vector3.forward, +1);
            pusher.maxThrust = 15f;
            pusher.maxTorque = 0.05f;

            AddSurface(root, "Aileron", 5, ControlSurface.Axis.Roll, 0.03f);
            AddSurface(root, "Elevator", 6, ControlSurface.Axis.Pitch, 0.02f);
            AddSurface(root, "Rudder", 7, ControlSurface.Axis.Yaw, 0.01f);

            // Sensors.
            var imuGo = new GameObject("IMU");
            imuGo.transform.SetParent(root.transform, false);
            imuGo.AddComponent<ImuModel>();
            imuGo.AddComponent<IMUPublisher>();

            var gt = new GameObject("GroundTruth");
            gt.transform.SetParent(root.transform, false);
            gt.AddComponent<GroundTruthPublisher>().PublishRate = 50f;

            AddCamera(root, "DownCamera", new Vector3(0, -0.08f, 0.1f), Quaternion.Euler(90, 0, 0), "/sim/down_cam/image_raw", "down_camera_optical_frame", 60f);
            AddCamera(root, "ForwardCamera", new Vector3(0, 0.05f, 0.35f), Quaternion.identity, "/sim/forward_cam/image_raw", "forward_camera_optical_frame", 70f);

            return root;
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

        static Rotor AddRotor(GameObject parent, string name, int channel, Vector3 pos, Vector3 axis, int spin)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent.transform, false);
            go.transform.localPosition = pos;
            go.transform.localRotation = Quaternion.FromToRotation(Vector3.up, axis);
            var disc = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            disc.name = "Disc";
            Object.DestroyImmediate(disc.GetComponent<Collider>());
            disc.transform.SetParent(go.transform, false);
            disc.transform.localScale = new Vector3(0.25f, 0.005f, 0.25f);
            var r = go.AddComponent<Rotor>();
            r.channel = channel;
            r.spinDirection = spin;
            return r;
        }

        static void AddSurface(GameObject parent, string name, int channel, ControlSurface.Axis axis, float coeff)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent.transform, false);
            var s = go.AddComponent<ControlSurface>();
            s.channel = channel;
            s.axis = axis;
            s.momentCoefficient = coeff;
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
