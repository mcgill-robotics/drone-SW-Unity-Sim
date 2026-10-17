using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
using DroneSim;

namespace DroneSim.Editor
{
    /// <summary>
    /// Builds the flight scene from code so the whole setup is reviewable and reproducible.
    /// Menu: Drone Sim > Build Flight Scene. Also runnable headless via -executeMethod.
    ///
    /// The scene holds the world and the managers. The vehicle is not in it: VehicleLoader builds
    /// it at startup from a spec in StreamingAssets/vehicles.
    /// </summary>
    public static class SceneBuilder
    {
        const string ScenePath = "Assets/_Project/Scenes/Flight.unity";

        [MenuItem("Drone Sim/Build Flight Scene")]
        public static void Build()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

            // Ground: a flat 4 km plane in plain green.
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
            managers.AddComponent<SimSettings>();
            managers.AddComponent<VehicleLoader>();
            managers.AddComponent<ROSClock>();
            managers.AddComponent<SimulationResetManager>();
            managers.AddComponent<GeoOrigin>();
            managers.AddComponent<Wind>();
            managers.AddComponent<ManualControl>();
            var ros = managers.AddComponent<ROSConnection>();
            ros.RosIPAddress = "127.0.0.1";
            ros.RosPort = 10000;
            ros.ConnectOnStart = true;

            managers.AddComponent<Px4HilBridge>();

            // User camera.
            var cam = GameObject.Find("Main Camera");
            if (cam != null)
            {
                cam.transform.position = new Vector3(-6f, 3f, -8f);
                // Draw after the onboard cameras so the Game view always shows the chase view.
                cam.GetComponent<Camera>().depth = 10;
                var orbit = cam.AddComponent<OrbitFollowCamera>();
                orbit.distance = 10f;
            }

            System.IO.Directory.CreateDirectory("Assets/_Project/Scenes");
            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();
            Debug.Log($"[SceneBuilder] Saved {ScenePath}");
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
    }
}
