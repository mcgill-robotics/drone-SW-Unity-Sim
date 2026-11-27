using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
using CesiumForUnity;
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
        const string HelsinkiTilesetUrl = "https://kartta.hel.fi/3d/mesh/Helsinki_2024/tileset.json";
        const double MalmiLatitude = 60.25389;
        const double MalmiLongitude = 25.04417;
        const double MalmiElevation = 17.0;

        [MenuItem("Drone Sim/Build Flight Scene")]
        public static void Build()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

            // Helsinki 3D mesh, georeferenced to Helsinki-Malmi Airport.
            var georeferenceObject = new GameObject("CesiumGeoreference");
            var georeference = georeferenceObject.AddComponent<CesiumGeoreference>();
            georeference.SetOriginLongitudeLatitudeHeight(MalmiLongitude, MalmiLatitude, MalmiElevation);

            var tilesetObject = new GameObject("Helsinki 3D Mesh");
            tilesetObject.transform.SetParent(georeferenceObject.transform, false);
            var tileset = tilesetObject.AddComponent<Cesium3DTileset>();
            tileset.tilesetSource = CesiumDataSource.FromUrl;
            tileset.url = HelsinkiTilesetUrl;
            // Generate colliders for streamed terrain and buildings so low-altitude
            // obstacle collision and raycasts can interact with the world.
            tileset.createPhysicsMeshes = true;
            tileset.updateInEditor = false;
            tileset.enableFogCulling = false;
            tileset.maximumScreenSpaceError = 8f;
            tileset.maximumCachedBytes = 1L * 1024L * 1024L * 1024L;
            tileset.maximumSimultaneousTileLoads = 24;
            tileset.loadingDescendantLimit = 20;
            tileset.enableFrustumCulling = true;
            tileset.enforceCulledScreenSpaceError = true;
            tileset.culledScreenSpaceError = 64f;
            tileset.showCreditsOnScreen = true;

            // Helsinki-Malmi Airport runway 18/36. The runway remains a simple, stable
            // physics surface while photogrammetry tiles stream in around it.
            var runway = GameObject.CreatePrimitive(PrimitiveType.Cube);
            runway.name = "Runway";
            runway.transform.position = new Vector3(0f, 0.02f, 0f);
            runway.transform.rotation = Quaternion.Euler(0f, 186f, 0f);
            runway.transform.localScale = new Vector3(30f, 0.04f, 1280f);
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
                var camera = cam.GetComponent<Camera>();
                camera.depth = 10;
                camera.farClipPlane = 10000f;
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
