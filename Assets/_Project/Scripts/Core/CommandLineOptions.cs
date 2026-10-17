using System;
using System.Globalization;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;

namespace DroneSim
{
    /// <summary>
    /// Command-line options for standalone builds. Applied to each scene after Awake and before
    /// Start, so they take effect before PX4 or ROS connect.
    ///
    ///   --scene NAME      Load this scene instead of the first one in the build (FlightX500, Flight)
    ///   --px4-port PORT   TCP port PX4 SITL connects to (default 4560)
    ///   --ros-ip IP       ROS TCP endpoint address (default 127.0.0.1)
    ///   --ros-port PORT   ROS TCP endpoint port (default 10000)
    ///   --no-ros          Don't connect or publish to ROS
    ///
    /// The home location comes from PX4_HOME_LAT, PX4_HOME_LON and PX4_HOME_ALT (see GeoOrigin).
    /// </summary>
    public static class CommandLineOptions
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Init()
        {
            SceneManager.sceneLoaded -= OnSceneLoaded;
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            string wanted = Value("--scene");
            if (wanted != null && wanted != scene.name)
            {
                if (Application.CanStreamedLevelBeLoaded(wanted))
                {
                    // Keep this scene's components from starting (opening ports) before it unloads.
                    foreach (var root in scene.GetRootGameObjects()) root.SetActive(false);
                    SceneManager.LoadScene(wanted);
                    return;
                }
                Debug.LogError($"[CommandLine] No scene named {wanted} in this build");
            }

            var bridge = UnityEngine.Object.FindAnyObjectByType<Px4HilBridge>();
            if (bridge != null && Int("--px4-port") is int px4Port) bridge.port = px4Port;

            var ros = UnityEngine.Object.FindAnyObjectByType<ROSConnection>();
            if (ros != null)
            {
                if (Value("--ros-ip") is string ip) ros.RosIPAddress = ip;
                if (Int("--ros-port") is int rosPort) ros.RosPort = rosPort;
                if (Flag("--no-ros")) ros.ConnectOnStart = false;
            }
            if (Flag("--no-ros") && SimSettings.Instance != null) SimSettings.Instance.publishROS = false;
        }

        private static bool Flag(string name) => Array.IndexOf(Environment.GetCommandLineArgs(), name) >= 0;

        private static string Value(string name)
        {
            string[] args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, name);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
        }

        private static int? Int(string name)
        {
            string v = Value(name);
            if (v == null) return null;
            if (int.TryParse(v, NumberStyles.Integer, CultureInfo.InvariantCulture, out int n)) return n;
            Debug.LogError($"[CommandLine] {name} expects a number, got {v}");
            return null;
        }
    }
}
