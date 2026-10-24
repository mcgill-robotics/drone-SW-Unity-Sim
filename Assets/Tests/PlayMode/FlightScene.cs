using System;
using System.Collections;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace DroneSim.Tests
{
    /// <summary>
    /// Loads Flight.unity the way a player started with --no-ros does, with the vehicle the test
    /// picks and the PX4 bridge on a free port, so a sim or PX4 already using 4560 doesn't interfere.
    /// </summary>
    public static class FlightScene
    {
        public const string Scene = "Flight";

        /// <summary>Every shipped vehicle spec, so a new spec is tested without touching the tests.</summary>
        public static string[] Vehicles => Directory
            .GetFiles(Path.Combine(Application.streamingAssetsPath, "vehicles"), "*.json")
            .Select(Path.GetFileNameWithoutExtension)
            .OrderBy(n => n, StringComparer.Ordinal)
            .ToArray();

        public static Px4HilBridge Bridge { get; private set; }
        public static Rigidbody Vehicle => SimSettings.Vehicle;

        /// <summary>Loads the scene and waits for its first physics step, so every Start has run.</summary>
        public static IEnumerator Load(string vehicle)
        {
            VehicleLoader.Override = vehicle;
            Bridge = null;
            SceneManager.sceneLoaded += Prepare;
            try { yield return SceneManager.LoadSceneAsync(Scene, LoadSceneMode.Single); }
            finally { SceneManager.sceneLoaded -= Prepare; }
            yield return new WaitForFixedUpdate();
        }

        /// <summary>Waits this many physics steps (4 ms each).</summary>
        public static IEnumerator Steps(int count)
        {
            for (int i = 0; i < count; i++) yield return new WaitForFixedUpdate();
        }

        // Runs after Awake, so the vehicle exists, and before Start, like CommandLineOptions.
        static void Prepare(Scene scene, LoadSceneMode mode)
        {
            var ros = UnityEngine.Object.FindAnyObjectByType<ROSConnection>();
            if (ros != null) ros.ConnectOnStart = false;
            if (SimSettings.Instance != null) SimSettings.Instance.publishROS = false;
            Bridge = UnityEngine.Object.FindAnyObjectByType<Px4HilBridge>();
            if (Bridge != null) Bridge.port = FreePort();
        }

        static int FreePort()
        {
            var listener = new TcpListener(IPAddress.Loopback, 0);
            listener.Start();
            int port = ((IPEndPoint)listener.LocalEndpoint).Port;
            listener.Stop();
            return port;
        }
    }
}
