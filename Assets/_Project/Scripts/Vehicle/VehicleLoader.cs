using System;
using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Builds the vehicle from its spec when the scene starts, before any other component looks
    /// for it. The spec is Override if set, else --vehicle if given (see CommandLineOptions), else
    /// the vehicle field: a name in StreamingAssets/vehicles or a path to a .json file.
    /// </summary>
    [DefaultExecutionOrder(-200)]
    public class VehicleLoader : MonoBehaviour
    {
        [Tooltip("Spec name in StreamingAssets/vehicles (x500, quadplane) or a path to a .json file")]
        public string vehicle = "x500";

        /// <summary>
        /// Vehicle for scenes loaded from now on, ahead of --vehicle and the field. For play-mode
        /// tests, which can't change the command line or reach the field before Awake.
        /// </summary>
        public static string Override { get; set; }

        public static VehicleSpec Loaded { get; private set; }

        // Domain reload is off in this project, so a leftover override would outlive its test.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ClearOverride() => Override = null;

        private void Awake()
        {
            Loaded = null;
            string path = VehicleSpec.Resolve(Override ?? CommandLineOptions.VehicleName ?? vehicle);
            VehicleSpec spec;
            try { spec = VehicleSpec.Load(path); }
            catch (Exception e) { Debug.LogError($"[Vehicle] can't read {path}: {e.Message}"); return; }

            var problems = spec.Problems();
            if (problems.Count > 0)
            {
                Debug.LogError($"[Vehicle] {path} has problems, no vehicle built:\n  " + string.Join("\n  ", problems));
                return;
            }

            GameObject root;
            try { root = VehicleBuilder.Build(spec, new Vector3(0f, VehicleBuilder.SpawnHeight(spec), 0f)); }
            catch (Exception e) { Debug.LogError($"[Vehicle] building {path} failed, no vehicle built: {e}"); return; }

            if (SimSettings.Instance != null) SimSettings.Instance.vehicle = root.GetComponent<Rigidbody>();
            else Debug.LogError("[Vehicle] no SimSettings in the scene, so sensors and the PX4 bridge can't find the vehicle");
            Loaded = spec;

            Debug.Log($"[Vehicle] loaded {path}\n{spec.Summary(out var warnings)}");
            foreach (var w in warnings) Debug.LogWarning($"[Vehicle] {spec.name}: {w}");
        }
    }
}
