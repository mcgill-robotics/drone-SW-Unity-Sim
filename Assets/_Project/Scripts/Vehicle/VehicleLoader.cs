using System;
using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Builds the vehicle from its spec when the scene starts, before any other component looks
    /// for it. The spec is --vehicle if given (see CommandLineOptions), else the vehicle field: a
    /// name in StreamingAssets/vehicles or a path to a .json file.
    /// </summary>
    [DefaultExecutionOrder(-200)]
    public class VehicleLoader : MonoBehaviour
    {
        [Tooltip("Spec name in StreamingAssets/vehicles (x500, quadplane) or a path to a .json file")]
        public string vehicle = "x500";

        public static VehicleSpec Loaded { get; private set; }

        private void Awake()
        {
            Loaded = null;
            string path = VehicleSpec.Resolve(CommandLineOptions.VehicleName ?? vehicle);
            VehicleSpec spec;
            try { spec = VehicleSpec.Load(path); }
            catch (Exception e) { Debug.LogError($"[Vehicle] can't read {path}: {e.Message}"); return; }

            var problems = spec.Problems();
            if (problems.Count > 0)
            {
                Debug.LogError($"[Vehicle] {path} has problems, no vehicle built:\n  " + string.Join("\n  ", problems));
                return;
            }

            var root = VehicleBuilder.Build(spec, new Vector3(0f, VehicleBuilder.SpawnHeight(spec), 0f));
            SimSettings.Instance.vehicle = root.GetComponent<Rigidbody>();
            Loaded = spec;

            Debug.Log($"[Vehicle] loaded {path}\n{spec.Summary(out var warnings)}");
            foreach (var w in warnings) Debug.LogWarning($"[Vehicle] {spec.name}: {w}");
        }
    }
}
