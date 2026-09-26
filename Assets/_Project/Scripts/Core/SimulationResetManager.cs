using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

namespace DroneSim
{
    /// <summary>
    /// Snaps the vehicle back to its start pose on demand, from ROS (/simulation/reset) or the R key.
    /// Optionally randomizes mass, thrust and wind so tuning runs see a spread of plants.
    /// Slimmed down from auv-sim-unity.
    /// </summary>
    [DefaultExecutionOrder(-50)]
    public class SimulationResetManager : MonoBehaviour
    {
        [Header("Randomization on reset")]
        public bool randomize = false;
        [Range(0f, 0.3f)] public float massSpread = 0.05f;
        [Range(0f, 0.3f)] public float thrustSpread = 0.05f;
        [Range(0f, 10f)] public float windSpeedMax = 3f;

        private Rigidbody rb;
        private Vector3 startPos;
        private Quaternion startRot;
        private float baseMass;
        private volatile bool resetRequested;

        private void Start()
        {
            rb = SimSettings.Vehicle;
            if (rb == null) { Debug.LogError("[Reset] No vehicle assigned in SimSettings"); return; }
            startPos = rb.position;
            startRot = rb.rotation;
            baseMass = rb.mass;

            if (ROSSettings.Instance != null)
            {
                ROSConnection.GetOrCreateInstance().Subscribe<BoolMsg>(
                    ROSSettings.Instance.SimulationResetTopic, m => { if (m.data) resetRequested = true; });
            }
        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.R)) resetRequested = true;
        }

        private void FixedUpdate()
        {
            if (!resetRequested) return;
            resetRequested = false;
            DoReset();
        }

        public void RequestReset() => resetRequested = true;

        private void DoReset()
        {
            rb.position = startPos;
            rb.rotation = startRot;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            if (randomize)
            {
                rb.mass = baseMass * (1f + Random.Range(-massSpread, massSpread));
                foreach (var r in rb.GetComponentsInChildren<Rotor>()) r.efficiency = 1f + Random.Range(-thrustSpread, thrustSpread);
                if (Wind.Instance != null)
                {
                    float a = Random.Range(0f, 360f) * Mathf.Deg2Rad;
                    Wind.Instance.meanWind = new Vector3(Mathf.Sin(a), 0f, Mathf.Cos(a)) * Random.Range(0f, windSpeedMax);
                }
            }

            foreach (var r in rb.GetComponentsInChildren<IResettable>()) r.ResetState();
            Debug.Log("[Reset] Vehicle reset to start pose");
        }
    }

    /// <summary>Components with internal state that must be cleared on teleport implement this.</summary>
    public interface IResettable
    {
        void ResetState();
    }
}
