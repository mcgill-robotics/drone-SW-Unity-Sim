using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// The handful of global switches the sim needs. Deliberately small; add fields only when
    /// more than one script needs them.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class SimSettings : MonoBehaviour
    {
        private static SimSettings _instance;
        public static SimSettings Instance { get => _instance != null ? _instance : _instance = FindAnyObjectByType<SimSettings>(); private set => _instance = value; }

        [Header("Vehicle")]
        [Tooltip("The rigidbody every sensor reads from")]
        public Rigidbody vehicle;

        [Header("ROS")]
        [Tooltip("Master switch for all ROS publishers")]
        public bool publishROS = true;

        public static bool PublishROS => Instance != null && Instance.publishROS;
        public static Rigidbody Vehicle => Instance != null ? Instance.vehicle : null;

        private void Awake() => Instance = this;
    }
}
