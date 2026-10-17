using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Names of the sim-only ROS topics and frame ids. Camera topics and frames are in the sim
    /// config (SimConfig), since they have to match the real drone's camera drivers.
    ///
    /// Frames: body data is FLU (+X forward, +Y left, +Z up), world data is ENU. Right-hand rule.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class ROSSettings : MonoBehaviour
    {
        private static ROSSettings _instance;
        public static ROSSettings Instance { get => _instance != null ? _instance : _instance = FindAnyObjectByType<ROSSettings>(); private set => _instance = value; }

        [Header("Commands (ROS -> Sim)")]
        public string SimulationResetTopic = "/simulation/reset";

        [Header("Clock (Sim -> ROS)")]
        public string ClockTopic = "/clock";

        [Header("Ground truth (Sim -> ROS)")]
        public string GroundTruthPoseTopic = "/sim/ground_truth/pose";
        public string GroundTruthTwistTopic = "/sim/ground_truth/twist";
        public string GroundTruthGpsTopic = "/sim/ground_truth/gps";

        [Header("Frames")]
        public string WorldFrameId = "map";
        public string BaseLinkFrameId = "base_link";

        private void Awake() => Instance = this;
    }
}
