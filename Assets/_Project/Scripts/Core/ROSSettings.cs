using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Registry of ROS topic names and frame ids. Everything the ROS 2 stack sees from Unity
    /// is named here and nowhere else.
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

        [Header("Sensors (Sim -> ROS)")]
        public string ClockTopic = "/clock";
        public string IMUTopic = "/sim/imu/data";
        public string ForwardCameraTopic = "/sim/forward_cam/image_raw";
        public string DownCameraTopic = "/sim/down_cam/image_raw";

        [Header("Ground truth (Sim -> ROS)")]
        public string GroundTruthPoseTopic = "/sim/ground_truth/pose";
        public string GroundTruthTwistTopic = "/sim/ground_truth/twist";
        public string GroundTruthGpsTopic = "/sim/ground_truth/gps";

        [Header("Frames")]
        public string WorldFrameId = "map";
        public string BaseLinkFrameId = "base_link";
        public string ImuFrameId = "imu_link";
        public string ForwardCamFrameId = "forward_camera_optical_frame";
        public string DownCamFrameId = "down_camera_optical_frame";

        private void Awake() => Instance = this;
    }
}
