using System;
using System.IO;
using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Settings read from a JSON file at startup, so a built player can be changed without
    /// rebuilding. Holds what ROS consumers depend on: the cameras and the lidars.
    ///
    /// Default file: StreamingAssets/sim_config.json. In the editor that is
    /// Assets/StreamingAssets; in a build it is drone-sim_Data/StreamingAssets on Windows and
    /// Linux and drone-sim.app/Contents/Resources/Data/StreamingAssets on macOS. --config PATH
    /// picks another file (see CommandLineOptions). Read once per play session.
    /// </summary>
    [Serializable]
    public class SimConfig
    {
        public CameraConfig[] cameras = new CameraConfig[0];
        public LidarConfig[] lidars = new LidarConfig[0];

        private static SimConfig _current;

        public static SimConfig Current => _current ??= Load(CommandLineOptions.ConfigPath ?? DefaultPath);

        public static string DefaultPath => Path.Combine(Application.streamingAssetsPath, "sim_config.json");

        // Domain reload is off in this project, so statics survive between play sessions.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ClearCache() => _current = null;

        /// <summary>Reads a config file. Logs an error and returns an empty config if it can't.</summary>
        public static SimConfig Load(string path)
        {
            try
            {
                var config = Parse(File.ReadAllText(path));
                Debug.Log($"[SimConfig] loaded {path}");
                return config;
            }
            catch (Exception e)
            {
                Debug.LogError($"[SimConfig] can't read {path}: {e.Message}");
                return new SimConfig();
            }
        }

        public static SimConfig Parse(string json)
        {
            var config = new SimConfig();
            JsonUtility.FromJsonOverwrite(json, config);
            return config;
        }

        /// <summary>The entry for the camera GameObject with this name, or null.</summary>
        public CameraConfig FindCamera(string name) => Array.Find(cameras, c => c != null && c.name == name);

        /// <summary>The entry for the lidar GameObject with this name, or null.</summary>
        public LidarConfig FindLidar(string name) => Array.Find(lidars, l => l != null && l.name == name);
    }

    /// <summary>
    /// One onboard Velodyne VLP-16, as ROS sees it: a sensor_msgs/PointCloud2 on topic, one message
    /// per revolution. The defaults are the sensor's own (10 Hz spin, 100 m).
    /// </summary>
    [Serializable]
    public class LidarConfig
    {
        public string name;                     // matches the lidar mount's name in the vehicle spec
        public string topic;                    // PointCloud2 topic, /velodyne_points in the shipped config
        public string frameId;                  // header frame_id of the cloud, velodyne in the shipped config
        public float rotationHz = 10f;          // spin rate, 5 to 20 Hz on the real sensor; also the publish rate
        public float minRange = 0.5f;           // metres; nearer hits give no point
        public float maxRange = 100f;           // metres; the VLP-16's rated range
        public float rangeNoiseStdDev = 0.015f; // metres, Gaussian; the datasheet gives +-3 cm accuracy

        /// <summary>Why this entry can't be used, or null if it can.</summary>
        public string Problem()
        {
            if (string.IsNullOrEmpty(topic)) return "topic is empty";
            if (string.IsNullOrEmpty(frameId)) return "frameId is empty";
            if (rotationHz < Vlp16Model.MinRotationHz || rotationHz > Vlp16Model.MaxRotationHz)
                return $"rotationHz must be {Vlp16Model.MinRotationHz} to {Vlp16Model.MaxRotationHz}, got {rotationHz}";
            if (minRange < 0f) return $"bad minRange {minRange}";
            if (maxRange <= minRange) return $"maxRange {maxRange} must be above minRange {minRange}";
            if (rangeNoiseStdDev < 0f) return $"bad rangeNoiseStdDev {rangeNoiseStdDev}";
            return null;
        }
    }

    /// <summary>
    /// One onboard camera, as ROS sees it. Images go on topic (raw) or topic/compressed (jpeg),
    /// CameraInfo on camera_info next to it. Intrinsics follow from the resolution and vertical
    /// field of view, with no distortion.
    /// </summary>
    [Serializable]
    public class CameraConfig
    {
        public string name;
        public string topic;
        public string frameId;
        public int width;
        public int height;
        public float rate;
        public float verticalFov;
        public string format;
        public int jpegQuality = 75;

        public bool Jpeg => format == "jpeg";

        /// <summary>Why this entry can't be used, or null if it can.</summary>
        public string Problem()
        {
            if (string.IsNullOrEmpty(topic)) return "topic is empty";
            if (string.IsNullOrEmpty(frameId)) return "frameId is empty";
            if (width <= 0 || height <= 0) return $"bad resolution {width}x{height}";
            if (rate <= 0f) return $"bad rate {rate}";
            if (verticalFov <= 0f || verticalFov >= 180f) return $"bad verticalFov {verticalFov}";
            if (format != "jpeg" && format != "raw") return $"format must be jpeg or raw, got {format}";
            if (Jpeg && (jpegQuality < 1 || jpegQuality > 100)) return $"jpegQuality must be 1 to 100, got {jpegQuality}";
            return null;
        }
    }
}
