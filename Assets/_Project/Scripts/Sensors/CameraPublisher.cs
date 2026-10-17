using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

namespace DroneSim
{
    /// <summary>
    /// Publishes a Unity camera as sensor_msgs/Image (or CompressedImage) plus CameraInfo.
    /// Topic, frame id, resolution, rate, field of view and format come from the sim config entry
    /// with this GameObject's name (SimConfig); without one the camera stays off.
    /// The camera is disabled and rendered only when a publish is due and the ROS endpoint is
    /// connected, so it costs nothing otherwise. Async GPU readback so the render thread is never
    /// stalled.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public class CameraPublisher : ROSPublisher
    {
        public override string Topic => config?.topic;

        private CameraConfig config;
        private Camera cam;
        private RenderTexture rt;
        private Texture2D encodeTex;
        private byte[] rawBuffer;
        private ImageMsg imageMsg;
        private CompressedImageMsg compressedMsg;
        private CameraInfoMsg infoMsg;
        private string compressedTopic, infoTopic;
        private bool reading;

        protected override void Start()
        {
            cam = GetComponent<Camera>();
            cam.enabled = false;

            config = SimConfig.Current.FindCamera(name);
            if (config == null) { Debug.LogWarning($"[CameraPublisher] no sim config entry for {name}, not publishing"); enabled = false; return; }
            string problem = config.Problem();
            if (problem != null) { Debug.LogError($"[CameraPublisher] sim config entry for {name}: {problem}, not publishing"); enabled = false; return; }
            cam.fieldOfView = config.verticalFov;
            PublishRate = config.rate;

            base.Start();
            useBaseRateLimiting = false;

            int width = config.width, height = config.height;
            string frameId = config.frameId;

            // Metal's default B8G8R8A8 target cannot be read back; pick an RGBA8 format that can.
            var fmt = GraphicsFormat.R8G8B8A8_SRGB;
            if (!SystemInfo.IsFormatSupported(fmt, GraphicsFormatUsage.ReadPixels)) fmt = GraphicsFormat.R8G8B8A8_UNorm;
            rt = new RenderTexture(width, height, 24, fmt);
            rt.Create();
            cam.targetTexture = rt;
            encodeTex = new Texture2D(width, height, TextureFormat.RGBA32, false);

            imageMsg = new ImageMsg { header = new HeaderMsg { frame_id = frameId }, encoding = "rgba8", width = (uint)width, height = (uint)height, step = (uint)(width * 4) };
            compressedMsg = new CompressedImageMsg { header = new HeaderMsg { frame_id = frameId }, format = "jpeg" };

            double f = (height / 2.0) / Mathf.Tan(cam.fieldOfView * 0.5f * Mathf.Deg2Rad);
            double cx = width / 2.0, cy = height / 2.0;
            infoMsg = new CameraInfoMsg
            {
                header = new HeaderMsg { frame_id = frameId },
                width = (uint)width, height = (uint)height,
                distortion_model = "plumb_bob",
                D = new double[5],
                K = new[] { f, 0, cx, 0, f, cy, 0, 0, 1 },
                R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                P = new[] { f, 0, cx, 0, 0, f, cy, 0, 0, 0, 1, 0 }
            };
        }

        protected override void RegisterPublisher()
        {
            string topic = config.topic;
            compressedTopic = topic + "/compressed";
            if (config.Jpeg) ros.RegisterPublisher<CompressedImageMsg>(compressedTopic);
            else ros.RegisterPublisher<ImageMsg>(topic);
            int i = topic.LastIndexOf('/');
            infoTopic = (i >= 0 ? topic.Substring(0, i + 1) : "") + "camera_info";
            ros.RegisterPublisher<CameraInfoMsg>(infoTopic);
        }

        protected override void FixedUpdate()
        {
            if (!SimSettings.PublishROS || reading) return;
            timeSinceLastPublish += Time.fixedDeltaTime;
            if (timeSinceLastPublish < timeBetweenPublishes) return;
            timeSinceLastPublish = 0f;
            if (rt == null || !rt.IsCreated() || ros.HasConnectionError) return;

            cam.Render();
            var stamp = ROSClock.GetROSTimestamp();
            reading = true;
            AsyncGPUReadback.Request(rt, 0, TextureFormat.RGBA32, req => OnReadback(req, stamp));
        }

        private void OnReadback(AsyncGPUReadbackRequest req, TimeMsg stamp)
        {
            reading = false;
            if (req.hasError) { Debug.LogWarning($"[CameraPublisher] readback failed on {config.topic}"); return; }
            var data = req.GetData<byte>();

            if (config.Jpeg)
            {
                if (rawBuffer == null || rawBuffer.Length != data.Length) rawBuffer = new byte[data.Length];
                data.CopyTo(rawBuffer);
                encodeTex.LoadRawTextureData(rawBuffer);
                encodeTex.Apply();
                compressedMsg.data = encodeTex.EncodeToJPG(config.jpegQuality);
                compressedMsg.header.stamp = stamp;
                ros.Publish(compressedTopic, compressedMsg);
            }
            else
            {
                if (imageMsg.data == null || imageMsg.data.Length != data.Length) imageMsg.data = new byte[data.Length];
                data.CopyTo(imageMsg.data);
                imageMsg.header.stamp = stamp;
                ros.Publish(config.topic, imageMsg);
            }
            infoMsg.header.stamp = stamp;
            ros.Publish(infoTopic, infoMsg);
        }

        public override void PublishMessage() { }

        private void OnDestroy()
        {
            if (rt != null) { rt.Release(); Destroy(rt); }
            if (encodeTex != null) Destroy(encodeTex);
        }
    }
}
