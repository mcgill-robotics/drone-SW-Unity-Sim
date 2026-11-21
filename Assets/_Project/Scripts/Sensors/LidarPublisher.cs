using System;
using Unity.Collections;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace DroneSim
{
    /// <summary>
    /// A simulated Velodyne VLP-16 published as sensor_msgs/PointCloud2, with the fields of the
    /// velodyne ROS 2 driver (see Vlp16Model). Topic, frame id, spin rate, range and noise come from
    /// the sim config entry with this GameObject's name (SimConfig); without one the lidar stays off.
    ///
    /// The sensor spins continuously, as the real one does. Every physics step casts the firing
    /// sequences that fall inside that step (about 72 of them, 14 degrees of the turn at 10 Hz) from
    /// the pose the lidar has at that step, and one cloud is published each time the turn completes.
    /// So a moving vehicle distorts the cloud the way it does on the real sensor: each point is in
    /// the sensor frame of the moment its laser fired, and its time field says when that was,
    /// counted from the start of the revolution. The header stamp is the start of the revolution.
    ///
    /// The rays are batched physics raycasts, so the lidar works without a GPU (-nographics) and
    /// sees colliders only. A laser whose first hit is the vehicle carrying the lidar, or anything
    /// nearer than the minimum range, gives no point: the real sensor reports nothing there and
    /// can't see past it either.
    ///
    /// With each cloud it publishes on /tf where the lidar sits on the vehicle: a transform from the
    /// vehicle's body frame (ROSSettings base_link, the frame odometry and the ground truth pose
    /// describe) to the cloud's frame. So code that has the vehicle's pose can put the points in the
    /// world: point in world = pose * mount * point.
    ///
    /// Like the cameras it only runs while the ROS endpoint is connected. In the editor it runs
    /// regardless, so the points can be checked in the Scene view with the lidar selected.
    /// </summary>
    public class LidarPublisher : ROSPublisher
    {
        // Unity has no reflectivity to measure, so every point gets the same intensity.
        const float Intensity = 100f;
        const string TfTopic = "/tf";

        public override string Topic => config?.topic;

        /// <summary>Points in the last completed revolution.</summary>
        public int LastPointCount { get; private set; }
        /// <summary>World positions of the last revolution's points; the first LastPointCount entries are valid.</summary>
        public Vector3[] LastWorldPoints { get; private set; } = new Vector3[0];
        /// <summary>Revolutions completed since the scene started.</summary>
        public int Revolutions { get; private set; }

        private static readonly PointFieldMsg[] Fields =
        {
            new PointFieldMsg { name = "x",         offset = Vlp16Model.XOffset,         datatype = 7, count = 1 }, // 7 = FLOAT32
            new PointFieldMsg { name = "y",         offset = Vlp16Model.YOffset,         datatype = 7, count = 1 },
            new PointFieldMsg { name = "z",         offset = Vlp16Model.ZOffset,         datatype = 7, count = 1 },
            new PointFieldMsg { name = "intensity", offset = Vlp16Model.IntensityOffset, datatype = 7, count = 1 },
            new PointFieldMsg { name = "ring",      offset = Vlp16Model.RingOffset,      datatype = 4, count = 1 }, // 4 = UINT16
            new PointFieldMsg { name = "time",      offset = Vlp16Model.TimeOffset,      datatype = 7, count = 1 },
        };

        private LidarConfig config;
        private Rigidbody selfBody;    // the vehicle carrying the lidar; its own colliders block lasers but give no points
        private int columns;           // firing sequences per revolution
        private Vector3[] localDirs;   // one per ray, column by column, ring 0 first: ray i is ring i % 16 of column i / 16
        // Raycast inputs and outputs, sized for a whole revolution once in Start and reused, so the
        // physics step allocates nothing.
        private NativeArray<RaycastCommand> commands;
        private NativeArray<RaycastHit> hits;

        // The revolution in progress.
        private int column;            // next firing sequence to cast
        private double pendingFirings; // firing sequences owed, with the fraction carried between steps
        private double scanStartTime;  // simulated time of the revolution's first firing
        private byte[] scanData;       // packed points so far
        private Vector3[] scanPoints;  // their world positions
        private int scanCount;

        protected override void Start()
        {
            // Settings come from sim_config.json, matched by this GameObject's name (the mount name in
            // the vehicle spec). A missing or bad entry turns the lidar off rather than guessing.
            config = SimConfig.Current.FindLidar(name);
            if (config == null) { Debug.LogWarning($"[LidarPublisher] no sim config entry for {name}, not publishing"); enabled = false; return; }
            string problem = config.Problem();
            if (problem != null) { Debug.LogError($"[LidarPublisher] sim config entry for {name}: {problem}, not publishing"); enabled = false; return; }
            // One cloud per revolution. The base class's rate limiting is turned off because the
            // timing comes from the spin below; PublishRate is set so the Inspector shows the real rate.
            PublishRate = config.rotationHz;

            base.Start();
            useBaseRateLimiting = false;

            selfBody = GetComponentInParent<Rigidbody>();

            // Every ray direction of one revolution, in the lidar's own axes, worked out once. Each
            // step only rotates them by the lidar's current attitude.
            columns = Vlp16Model.ColumnsPerRevolution(config.rotationHz);
            int rays = columns * Vlp16Model.Rings;
            localDirs = new Vector3[rays];
            for (int c = 0; c < columns; c++)
                for (int r = 0; r < Vlp16Model.Rings; r++)
                    localDirs[c * Vlp16Model.Rings + r] = Vlp16Model.Direction(r, c * 360f / columns);

            commands = new NativeArray<RaycastCommand>(rays, Allocator.Persistent);
            hits = new NativeArray<RaycastHit>(rays, Allocator.Persistent);
            scanData = new byte[rays * Vlp16Model.PointStep];
            scanPoints = new Vector3[rays];
            LastWorldPoints = new Vector3[rays];
        }

        protected override void RegisterPublisher()
        {
            ros.RegisterPublisher<PointCloud2Msg>(config.topic);
            ros.RegisterPublisher<TFMessageMsg>(TfTopic);
        }

        /// <summary>
        /// Where the lidar sits on the vehicle, in ROS axes: from the vehicle body frame (FLU, at the
        /// Rigidbody's origin, as in the ground truth pose) to the lidar frame.
        /// </summary>
        public TransformMsg MountTransform()
        {
            // Worked out from the scene rather than read from the spec, so it always matches where
            // the lidar really is, including a mount moved by hand in the editor.
            Transform body = selfBody != null ? selfBody.transform : transform.root;
            Vector3 position = body.InverseTransformPoint(transform.position);
            Quaternion rotation = Quaternion.Inverse(body.rotation) * transform.rotation;
            return new TransformMsg { translation = position.To<FLU>(), rotation = rotation.To<FLU>() };
        }

        protected override void FixedUpdate()
        {
            bool publish = SimSettings.PublishROS && !ros.HasConnectionError;
            if (!publish && !Application.isEditor)
            {
                // Not running: drop the partial revolution so the next one starts clean.
                column = 0; scanCount = 0; pendingFirings = 0.0;
                return;
            }

            // The firing sequences that fall inside this physics step, cast from this step's pose.
            // A 4 ms step holds 72.3 of them; the fraction is carried to the next step, so over time
            // the count matches the real firing rate exactly instead of drifting by 0.3 per step.
            pendingFirings += Time.fixedDeltaTime / Vlp16Model.FiringCycleSeconds;
            int firings = (int)pendingFirings;
            pendingFirings -= firings;

            double stepTime = ROSClock.SimTime;
            // A loop rather than one Cast because a step can straddle the end of a revolution: cast up
            // to the end, publish, then carry on with the next revolution in the same step.
            int done = 0;
            while (done < firings)
            {
                if (column == 0) scanStartTime = stepTime + done * Vlp16Model.FiringCycleSeconds;
                int take = Mathf.Min(firings - done, columns - column);
                Cast(column, take);
                column += take;
                done += take;
                if (column == columns) FinishRevolution(publish);
            }
        }

        /// <summary>
        /// Casts a whole revolution from the current pose in one go and returns the number of
        /// points, readable from LastWorldPoints. For tests and debugging: it restarts the
        /// revolution in progress and publishes nothing.
        /// </summary>
        public int Scan()
        {
            if (!commands.IsCreated) return 0;
            column = 0; scanCount = 0; pendingFirings = 0.0;
            Cast(0, columns);
            FinishRevolution(false);
            return LastPointCount;
        }

        // Casts count firing sequences starting at firstColumn from the current pose and appends
        // their points to the revolution in progress.
        private void Cast(int firstColumn, int count)
        {
            int rays = count * Vlp16Model.Rings, firstRay = firstColumn * Vlp16Model.Rings;
            Vector3 origin = transform.position;
            Quaternion rotation = transform.rotation;
            var query = new QueryParameters(Physics.DefaultRaycastLayers, false, QueryTriggerInteraction.Ignore, false);

            // Views onto the preallocated arrays, just long enough for this step's rays.
            var batch = commands.GetSubArray(0, rays);
            var results = hits.GetSubArray(0, rays);
            for (int i = 0; i < rays; i++)
                batch[i] = new RaycastCommand(origin, rotation * localDirs[firstRay + i], query, config.maxRange);
            // Unity's job system casts them in parallel, 256 rays per job, keeping only the first hit
            // of each ray (one return, like the sensor in its default strongest-return mode).
            // Complete() waits, so the results are ready on the next line.
            RaycastCommand.ScheduleBatch(batch, results, 256, 1).Complete();

            for (int i = 0; i < rays; i++)
            {
                RaycastHit hit = results[i];
                if (!IsReturn(hit)) continue;
                // Noise is added along the beam, as the real sensor's error is in the measured distance.
                float range = hit.distance + Stochastic.GenerateGaussian() * config.rangeNoiseStdDev;
                // The point in the lidar frame at this step's pose. Points from different steps are
                // in slightly different frames when the vehicle moves: that is the motion distortion.
                Vector3 local = localDirs[firstRay + i] * range;
                int ring = i % Vlp16Model.Rings;
                float time = Vlp16Model.PointTime(firstColumn + i / Vlp16Model.Rings, ring);
                Vlp16Model.WritePoint(scanData, scanCount * Vlp16Model.PointStep, local, Intensity, ring, time);
                scanPoints[scanCount++] = origin + rotation * local;
            }
        }

        // Hands the finished revolution over as the last scan, publishes it and starts the next.
        private void FinishRevolution(bool publish)
        {
            // Swap the buffers instead of copying: the finished revolution becomes the last scan, and
            // the old last scan's array is reused for the next revolution.
            (LastWorldPoints, scanPoints) = (scanPoints, LastWorldPoints);
            LastPointCount = scanCount;
            Revolutions++;

            if (publish)
            {
                // A new array per message: the previous one may still be waiting to be sent.
                var data = new byte[scanCount * Vlp16Model.PointStep];
                Buffer.BlockCopy(scanData, 0, data, 0, data.Length);
                int sec = (int)scanStartTime;
                var stamp = new TimeMsg { sec = sec, nanosec = (uint)((scanStartTime - sec) * 1e9) };
                // The mount goes out just before the cloud and with the same stamp, so anything looking
                // up the cloud's frame finds a transform for that exact time.
                ros.Publish(TfTopic, new TFMessageMsg(new[]
                {
                    new TransformStampedMsg(
                        new HeaderMsg { frame_id = ROSSettings.Instance.BaseLinkFrameId, stamp = stamp },
                        config.frameId, MountTransform()),
                }));
                ros.Publish(config.topic, new PointCloud2Msg
                {
                    header = new HeaderMsg { frame_id = config.frameId, stamp = stamp },
                    height = 1,
                    width = (uint)scanCount,
                    fields = Fields,
                    is_bigendian = false,
                    point_step = Vlp16Model.PointStep,
                    row_step = (uint)data.Length,
                    data = data,
                    is_dense = true,
                });
            }

            column = 0;
            scanCount = 0;
        }

        public override void PublishMessage() { }

        // A hit at or past the minimum range, on something other than the vehicle carrying the lidar.
        private bool IsReturn(RaycastHit hit) =>
            hit.collider != null && hit.distance >= config.minRange
            && (selfBody == null || hit.collider.attachedRigidbody != selfBody);

        private void OnDrawGizmosSelected()
        {
            // Every point, as a short tick. Skipping points at a fixed stride would hide whole rings:
            // the points are stored column by column, so a stride lands on the same lasers each time.
            Gizmos.color = Color.cyan;
            Vector3 tick = Vector3.up * 0.04f;
            for (int i = 0; i < LastPointCount; i++)
                Gizmos.DrawLine(LastWorldPoints[i], LastWorldPoints[i] + tick);
        }

        private void OnDestroy()
        {
            if (commands.IsCreated) commands.Dispose();
            if (hits.IsCreated) hits.Dispose();
        }
    }
}
