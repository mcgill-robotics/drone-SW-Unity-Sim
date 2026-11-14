using System;
using Unity.Collections;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

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
    /// Like the cameras it only runs while the ROS endpoint is connected. In the editor it runs
    /// regardless, so the points can be checked in the Scene view with the lidar selected.
    /// </summary>
    public class LidarPublisher : ROSPublisher
    {
        // Unity has no reflectivity to measure, so every point gets the same intensity.
        const float Intensity = 100f;

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
        private Rigidbody selfBody;
        private int columns;           // firing sequences per revolution
        private Vector3[] localDirs;   // one per ray, column by column, ring 0 first
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
            config = SimConfig.Current.FindLidar(name);
            if (config == null) { Debug.LogWarning($"[LidarPublisher] no sim config entry for {name}, not publishing"); enabled = false; return; }
            string problem = config.Problem();
            if (problem != null) { Debug.LogError($"[LidarPublisher] sim config entry for {name}: {problem}, not publishing"); enabled = false; return; }
            PublishRate = config.rotationHz;

            base.Start();
            useBaseRateLimiting = false;

            selfBody = GetComponentInParent<Rigidbody>();

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

        protected override void RegisterPublisher() => ros.RegisterPublisher<PointCloud2Msg>(config.topic);

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
            pendingFirings += Time.fixedDeltaTime / Vlp16Model.FiringCycleSeconds;
            int firings = (int)pendingFirings;
            pendingFirings -= firings;

            double stepTime = ROSClock.SimTime;
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

            var batch = commands.GetSubArray(0, rays);
            var results = hits.GetSubArray(0, rays);
            for (int i = 0; i < rays; i++)
                batch[i] = new RaycastCommand(origin, rotation * localDirs[firstRay + i], query, config.maxRange);
            RaycastCommand.ScheduleBatch(batch, results, 256, 1).Complete();

            for (int i = 0; i < rays; i++)
            {
                RaycastHit hit = results[i];
                if (!IsReturn(hit)) continue;
                float range = hit.distance + Stochastic.GenerateGaussian() * config.rangeNoiseStdDev;
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
            (LastWorldPoints, scanPoints) = (scanPoints, LastWorldPoints);
            LastPointCount = scanCount;
            Revolutions++;

            if (publish)
            {
                // A new array per message: the previous one may still be waiting to be sent.
                var data = new byte[scanCount * Vlp16Model.PointStep];
                Buffer.BlockCopy(scanData, 0, data, 0, data.Length);
                int sec = (int)scanStartTime;
                ros.Publish(config.topic, new PointCloud2Msg
                {
                    header = new HeaderMsg
                    {
                        frame_id = config.frameId,
                        stamp = new TimeMsg { sec = sec, nanosec = (uint)((scanStartTime - sec) * 1e9) },
                    },
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
