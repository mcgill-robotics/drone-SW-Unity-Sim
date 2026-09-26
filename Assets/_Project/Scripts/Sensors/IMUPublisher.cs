using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace DroneSim
{
    /// <summary>
    /// Publishes sensor_msgs/Imu in FLU from an ImuModel on the same GameObject.
    /// </summary>
    [RequireComponent(typeof(ImuModel))]
    public class IMUPublisher : ROSPublisher
    {
        public override string Topic => ROSSettings.Instance.IMUTopic;

        [Tooltip("Publish orientation (true) or mark it unknown with covariance[0] = -1 (false)")]
        public bool publishOrientation = true;

        private ImuModel imu;
        private ImuMsg msg;

        protected override void Start()
        {
            imu = GetComponent<ImuModel>();
            base.Start();
            msg = new ImuMsg
            {
                header = new HeaderMsg { frame_id = ROSSettings.Instance.ImuFrameId },
                orientation_covariance = new double[9],
                angular_velocity_covariance = new double[9],
                linear_acceleration_covariance = new double[9]
            };
            double gv = imu.gyroNoise * imu.gyroNoise, av = imu.accelNoise * imu.accelNoise;
            for (int i = 0; i < 9; i += 4) { msg.angular_velocity_covariance[i] = gv; msg.linear_acceleration_covariance[i] = av; }
            if (!publishOrientation) msg.orientation_covariance[0] = -1.0;
        }

        protected override void RegisterPublisher() => ros.RegisterPublisher<ImuMsg>(Topic);

        public override void PublishMessage()
        {
            msg.header.stamp = ROSClock.GetROSTimestamp();
            msg.linear_acceleration = imu.Acceleration.To<FLU>();
            Vector3 w = imu.AngularRate;
            // Angular rate is a pseudovector: handedness flip negates it after the axis swap.
            msg.angular_velocity = new Vector3Msg { x = -w.z, y = w.x, z = -w.y };
            msg.orientation = publishOrientation ? imu.Orientation.To<FLU>() : new QuaternionMsg();
            ros.Publish(Topic, msg);
        }
    }
}
