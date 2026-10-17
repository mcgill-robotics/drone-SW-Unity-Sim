using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace DroneSim
{
    /// <summary>
    /// Ground truth pose (ENU, relative to the GeoOrigin), body twist (FLU) and NavSatFix.
    /// For validating estimators and mapping.
    /// </summary>
    public class GroundTruthPublisher : ROSPublisher
    {
        public override string Topic => ROSSettings.Instance.GroundTruthPoseTopic;

        private Rigidbody rb;
        private PoseStampedMsg pose;
        private TwistStampedMsg twist;
        private NavSatFixMsg fix;

        protected override void Start()
        {
            base.Start();
            rb = SimSettings.Vehicle;
            pose = new PoseStampedMsg { header = new HeaderMsg { frame_id = ROSSettings.Instance.WorldFrameId } };
            twist = new TwistStampedMsg { header = new HeaderMsg { frame_id = ROSSettings.Instance.BaseLinkFrameId } };
            fix = new NavSatFixMsg
            {
                header = new HeaderMsg { frame_id = ROSSettings.Instance.BaseLinkFrameId },
                status = new NavSatStatusMsg { status = NavSatStatusMsg.STATUS_FIX, service = NavSatStatusMsg.SERVICE_GPS },
                position_covariance_type = NavSatFixMsg.COVARIANCE_TYPE_UNKNOWN,
                position_covariance = new double[9]
            };
        }

        protected override void RegisterPublisher()
        {
            var s = ROSSettings.Instance;
            ros.RegisterPublisher<PoseStampedMsg>(s.GroundTruthPoseTopic);
            ros.RegisterPublisher<TwistStampedMsg>(s.GroundTruthTwistTopic);
            ros.RegisterPublisher<NavSatFixMsg>(s.GroundTruthGpsTopic);
        }

        public override void PublishMessage()
        {
            if (rb == null) return;
            var s = ROSSettings.Instance;
            var stamp = ROSClock.GetROSTimestamp();

            Vector3 local = GeoOrigin.Instance != null ? GeoOrigin.Instance.ToLocal(rb.position) : rb.position;
            Vector3 enu = GeoOrigin.ToEnu(local);
            Quaternion q = GeoOrigin.ToEnuFlu(rb.rotation);
            pose.header.stamp = stamp;
            pose.pose.position = new PointMsg(enu.x, enu.y, enu.z);
            pose.pose.orientation = new QuaternionMsg(q.x, q.y, q.z, q.w);
            ros.Publish(s.GroundTruthPoseTopic, pose);

            Vector3 v = rb.transform.InverseTransformDirection(rb.linearVelocity);
            Vector3 w = rb.transform.InverseTransformDirection(rb.angularVelocity);
            twist.header.stamp = stamp;
            twist.twist.linear = v.To<FLU>();
            twist.twist.angular = new Vector3Msg { x = -w.z, y = w.x, z = -w.y };
            ros.Publish(s.GroundTruthTwistTopic, twist);

            if (GeoOrigin.Instance != null)
            {
                GeoOrigin.Instance.ToGeodetic(rb.position, out double lat, out double lon, out double alt);
                fix.header.stamp = stamp;
                fix.latitude = lat; fix.longitude = lon; fix.altitude = alt;
                ros.Publish(s.GroundTruthGpsTopic, fix);
            }
        }
    }
}
