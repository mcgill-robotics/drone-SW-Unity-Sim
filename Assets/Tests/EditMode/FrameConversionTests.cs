using DroneSim;
using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    /// <summary>
    /// Unity is left-handed x right, y up, z forward (north). PX4 wants FRD body and NED world;
    /// ROS wants FLU body and ENU world. Each case is a motion whose aero sign is unambiguous.
    /// </summary>
    public class FrameConversionTests
    {
        const float Eps = 1e-5f;

        static void AssertVec(Vector3 expected, Vector3 actual) =>
            Assert.Less(Vector3.Distance(expected, actual), Eps, $"expected {expected} got {actual}");

        [Test]
        public void BodyToFrd_MapsAxes()
        {
            AssertVec(new Vector3(1, 0, 0), Frames.BodyToFrd(Vector3.forward));
            AssertVec(new Vector3(0, 1, 0), Frames.BodyToFrd(Vector3.right));
            AssertVec(new Vector3(0, 0, 1), Frames.BodyToFrd(Vector3.down));
        }

        [Test]
        public void Accelerometer_AtRest_ReadsMinusGInFrdZ()
        {
            // Proper acceleration at rest points up; FRD z is down, so PX4 expects about -9.8.
            AssertVec(new Vector3(0, 0, -9.81f), Frames.BodyToFrd(new Vector3(0, 9.81f, 0)));
        }

        [Test]
        public void RateToFrd_RollRight_IsPositiveP()
        {
            // Unity +z rotation lifts the right wing (roll left), so right-wing-down is -z.
            AssertVec(new Vector3(1, 0, 0), Frames.RateToFrd(new Vector3(0, 0, -1)));
        }

        [Test]
        public void RateToFrd_NoseUp_IsPositiveQ()
        {
            // Unity +x rotation drops the nose.
            AssertVec(new Vector3(0, 1, 0), Frames.RateToFrd(new Vector3(-1, 0, 0)));
        }

        [Test]
        public void RateToFrd_YawRight_IsPositiveR()
        {
            AssertVec(new Vector3(0, 0, 1), Frames.RateToFrd(new Vector3(0, 1, 0)));
        }

        [Test]
        public void UnityRotationConventions_HoldAsAssumed()
        {
            AssertVec(Vector3.up, Quaternion.Euler(0, 0, 90) * Vector3.right);      // +z lifts right wing
            AssertVec(Vector3.down, Quaternion.Euler(90, 0, 0) * Vector3.forward);  // +x drops nose
            AssertVec(Vector3.right, Quaternion.Euler(0, 90, 0) * Vector3.forward); // +y yaws right
        }

        [TestCase(0f, 0f, 0f)]
        [TestCase(0f, 90f, 0f)]
        [TestCase(-30f, 0f, 0f)]
        [TestCase(0f, 0f, -20f)]
        [TestCase(-10f, 135f, 25f)]
        public void ToNedFrd_RotatesFrdVectorsLikeUnityRotatesBodyVectors(float x, float y, float z)
        {
            Quaternion unity = Quaternion.Euler(x, y, z);
            Vector4 q = Px4HilBridge.ToNedFrd(unity);
            var ned = new Quaternion(q.x, q.y, q.z, q.w);
            Assert.AreEqual(1f, new Vector4(q.x, q.y, q.z, q.w).magnitude, Eps);

            foreach (var body in new[] { Vector3.forward, Vector3.right, Vector3.up })
            {
                Vector3 worldUnity = unity * body;
                Vector3 expectedNed = new Vector3(worldUnity.z, worldUnity.x, -worldUnity.y);
                AssertVec(expectedNed, ned * Frames.BodyToFrd(body));
            }
        }

        [Test]
        public void ToNedFrd_YawRight90_IsHeadingEast()
        {
            Vector4 q = Px4HilBridge.ToNedFrd(Quaternion.Euler(0, 90, 0));
            float s = Mathf.Sqrt(0.5f);
            Assert.AreEqual(s, q.w, Eps);
            Assert.AreEqual(0f, q.x, Eps);
            Assert.AreEqual(0f, q.y, Eps);
            Assert.AreEqual(s, q.z, Eps);
        }

        /// <summary>Unity body to FLU, the same mapping as the ROS connector's To&lt;FLU&gt;().</summary>
        static Vector3 Flu(Vector3 body) => new Vector3(body.z, -body.x, body.y);

        [Test]
        public void ToEnu_MapsAxes()
        {
            AssertVec(new Vector3(1, 0, 0), GeoOrigin.ToEnu(Vector3.right));
            AssertVec(new Vector3(0, 1, 0), GeoOrigin.ToEnu(Vector3.forward));
            AssertVec(new Vector3(0, 0, 1), GeoOrigin.ToEnu(Vector3.up));
        }

        [TestCase(0f, 0f, 0f)]
        [TestCase(0f, 90f, 0f)]
        [TestCase(-30f, 0f, 0f)]
        [TestCase(0f, 0f, -20f)]
        [TestCase(-10f, 135f, 25f)]
        public void ToEnuFlu_RotatesFluVectorsLikeUnityRotatesBodyVectors(float x, float y, float z)
        {
            Quaternion unity = Quaternion.Euler(x, y, z);
            Quaternion enu = GeoOrigin.ToEnuFlu(unity);
            Assert.AreEqual(1f, new Vector4(enu.x, enu.y, enu.z, enu.w).magnitude, Eps);

            foreach (var body in new[] { Vector3.forward, Vector3.right, Vector3.up })
                AssertVec(GeoOrigin.ToEnu(unity * body), enu * Flu(body));
        }

        [Test]
        public void ToEnuFlu_FacingEast_IsIdentity()
        {
            Quaternion q = GeoOrigin.ToEnuFlu(Quaternion.Euler(0, 90, 0));
            Assert.AreEqual(1f, Mathf.Abs(q.w), Eps);
        }

        [Test]
        public void PressureAtAltitude_IsStandardAtmosphere()
        {
            Assert.AreEqual(1013.25f, Px4HilBridge.PressureAtAltitude(0f), 1e-3f);
            Assert.AreEqual(898.75f, Px4HilBridge.PressureAtAltitude(1000f), 0.1f);
        }
    }
}
