using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    /// <summary>Body drag: quadratic drag on the airframe and linear angular damping, configured in the vehicle spec.</summary>
    public class BodyDragTests
    {
        const float SeaLevelDensity = 1.225f;

        static void AssertVector(Vector3 expected, Vector3 actual, float tolerance)
        {
            Assert.AreEqual(expected.x, actual.x, tolerance, $"x of {actual}");
            Assert.AreEqual(expected.y, actual.y, tolerance, $"y of {actual}");
            Assert.AreEqual(expected.z, actual.z, tolerance, $"z of {actual}");
        }

        [Test]
        public void Force_QuadraticInSpeed_AlongForward()
        {
            // -1/2 x 1.225 x 0.02 m^2 x 20 m/s x 20 m/s = -4.9 N along z, nothing on the other axes.
            Vector3 f = BodyDrag.Force(new Vector3(0f, 0f, 20f), new Vector3(0.01f, 0.05f, 0.02f), SeaLevelDensity);
            AssertVector(new Vector3(0f, 0f, -4.9f), f, 1e-4f);
        }

        [Test]
        public void Force_IsOddInVelocity()
        {
            var area = new Vector3(0.01f, 0.05f, 0.02f);
            Vector3 v = new Vector3(3f, -7f, 11f);
            Vector3 forward = BodyDrag.Force(v, area, SeaLevelDensity);
            Vector3 backward = BodyDrag.Force(-v, area, SeaLevelDensity);
            Assert.AreEqual(-forward.x, backward.x);
            Assert.AreEqual(-forward.y, backward.y);
            Assert.AreEqual(-forward.z, backward.z);
        }

        [Test]
        public void Force_AtTerminalSpeed_BalancesWeight()
        {
            // 2 kg falling flat at CdA 0.05 m^2 stops accelerating where drag equals weight.
            float mass = 2f, cdA = 0.05f;
            float terminal = Mathf.Sqrt(2f * mass * VehicleSpec.Gravity / (SeaLevelDensity * cdA));   // 25.3 m/s
            Vector3 f = BodyDrag.Force(new Vector3(0f, -terminal, 0f), new Vector3(0f, cdA, 0f), SeaLevelDensity);
            Assert.AreEqual(mass * VehicleSpec.Gravity, f.y, 1e-3f);
        }

        [Test]
        public void Force_ZeroVelocity_IsZero()
        {
            Vector3 f = BodyDrag.Force(Vector3.zero, new Vector3(0.01f, 0.05f, 0.02f), SeaLevelDensity);
            Assert.AreEqual(0f, f.magnitude);
        }

        [Test]
        public void Moment_IsLinearAndOpposesRotation()
        {
            Vector3 m = BodyDrag.Moment(new Vector3(1f, 2f, 3f), new Vector3(0.1f, 0.2f, 0.3f));
            AssertVector(new Vector3(-0.1f, -0.4f, -0.9f), m, 1e-6f);
        }

        static VehicleSpec X500WithBodyDrag()
        {
            var spec = VehicleSpec.Load(VehicleSpec.Resolve("x500"));
            spec.bodyDrag.enabled = true;
            spec.bodyDrag.cdA = new[] { 0.05f, 0.05f, 0.1f };               // FRD: x forward, y right, z down
            spec.bodyDrag.angularDamping = new[] { 0.01f, 0.01f, 0.02f };   // roll, pitch, yaw
            return spec;
        }

        [Test]
        public void Builder_WithBodyDragEnabled_AddsComponentAndClearsLegacyDamping()
        {
            var go = VehicleBuilder.Build(X500WithBodyDrag(), Vector3.zero);
            try
            {
                var drag = go.GetComponent<BodyDrag>();
                Assert.IsNotNull(drag);
                Assert.AreEqual(0f, go.GetComponent<Rigidbody>().angularDamping);
                // Unity axes: x right = FRD y, y up = FRD -z, z forward = FRD x.
                AssertVector(new Vector3(0.05f, 0.1f, 0.05f), drag.dragArea, 1e-6f);
                // Unity x = pitch, y = yaw, z = roll.
                AssertVector(new Vector3(0.01f, 0.02f, 0.01f), drag.angularDamping, 1e-6f);
            }
            finally { Object.DestroyImmediate(go); }
        }

        [Test]
        public void Builder_WithBodyDragOff_KeepsLegacyDamping()
        {
            var go = VehicleBuilder.Build(VehicleSpec.Load(VehicleSpec.Resolve("x500")), Vector3.zero);
            try
            {
                Assert.IsNull(go.GetComponent<BodyDrag>());
                Assert.AreEqual(0.05f, go.GetComponent<Rigidbody>().angularDamping);
            }
            finally { Object.DestroyImmediate(go); }
        }
    }
}
