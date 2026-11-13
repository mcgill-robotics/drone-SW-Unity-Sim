using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    /// <summary>
    /// Rotor drag: the force a spinning propeller feels from air flowing across its disc, as in PX4's
    /// gz MulticopterMotorModel (rotorDragCoefficient), and the speed fraction that feeds it.
    /// </summary>
    public class RotorDragTests
    {
        const float GazeboCoefficient = 8.06428e-5f;   // kg/rad, PX4 gz x500
        static readonly Vector3 Up = Vector3.up;

        GameObject go;

        [SetUp]
        public void SetUp() => go = new GameObject("Vehicle", typeof(Rigidbody));

        [TearDown]
        public void TearDown() => Object.DestroyImmediate(go);

        static void AssertVector(Vector3 expected, Vector3 actual, float tolerance)
        {
            Assert.AreEqual(expected.x, actual.x, tolerance, $"x of {actual}");
            Assert.AreEqual(expected.y, actual.y, tolerance, $"y of {actual}");
            Assert.AreEqual(expected.z, actual.z, tolerance, $"z of {actual}");
        }

        [Test]
        public void DragForce_OpposesPerpendicularAirflow()
        {
            // 500 rad/s x 8.06428e-5 kg/rad x 10 m/s = 0.403214 N, against the wind.
            Vector3 f = Rotor.DragForce(new Vector3(10f, 0f, 0f), Up, 500f, GazeboCoefficient);
            AssertVector(new Vector3(-0.403214f, 0f, 0f), f, 1e-5f);
        }

        [Test]
        public void DragForce_IgnoresAirflowAlongTheAxis()
        {
            Vector3 f = Rotor.DragForce(new Vector3(0f, 5f, 0f), Up, 500f, GazeboCoefficient);
            Assert.AreEqual(0f, f.magnitude, 1e-9f);
        }

        [Test]
        public void DragForce_UsesOnlyThePerpendicularPart()
        {
            // The 4 m/s along the axis is dropped; 3 m/s across it remains.
            Vector3 f = Rotor.DragForce(new Vector3(3f, 4f, 0f), Up, 500f, GazeboCoefficient);
            AssertVector(new Vector3(-0.120964f, 0f, 0f), f, 1e-5f);
        }

        [Test]
        public void DragForce_ZeroAirflow_IsExactlyZero()
        {
            Vector3 f = Rotor.DragForce(Vector3.zero, Up, 500f, GazeboCoefficient);
            Assert.AreEqual(0f, f.magnitude);
            Assert.IsFalse(float.IsNaN(f.x) || float.IsNaN(f.y) || float.IsNaN(f.z));
        }

        [Test]
        public void DragForce_ZeroSpeed_IsExactlyZero()
        {
            Vector3 f = Rotor.DragForce(new Vector3(10f, 0f, 0f), Up, 0f, GazeboCoefficient);
            Assert.AreEqual(0f, f.magnitude);
        }

        [Test]
        public void DragForce_NonUnitAxis_IsNormalised()
        {
            Vector3 unit = Rotor.DragForce(new Vector3(3f, 4f, 0f), Up, 500f, GazeboCoefficient);
            Vector3 scaled = Rotor.DragForce(new Vector3(3f, 4f, 0f), new Vector3(0f, 3f, 0f), 500f, GazeboCoefficient);
            AssertVector(unit, scaled, 1e-6f);
        }

        static void Settle(Rotor r) { for (int i = 0; i < 500; i++) r.Step(0.004f); }

        static float[] Controls(float value)
        {
            var c = new float[VehicleActuators.ChannelCount];
            c[0] = value;
            return c;
        }

        [Test]
        public void SpeedFraction_NoTable_EqualsThrottle()
        {
            var r = go.AddComponent<Rotor>();
            r.channel = 0;
            r.maxThrust = 8f;
            r.maxTorque = 0.128f;
            r.Apply(Controls(0.5f), armed: true);
            Settle(r);
            Assert.AreEqual(0.5f, r.SpeedFraction, 1e-4f);
        }

        [Test]
        public void SpeedFraction_Table_IsSquareRootOfThrustRatio()
        {
            var r = go.AddComponent<Rotor>();
            r.channel = 0;
            r.timeConstant = 0.02f;
            r.tableThrottle = new[] { 0.05f, 0.5f, 1f };
            r.tableThrust = new[] { 0.4f, 6f, 20f };
            r.maxThrust = 20f;
            r.maxTorque = 0.3f;
            r.Apply(Controls(0.5f), armed: true);
            Settle(r);
            Assert.AreEqual(Mathf.Sqrt(6f / 20f), r.SpeedFraction, 1e-4f);   // 0.547723
        }
    }
}
