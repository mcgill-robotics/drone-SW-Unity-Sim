using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    /// <summary>Scaling and channel conventions of PX4's pwm_out_sim and standard_vtol.</summary>
    public class ActuatorTests
    {
        GameObject go;

        [SetUp]
        public void SetUp() => go = new GameObject("Vehicle", typeof(Rigidbody));

        [TearDown]
        public void TearDown() => Object.DestroyImmediate(go);

        static float[] Controls(int channel, float value)
        {
            var c = new float[VehicleActuators.ChannelCount];
            c[channel] = value;
            return c;
        }

        [TestCase(0f, 0f)]
        [TestCase(0.5f, 0.5f)]
        [TestCase(1f, 1f)]
        [TestCase(-0.3f, 0f)]
        [TestCase(1.4f, 1f)]
        public void Rotor_TakesPx4MotorValueAsThrottle(float px4, float throttle)
        {
            var r = go.AddComponent<Rotor>();
            r.channel = StandardVtolChannels.RotorFrontRight;
            r.Apply(Controls(r.channel, px4), armed: true);
            Assert.AreEqual(throttle, r.Commanded, 1e-6f);
        }

        [TestCase(0f, 0.15f)]
        [TestCase(1f, 1f)]
        [TestCase(0.5f, 0.575f)]
        public void Rotor_IdleSpeed_MapsZeroToIdle(float px4, float speed)
        {
            var r = go.AddComponent<Rotor>();
            r.channel = StandardVtolChannels.RotorFrontRight;
            r.idleSpeed = 0.15f;
            r.Apply(Controls(r.channel, px4), armed: true);
            Assert.AreEqual(speed, r.Commanded, 1e-6f);
        }

        [Test]
        public void Rotor_Disarmed_IsZero()
        {
            var r = go.AddComponent<Rotor>();
            r.channel = StandardVtolChannels.Pusher;
            r.Apply(Controls(r.channel, 0.8f), armed: false);
            Assert.AreEqual(0f, r.Commanded);
        }

        [Test]
        public void ControlSurface_PassesServoValueWhileDisarmed()
        {
            var left = go.AddComponent<ControlSurface>();
            left.channel = StandardVtolChannels.AileronLeft;
            left.torque = new Vector3(-0.5f, 0f, 0f);
            left.Apply(Controls(left.channel, 0.4f), armed: false);
            Assert.AreEqual(0.4f, left.Commanded, 1e-6f);
        }

        [TestCase(1f, 0f, 0f)]
        [TestCase(0f, 1f, 0f)]
        [TestCase(0f, 0f, 1f)]
        [TestCase(-0.5f, 0.2f, 0.3f)]
        public void ControlSurface_MomentInFrd_IsTheTorqueVector(float roll, float pitch, float yaw)
        {
            // PX4's CA_SV_CSn_TRQ_R/P/Y is the FRD moment of a positive deflection.
            var torque = new Vector3(roll, pitch, yaw);
            Vector3 frd = Frames.RateToFrd(ControlSurface.BodyTorque(torque, 2f));
            Assert.Less(Vector3.Distance(2f * torque, frd), 1e-6f, $"got {frd}");
        }

        [Test]
        public void ControlSurface_LeftAileron_RollsLeft()
        {
            // Unity: +z rotation lifts the right wing, so rolling left is a positive z moment.
            Vector3 m = ControlSurface.BodyTorque(new Vector3(-0.5f, 0f, 0f), 1f);
            Assert.Greater(m.z, 0f);
            Assert.AreEqual(0f, m.x);
            Assert.AreEqual(0f, m.y);
        }

        static Rotor TableRotor(GameObject go)
        {
            var r = go.AddComponent<Rotor>();
            r.channel = 0;
            r.timeConstant = 0.02f;
            r.idleSpeed = 0.15f;                 // ignored with a table
            r.tableThrottle = new[] { 0.05f, 0.5f, 1f };
            r.tableThrust = new[] { 0.4f, 6f, 20f };
            r.maxThrust = 20f;
            r.maxTorque = 0.3f;
            return r;
        }

        static void Settle(Rotor r) { for (int i = 0; i < 500; i++) r.Step(0.004f); }

        [TestCase(0.5f, 6f)]
        [TestCase(0.75f, 13f)]
        [TestCase(1f, 20f)]
        public void Rotor_Table_IsLookedUpOnPx4Throttle(float px4, float thrust)
        {
            var r = TableRotor(go);
            r.Apply(Controls(0, px4), armed: true);
            Assert.AreEqual(px4, r.Commanded, 1e-6f);
            Settle(r);
            Assert.AreEqual(thrust, r.Thrust, 1e-3f);
            Assert.AreEqual(thrust * 0.3f / 20f, r.Torque, 1e-4f);
        }

        [Test]
        public void Rotor_Table_StopsWhenDisarmed()
        {
            var r = TableRotor(go);
            r.Apply(Controls(0, 0f), armed: true);
            Settle(r);
            Assert.AreEqual(0.4f, r.Thrust, 1e-3f, "armed at zero throttle makes the first row's thrust");
            r.Apply(Controls(0, 0f), armed: false);
            Settle(r);
            Assert.AreEqual(0f, r.Thrust, 1e-6f);
            Assert.AreEqual(0f, r.Torque, 1e-6f);
        }

        [Test]
        public void Rotor_NoTable_ThrustIsSpeedSquaredWithIdle()
        {
            var r = go.AddComponent<Rotor>();
            r.maxThrust = 8f;
            r.maxTorque = 0.128f;
            r.idleSpeed = 0.15f;
            r.Apply(Controls(r.channel, 0.5f), armed: true);
            Settle(r);
            Assert.AreEqual(8f * 0.575f * 0.575f, r.Thrust, 1e-4f);
            Assert.AreEqual(0.128f * 0.575f * 0.575f, r.Torque, 1e-5f);
        }

        [TestCase(-1f, 0f)]
        [TestCase(0f, 0f)]
        [TestCase(0.25f, 1.5f)]
        [TestCase(0.5f, 3f)]
        [TestCase(0.75f, 5f)]
        [TestCase(1.2f, 7f)]
        public void Rotor_Interpolate_IsPiecewiseLinearAndClamped(float x, float y)
        {
            Assert.AreEqual(y, Rotor.Interpolate(new[] { 0f, 0.5f, 1f }, new[] { 0f, 3f, 7f }, x), 1e-5f);
        }

        [Test]
        public void Rotor_Interpolate_LongTable()
        {
            var xs = new float[101];
            var ys = new float[101];
            for (int i = 0; i <= 100; i++) { xs[i] = i / 100f; ys[i] = 3f * i; }
            for (int i = 0; i < 100; i++)
            {
                Assert.AreEqual(ys[i], Rotor.Interpolate(xs, ys, xs[i]), 1e-3f);
                Assert.AreEqual(ys[i] + 1.5f, Rotor.Interpolate(xs, ys, xs[i] + 0.005f), 1e-3f);
            }
        }

        [Test]
        public void StandardVtolChannels_MatchPx4Airframe1040()
        {
            // PWM_MAIN_FUNC1..8 = 101..105 (motors 1-5), 201..203 (servos 1-3).
            Assert.AreEqual(new[] { 0, 1, 2, 3, 4, 5, 6, 7 }, new[]
            {
                StandardVtolChannels.RotorFrontRight, StandardVtolChannels.RotorRearLeft,
                StandardVtolChannels.RotorFrontLeft, StandardVtolChannels.RotorRearRight,
                StandardVtolChannels.Pusher, StandardVtolChannels.AileronLeft,
                StandardVtolChannels.AileronRight, StandardVtolChannels.Elevator
            });
        }
    }
}
