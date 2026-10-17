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
