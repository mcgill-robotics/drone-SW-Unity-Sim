using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    /// <summary>Motor spin-up/spin-down lag and command dead time of a Rotor.</summary>
    public class RotorDynamicsTests
    {
        GameObject go;

        [SetUp]
        public void SetUp() => go = new GameObject("Vehicle", typeof(Rigidbody));

        [TearDown]
        public void TearDown() => Object.DestroyImmediate(go);

        static float[] Controls(float value)
        {
            var c = new float[VehicleActuators.ChannelCount];
            c[0] = value;
            return c;
        }

        // A rotor with no idle and a quadratic thrust curve, so Throttle is the lagged speed fraction.
        static Rotor NewRotor(GameObject target)
        {
            var r = target.AddComponent<Rotor>();
            r.channel = 0;
            r.maxThrust = 8f;
            r.maxTorque = 0.128f;
            r.idleSpeed = 0f;
            return r;
        }

        static void StepFor(Rotor r, int steps, float dt)
        {
            for (int i = 0; i < steps; i++) r.Step(dt);
        }

        [Test]
        public void Lag_Rising_ReachesOneMinusInvEAtTimeConstantUp()
        {
            var r = NewRotor(go);
            r.timeConstantUp = 0.0125f;
            r.timeConstantDown = 0.025f;
            r.Apply(Controls(1f), armed: true);
            StepFor(r, 5, 0.0025f);                       // 0.0125 s = one time constant
            Assert.AreEqual(0.632121f, r.Throttle, 1e-4f);
        }

        [Test]
        public void Lag_Falling_ReachesInvEAtTimeConstantDown()
        {
            var r = NewRotor(go);
            r.timeConstantUp = 0.0125f;
            r.timeConstantDown = 0.025f;
            r.Apply(Controls(1f), armed: true);
            StepFor(r, 500, 0.004f);                      // settle at full speed
            r.Apply(Controls(0f), armed: true);
            StepFor(r, 10, 0.0025f);                      // 0.025 s = one time constant
            Assert.AreEqual(0.367879f, r.Throttle, 1e-4f);
        }

        [Test]
        public void Lag_IsIndependentOfStepSize()
        {
            float Rise(int steps, float dt)
            {
                var g = new GameObject("Vehicle", typeof(Rigidbody));
                try
                {
                    var r = NewRotor(g);
                    r.timeConstantUp = 0.0125f;
                    r.timeConstantDown = 0.025f;
                    r.Apply(Controls(1f), armed: true);
                    StepFor(r, steps, dt);
                    return r.Throttle;
                }
                finally { Object.DestroyImmediate(g); }
            }

            float coarse = Rise(1, 0.0125f), medium = Rise(5, 0.0025f), fine = Rise(25, 0.0005f);
            Assert.AreEqual(medium, coarse, 1e-4f);
            Assert.AreEqual(medium, fine, 1e-4f);
        }

        [Test]
        public void Lag_UnsetUpAndDown_UseTimeConstant()
        {
            var r = NewRotor(go);
            r.timeConstant = 0.05f;
            r.Apply(Controls(1f), armed: true);
            StepFor(r, 20, 0.0025f);                      // 0.05 s
            Assert.AreEqual(0.632121f, r.Throttle, 1e-4f);
        }

        [Test]
        public void Latency_HoldsCommandUntilItsAgeReachesLatency()
        {
            var r = NewRotor(go);
            r.timeConstant = 0.02f;
            r.latency = 0.02f;
            r.Apply(Controls(1f), armed: true);
            StepFor(r, 5, 0.004f);                        // 0.020 s of dead time, no response yet
            Assert.AreEqual(0f, r.Throttle);
            r.Step(0.004f);                               // the first step after it
            Assert.Greater(r.Throttle, 0f);
        }

        [Test]
        public void Latency_DisarmBeforeItElapses_NeverRises()
        {
            var r = NewRotor(go);
            r.timeConstant = 0.02f;
            r.latency = 0.02f;
            r.Apply(Controls(1f), armed: true);
            r.Step(0.004f);
            r.Apply(Controls(1f), armed: false);
            StepFor(r, 20, 0.004f);
            Assert.AreEqual(0f, r.Throttle);
            Assert.AreEqual(0f, r.Commanded);
        }

        [Test]
        public void Latency_ResetClearsQueuedCommands()
        {
            var r = NewRotor(go);
            r.timeConstant = 0.02f;
            r.latency = 0.02f;
            r.Apply(Controls(1f), armed: true);
            r.ResetActuator();
            StepFor(r, 20, 0.004f);
            Assert.AreEqual(0f, r.Throttle);
        }

        [TestCase(0.004f, 5)]
        [TestCase(0.002f, 10)]
        [TestCase(0.001f, 20)]
        public void Latency_DeadTimeIsIndependentOfStepSize(float dt, int silentSteps)
        {
            var r = NewRotor(go);
            r.timeConstant = 0.02f;
            r.latency = 0.02f;
            r.Apply(Controls(1f), armed: true);
            StepFor(r, silentSteps, dt);                  // silentSteps x dt = 0.02 s, whatever the step
            Assert.AreEqual(0f, r.Throttle);
            r.Step(dt);
            Assert.Greater(r.Throttle, 0f);
        }

        [Test]
        public void Latency_PendingCommandsAreCapped()
        {
            // A rotor that is never stepped (disabled) still gets Apply every physics step.
            var r = NewRotor(go);
            r.latency = 0.02f;
            for (int i = 0; i < 5000; i++) r.Apply(Controls(1f), armed: true);
            Assert.LessOrEqual(r.PendingCommands, Rotor.MaxPendingCommands);
        }
    }
}
