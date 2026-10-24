using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace DroneSim.Tests
{
    /// <summary>
    /// The flight scene with each shipped vehicle and no PX4. The test runner fails a test on any
    /// logged error or exception, so these also check that the scene starts cleanly.
    /// </summary>
    public class FlightSceneTests
    {
        [TearDown]
        public void TearDown() => VehicleLoader.Override = null;

        [UnityTest]
        public IEnumerator Vehicle_BuildsAndSettlesOnTheGround(
            [ValueSource(typeof(FlightScene), nameof(FlightScene.Vehicles))] string vehicle)
        {
            yield return FlightScene.Load(vehicle);
            var rb = FlightScene.Vehicle;
            Assert.IsNotNull(rb, $"no vehicle built from {vehicle}");
            Assert.IsNotNull(VehicleLoader.Loaded);
            Assert.IsTrue(FlightScene.Bridge != null && FlightScene.Bridge.enabled, "the PX4 bridge turned itself off");
            foreach (var cam in rb.GetComponentsInChildren<CameraPublisher>(true))
                Assert.IsTrue(cam.enabled, $"camera {cam.name} has no usable entry in sim_config.json");

            yield return FlightScene.Steps(750);

            Assert.Less(rb.linearVelocity.magnitude, 0.05f, "still moving");
            Assert.Less(rb.angularVelocity.magnitude, 0.05f, "still rotating");
            Assert.Less(Vector3.Angle(rb.transform.up, Vector3.up), 5f, "not upright");
            // The vehicle spawns over the 4 cm runway strip, so the ground is whatever is under it,
            // not y = 0.
            var box = rb.GetComponent<BoxCollider>();
            var below = Physics.RaycastAll(box.bounds.center, Vector3.down, 10f)
                .Where(h => h.rigidbody != rb).OrderBy(h => h.distance).ToArray();
            Assert.IsNotEmpty(below, "nothing under the vehicle");
            Assert.AreEqual(below[0].point.y, box.bounds.min.y, 0.02f, "the collider's bottom should rest on the ground");

            // Proper acceleration at rest points up: +y in Unity body axes, about 9.81 plus noise.
            var imu = rb.GetComponentInChildren<ImuModel>();
            Assert.Less(Vector3.Distance(new Vector3(0f, 9.81f, 0f), imu.Acceleration), 0.5f, $"IMU reads {imu.Acceleration}");
        }

        [UnityTest]
        public IEnumerator UnknownVehicle_LogsAnErrorAndBuildsNothing()
        {
            var logs = new List<(LogType type, string message)>();
            void Record(string message, string stack, LogType type) { lock (logs) logs.Add((type, message)); }

            // Errors are expected here (the vehicle, then everything that needs it), so collect
            // them and check there's no exception instead of failing on the first.
            LogAssert.ignoreFailingMessages = true;
            Application.logMessageReceivedThreaded += Record;
            try
            {
                yield return FlightScene.Load("no_such_vehicle");
                yield return FlightScene.Steps(25);
            }
            finally
            {
                Application.logMessageReceivedThreaded -= Record;
                LogAssert.ignoreFailingMessages = false;
            }

            Assert.IsNull(FlightScene.Vehicle);
            Assert.IsNull(VehicleLoader.Loaded);
            lock (logs)
            {
                Assert.IsTrue(logs.Any(l => l.type == LogType.Error && l.message.StartsWith("[Vehicle] can't read")),
                    "expected the vehicle loader's error");
                var exceptions = logs.Where(l => l.type == LogType.Exception).Select(l => l.message).ToList();
                Assert.IsEmpty(exceptions, string.Join("\n", exceptions));
            }
        }
    }
}
