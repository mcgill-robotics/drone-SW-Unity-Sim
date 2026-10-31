using System;
using System.Collections;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace DroneSim.Tests
{
    /// <summary>
    /// The PX4 bridge over a real socket, with FakePx4 on the other end: what PX4 receives from a
    /// vehicle at rest, lockstep, PX4 stopping and restarting, and arming.
    /// </summary>
    public class Px4BridgeTests
    {
        FakePx4 px4;

        [TearDown]
        public void TearDown()
        {
            px4?.Dispose();
            px4 = null;
            VehicleLoader.Override = null;
        }

        static IEnumerator Until(Func<bool> condition, string what, float seconds = 15f)
        {
            float end = Time.realtimeSinceStartup + seconds;
            while (!condition())
            {
                if (Time.realtimeSinceStartup > end) Assert.Fail($"timed out after {seconds} s waiting for {what}");
                yield return null;
            }
        }

        /// <summary>Waits for this many more physics steps, counted by the HIL_SENSOR messages PX4 gets.</summary>
        IEnumerator SimSteps(int count)
        {
            int target = px4.Sensors + count;
            yield return Until(() => px4.Sensors >= target, $"{count} sim steps");
        }

        [UnityTest]
        public IEnumerator Sensors_AtRest_MatchTheVehicleAndHome()
        {
            yield return FlightScene.Load(FlightScene.Vehicles[0]);
            yield return FlightScene.Steps(500); // land and settle
            px4 = new FakePx4(FlightScene.Bridge.port);
            yield return Until(() => px4.Sensors >= 300 && px4.GpsFixes > 0 && px4.States > 0 && px4.Heartbeats > 0,
                "300 HIL_SENSOR, HIL_GPS, HIL_STATE_QUATERNION and a heartbeat");

            // PX4's clock is the sim's: one HIL_SENSOR per physics step, exactly one step apart.
            ulong step = (ulong)(Time.fixedDeltaTime * 1e6f);
            Assert.AreEqual((step, step), px4.SensorSteps);

            var rb = FlightScene.Vehicle;
            var geo = GeoOrigin.Instance;
            geo.ToGeodetic(rb.position, out double lat, out double lon, out double alt);
            var s = px4.LastSensor;

            // Level, facing north, at rest: FRD is NED, and the accelerometer reads -g on z.
            Assert.Less(Vector3.Distance(new Vector3(0f, 0f, -9.81f), s.acc), 0.5f, $"accelerometer {s.acc}");
            Assert.Less(s.gyro.magnitude, 0.05f, $"gyro {s.gyro}");
            Assert.Less(Vector3.Distance(geo.MagneticFieldNed, s.mag), 0.01f, $"magnetometer {s.mag}, field {geo.MagneticFieldNed}");
            Assert.AreEqual(Px4HilBridge.PressureAtAltitude((float)alt), s.absPressure, 0.2f, "barometer (hPa)");
            Assert.AreEqual(alt, s.pressureAlt, 0.5, "pressure altitude");
            Assert.AreEqual(0f, s.diffPressure, 0.02f, "airspeed pressure (hPa)");

            var gps = px4.LastGps;
            Assert.AreEqual(3, gps.fixType, "3D fix");
            Assert.AreEqual(lat, gps.lat, 2e-5);
            Assert.AreEqual(lon, gps.lon, 2e-5);
            Assert.AreEqual(alt, gps.alt, 2.0);

            var state = px4.LastState;
            Assert.Greater(Mathf.Abs(state.attitude.w), 0.999f, $"attitude {state.attitude}");
            Assert.Less(state.velocityNed.magnitude, 0.05f, $"velocity {state.velocityNed}");
        }

        [UnityTest]
        public IEnumerator Lockstep_WaitsForPx4_PausesWhenItStops_AndResumes()
        {
            yield return FlightScene.Load(FlightScene.Vehicles[0]);
            var bridge = FlightScene.Bridge;
            Assert.IsFalse(bridge.Connected);

            px4 = new FakePx4(bridge.port);
            yield return Until(() => bridge.ReceivedControls, "lockstep to engage on the first controls");

            // A PX4 that takes 20 ms per answer holds each 4 ms step to 20 ms.
            px4.answerDelayMs = 20;
            yield return SimSteps(5);
            float start = Time.realtimeSinceStartup;
            yield return SimSteps(50);
            Assert.Greater(Time.realtimeSinceStartup - start, 0.9f, "the sim ran ahead of PX4");
            px4.answerDelayMs = 0;

            // PX4 stops answering: after a few timed-out steps the sim stops waiting.
            bridge.lockstepTimeoutMs = 20;
            px4.answer = false;
            yield return Until(() => !bridge.ReceivedControls, "lockstep to pause");
            Assert.IsTrue(bridge.Connected);

            px4.answer = true;
            yield return Until(() => bridge.ReceivedControls, "lockstep to resume");
        }

        [UnityTest]
        public IEnumerator Px4Restart_DropsTheLinkAndTakesTheNextOne()
        {
            yield return FlightScene.Load(FlightScene.Vehicles[0]);
            var bridge = FlightScene.Bridge;
            bridge.lockstepTimeoutMs = 200;

            px4 = new FakePx4(bridge.port);
            yield return Until(() => bridge.ReceivedControls, "lockstep with the first PX4");
            px4.Dispose();
            yield return Until(() => !bridge.Connected, "the bridge to notice PX4 is gone");
            Assert.IsFalse(bridge.ReceivedControls);

            px4 = new FakePx4(bridge.port);
            yield return Until(() => bridge.ReceivedControls && px4.Sensors > 10, "lockstep with the restarted PX4");
        }

        [UnityTest]
        public IEnumerator Arming_SpinsTheLiftRotors_AndTheVehicleClimbs(
            [ValueSource(typeof(FlightScene), nameof(FlightScene.Vehicles))] string vehicle)
        {
            yield return FlightScene.Load(vehicle);
            yield return FlightScene.Steps(500); // land and settle
            var rb = FlightScene.Vehicle;
            var liftChannels = VehicleLoader.Loaded.LiftRotors.Select(r => r.channel).ToList();
            Assert.IsNotEmpty(liftChannels);
            var lift = rb.GetComponentsInChildren<Rotor>().Where(r => liftChannels.Contains(r.channel)).ToList();
            var full = new float[VehicleActuators.ChannelCount];
            foreach (int ch in liftChannels) full[ch] = 1f;

            // Full throttle while disarmed does nothing.
            px4 = new FakePx4(FlightScene.Bridge.port);
            px4.Command(false, full);
            yield return SimSteps(125);
            Assert.IsTrue(lift.All(r => r.Commanded == 0f && r.Thrust == 0f), "a disarmed rotor is turning");
            Assert.Less(rb.linearVelocity.magnitude, 0.05f);
            float y0 = rb.position.y;

            // Armed, it lifts off. Kept short: the throttle is open loop, so it would tip over in time.
            px4.Command(true, full);
            yield return SimSteps(100);
            foreach (var r in lift) Assert.AreEqual(1f, r.Commanded, 1e-6f, $"{r.name} commanded");
            Assert.Greater(rb.linearVelocity.y, 0.3f, "not climbing");
            Assert.Greater(rb.position.y - y0, 0.03f, "didn't leave the ground");
            Assert.Less(Vector3.Angle(rb.transform.up, Vector3.up), 45f, "tipped over");
            Assert.Less(px4.LastState.velocityNed.z, -0.1f, "PX4 should see the climb as negative down velocity");
        }
    }
}
