using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    public class VehicleSpecTests
    {
        static VehicleSpec Shipped(string name) => VehicleSpec.Load(VehicleSpec.Resolve(name));

        [Test]
        public void FrdToUnity_MapsForwardRightDown()
        {
            Assert.AreEqual(new Vector3(0, 0, 1), Frames.FrdToUnity(new Vector3(1, 0, 0)));
            Assert.AreEqual(new Vector3(1, 0, 0), Frames.FrdToUnity(new Vector3(0, 1, 0)));
            Assert.AreEqual(new Vector3(0, -1, 0), Frames.FrdToUnity(new Vector3(0, 0, 1)));
        }

        [Test]
        public void Inertia_Diagonal_MapsRollPitchYawToUnityAxes()
        {
            // Unity x = pitch, y = yaw, z = roll.
            Frames.InertiaToUnity(new InertiaSpec { ixx = 1, iyy = 2, izz = 3 }, out Vector3 p, out Quaternion r);
            Assert.AreEqual(new Vector3(2, 3, 1), p);
            Assert.AreEqual(Quaternion.identity, r);
        }

        [Test]
        public void Inertia_WithProducts_ReconstructsTheTensor()
        {
            var i = new InertiaSpec { ixx = 0.4f, iyy = 0.6f, izz = 0.9f, ixy = 0.02f, ixz = -0.05f, iyz = 0.01f };
            Frames.InertiaToUnity(i, out Vector3 p, out Quaternion r);

            // Expected tensor in Unity axes, built element by element from the FRD one.
            double[,] m = i.Matrix();
            int[] src = { 1, 2, 0 };          // Unity axis k is FRD axis src[k]
            double[] sign = { 1, -1, 1 };     // ... with this sign
            Matrix4x4 rot = Matrix4x4.Rotate(r);
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                {
                    double expected = sign[a] * sign[b] * m[src[a], src[b]];
                    double got = rot[a, 0] * p.x * rot[b, 0] + rot[a, 1] * p.y * rot[b, 1] + rot[a, 2] * p.z * rot[b, 2];
                    Assert.AreEqual(expected, got, 1e-5, $"element {a},{b}");
                }
        }

        [TestCase("x500")]
        [TestCase("quadplane")]
        public void ShippedSpecs_AreValid(string name)
        {
            var spec = Shipped(name);
            Assert.IsEmpty(spec.Problems());
            spec.Summary(out var warnings);
            Assert.IsEmpty(warnings, string.Join("; ", warnings));
        }

        [Test]
        public void HoverThrottle_X500_MatchesQuadraticModelWithIdle()
        {
            // 2 kg over 4 rotors of 8.54858 N, speed^2 thrust, 15% idle speed.
            float speed = Mathf.Sqrt(2f * VehicleSpec.Gravity / 4f / 8.54858f);
            Assert.AreEqual((speed - 0.15f) / 0.85f, Shipped("x500").HoverThrottle(), 1e-4f);
        }

        [Test]
        public void Problems_FlagsBadEntries()
        {
            var spec = Shipped("quadplane");
            spec.rotors[1].channel = spec.rotors[0].channel;
            spec.rotors[2].direction = "clockwise";
            spec.controlSurfaces[0].type = "aileron";
            spec.massProperties.mass = 0f;
            var p = spec.Problems();
            Assert.IsTrue(p.Any(s => s.Contains("also used by")), string.Join("; ", p));
            Assert.IsTrue(p.Any(s => s.Contains("direction")));
            Assert.IsTrue(p.Any(s => s.Contains("unknown type")));
            Assert.IsTrue(p.Any(s => s.Contains("mass")));
        }

        [Test]
        public void MissingOptionalFields_TakeDefaults()
        {
            var spec = VehicleSpec.Parse(@"{ ""name"": ""t"", ""rotors"": [ { ""name"": ""r"", ""position"": [0,0,0], ""direction"": ""ccw"", ""maxThrust"": 10 } ] }");
            var r = spec.rotors[0];
            Assert.AreEqual(new[] { 0f, 0f, -1f }, r.axis);
            Assert.AreEqual(0.016f, r.momentRatio);
            Assert.AreEqual(0.05f, r.timeConstant);
        }

        [Test]
        public void ThrustTable_LoadsAndInverts()
        {
            string dir = Path.Combine(Path.GetTempPath(), "vehicle_spec_test");
            Directory.CreateDirectory(dir);
            File.WriteAllText(Path.Combine(dir, "motor.csv"), "throttle,thrust_n\n0,0\n0.5,6\n1.0,20\n");
            var r = new RotorSpec { name = "r", thrustTable = "motor.csv" };
            Assert.AreEqual(20f, r.MaxThrust(dir));
            Assert.AreEqual(0.5f, r.ThrottleFor(6f, dir), 1e-5f);
            Assert.AreEqual(0.75f, r.ThrottleFor(13f, dir), 1e-5f);
            Assert.IsNaN(r.ThrottleFor(25f, dir));
        }

        [Test]
        public void Builder_MakesRotorsSurfacesAndMassFromSpec()
        {
            var spec = Shipped("quadplane");
            var go = VehicleBuilder.Build(spec, Vector3.zero);
            try
            {
                var rb = go.GetComponent<Rigidbody>();
                Assert.AreEqual(6.5f, rb.mass);
                Assert.AreEqual(5, go.GetComponentsInChildren<Rotor>().Length);
                var pusher = go.GetComponentsInChildren<Rotor>().Single(r => r.channel == 4);
                Assert.AreEqual(new Vector3(0, 0, -0.62f), pusher.transform.localPosition);
                Assert.AreEqual(1f, pusher.transform.up.z, 1e-5f);
                var left = go.GetComponentsInChildren<ControlSurface>().Single(s => s.channel == 5);
                Assert.AreEqual(new Vector3(-0.5f, 0, 0), left.torque);
                Assert.IsNotNull(go.GetComponent<Wing>());
                Assert.IsNotNull(go.GetComponent<VehicleActuators>());
            }
            finally { Object.DestroyImmediate(go); }
        }
    }
}
