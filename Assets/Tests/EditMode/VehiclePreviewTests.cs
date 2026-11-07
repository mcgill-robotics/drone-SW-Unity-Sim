using System;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using DroneSim.Editor;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

namespace DroneSim.Tests
{
    public class VehiclePreviewTests
    {
        VehicleLoader loader;
        string path;

        static GameObject Preview => Resources.FindObjectsOfTypeAll<Transform>()
            .SingleOrDefault(t => t.name == "Vehicle Preview")?.gameObject;

        [SetUp]
        public void SetUp()
        {
            VehiclePreview.Clear();
            loader = new GameObject("Preview Test Loader").AddComponent<VehicleLoader>();
            path = Path.Combine(Path.GetTempPath(), "vehicle_preview_" + Guid.NewGuid().ToString("N") + ".json");
        }

        [TearDown]
        public void TearDown()
        {
            VehiclePreview.Clear();
            if (loader != null) Object.DestroyImmediate(loader.gameObject);
            if (File.Exists(path)) File.Delete(path);
        }

        [Test]
        public void VehicleField_UsesDelayedInspectorEditing()
        {
            var field = typeof(VehicleLoader).GetField(nameof(VehicleLoader.vehicle));
            Assert.IsTrue(Attribute.IsDefined(field, typeof(DelayedAttribute)));
        }

        [Test]
        public void SelectionChange_ReplacesPreview_UnchangedSelectionKeepsIt()
        {
            VehiclePreview.Refresh(loader);
            var first = Preview;
            Assert.IsNotNull(first);
            VehiclePreview.Refresh(loader);
            Assert.AreSame(first, Preview);

            loader.vehicle = "quadplane";
            VehiclePreview.Refresh(loader);
            Assert.IsTrue(first == null);
            Assert.IsNotNull(Preview);
            Assert.IsNotNull(Preview.transform.Find("Pusher"));
        }

        [Test]
        public void ExternalSpecEdit_ReplacesPreviewWithoutAssetImport()
        {
            File.Copy(VehicleSpec.Resolve("x500"), path);
            loader.vehicle = path;
            VehiclePreview.Refresh(loader);
            var first = Preview;
            Assert.IsNotNull(first);

            var modified = File.GetLastWriteTimeUtc(path).AddSeconds(2);
            File.Copy(VehicleSpec.Resolve("quadplane"), path, true);
            File.SetLastWriteTimeUtc(path, modified);
            VehiclePreview.Refresh(loader);
            Assert.IsTrue(first == null);
            Assert.IsNotNull(Preview.transform.Find("Pusher"));
        }

        [Test]
        public void SimulationProblems_DoNotBlockPreview()
        {
            var spec = VehicleSpec.Load(VehicleSpec.Resolve("x500"));
            spec.massProperties.mass = 0f;
            spec.massProperties.inertia.ixx = 0f;
            spec.rotors[0].channel = -1;
            spec.rotors[0].thrustTable = path + ".csv";
            spec.px4.baseAirframe = "";
            File.WriteAllText(path, JsonUtility.ToJson(spec));
            loader.vehicle = path;

            VehiclePreview.Refresh(loader);

            Assert.IsNotNull(Preview);
            Assert.AreEqual(spec.visuals.Length + spec.rotors.Length, Preview.GetComponentsInChildren<MeshFilter>().Length);
            var problems = VehicleSpec.Load(path).Problems();
            foreach (string field in new[] { "massProperties.mass", "inertia", "channel", "thrustTable", "px4.baseAirframe" })
                Assert.IsTrue(problems.Any(p => p.Contains(field)), field);
        }

        [Test]
        public void InvalidSpec_RemovesPreview_AndRecoversWhenFixed()
        {
            File.Copy(VehicleSpec.Resolve("x500"), path);
            loader.vehicle = path;
            VehiclePreview.Refresh(loader);
            var first = Preview;
            var modified = File.GetLastWriteTimeUtc(path).AddSeconds(2);
            File.WriteAllText(path, "{}");
            File.SetLastWriteTimeUtc(path, modified);

            LogAssert.Expect(LogType.Warning, new Regex(@"\[Vehicle Preview\] no preview built:"));
            VehiclePreview.Refresh(loader);
            Assert.IsTrue(first == null);
            Assert.IsNull(Preview);
            VehiclePreview.Refresh(loader);
            LogAssert.NoUnexpectedReceived();

            File.Copy(VehicleSpec.Resolve("x500"), path, true);
            File.SetLastWriteTimeUtc(path, modified.AddSeconds(2));
            VehiclePreview.Refresh(loader);
            Assert.IsNotNull(Preview);
        }

        [Test]
        public void DisabledOrRemovedLoader_ClearsPreview()
        {
            VehiclePreview.Refresh(loader);
            loader.enabled = false;
            VehiclePreview.Refresh(loader);
            Assert.IsNull(Preview);

            loader.enabled = true;
            VehiclePreview.Refresh(loader);
            Assert.IsNotNull(Preview);
            Object.DestroyImmediate(loader.gameObject);
            VehiclePreview.Refresh(loader);
            Assert.IsNull(Preview);
        }

        [Test]
        public void PlayTransitionCallback_ClearsPreview_AndAllowsRecreationInEditMode()
        {
            VehiclePreview.Refresh(loader);
            var first = Preview;
            VehiclePreview.OnPlayModeChanged(PlayModeStateChange.ExitingEditMode);
            Assert.IsTrue(first == null);
            Assert.IsNull(Preview);

            VehiclePreview.OnPlayModeChanged(PlayModeStateChange.EnteredEditMode);
            VehiclePreview.Refresh(loader);
            Assert.IsNotNull(Preview);
            VehiclePreview.Clear();
            VehiclePreview.Clear();
            Assert.IsNull(Preview);
        }
    }
}
