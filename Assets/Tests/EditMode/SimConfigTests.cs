using NUnit.Framework;

namespace DroneSim.Tests
{
    public class SimConfigTests
    {
        const string Json = @"{ ""cameras"": [ {
            ""name"": ""ForwardCamera"", ""topic"": ""/cam/image_raw"", ""frameId"": ""cam_optical_frame"",
            ""width"": 640, ""height"": 480, ""rate"": 30, ""verticalFov"": 50, ""format"": ""raw"", ""jpegQuality"": 90 } ] }";

        static CameraConfig Valid() => SimConfig.Parse(Json).FindCamera("ForwardCamera");

        [Test]
        public void Parse_ReadsCameraEntries()
        {
            var cam = Valid();
            Assert.IsNotNull(cam);
            Assert.AreEqual("/cam/image_raw", cam.topic);
            Assert.AreEqual("cam_optical_frame", cam.frameId);
            Assert.AreEqual(640, cam.width);
            Assert.AreEqual(480, cam.height);
            Assert.AreEqual(30f, cam.rate);
            Assert.AreEqual(50f, cam.verticalFov);
            Assert.IsFalse(cam.Jpeg);
            Assert.AreEqual(90, cam.jpegQuality);
            Assert.IsNull(cam.Problem());
        }

        [Test]
        public void FindCamera_UnknownName_IsNull()
        {
            Assert.IsNull(SimConfig.Parse(Json).FindCamera("DownCamera"));
            Assert.IsNull(SimConfig.Parse("{}").FindCamera("ForwardCamera"));
        }

        [Test]
        public void Problem_FlagsBadEntries()
        {
            var cam = Valid(); cam.topic = ""; Assert.IsNotNull(cam.Problem());
            cam = Valid(); cam.width = 0; Assert.IsNotNull(cam.Problem());
            cam = Valid(); cam.rate = 0; Assert.IsNotNull(cam.Problem());
            cam = Valid(); cam.verticalFov = 180; Assert.IsNotNull(cam.Problem());
            cam = Valid(); cam.format = "png"; Assert.IsNotNull(cam.Problem());
            cam = Valid(); cam.format = "jpeg"; cam.jpegQuality = 0; Assert.IsNotNull(cam.Problem());
        }

        [Test]
        public void ShippedConfig_HasUsableEntryForEachSceneCamera()
        {
            var config = SimConfig.Load(SimConfig.DefaultPath);
            foreach (string name in new[] { "ForwardCamera", "DownCamera" })
            {
                var cam = config.FindCamera(name);
                Assert.IsNotNull(cam, name);
                Assert.IsNull(cam.Problem(), name);
            }
        }
    }
}
