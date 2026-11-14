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

        const string LidarJson = @"{ ""lidars"": [ {
            ""name"": ""Lidar"", ""topic"": ""/points"", ""frameId"": ""lidar_frame"",
            ""rotationHz"": 20, ""minRange"": 1, ""maxRange"": 80, ""rangeNoiseStdDev"": 0.03 } ] }";

        static LidarConfig ValidLidar() => SimConfig.Parse(LidarJson).FindLidar("Lidar");

        [Test]
        public void Parse_ReadsLidarEntries()
        {
            var lidar = ValidLidar();
            Assert.IsNotNull(lidar);
            Assert.AreEqual("/points", lidar.topic);
            Assert.AreEqual("lidar_frame", lidar.frameId);
            Assert.AreEqual(20f, lidar.rotationHz);
            Assert.AreEqual(1f, lidar.minRange);
            Assert.AreEqual(80f, lidar.maxRange);
            Assert.AreEqual(0.03f, lidar.rangeNoiseStdDev);
            Assert.IsNull(lidar.Problem());
        }

        [Test]
        public void Parse_LidarEntryWithOnlyNames_GetsTheSensorDefaults()
        {
            var lidar = SimConfig.Parse(@"{ ""lidars"": [ { ""name"": ""Lidar"", ""topic"": ""/points"", ""frameId"": ""lidar_frame"" } ] }").FindLidar("Lidar");
            Assert.AreEqual(10f, lidar.rotationHz);
            Assert.AreEqual(100f, lidar.maxRange);
            Assert.IsNull(lidar.Problem());
        }

        [Test]
        public void FindLidar_UnknownName_IsNull()
        {
            Assert.IsNull(SimConfig.Parse(LidarJson).FindLidar("RearLidar"));
            Assert.IsNull(SimConfig.Parse("{}").FindLidar("Lidar"));
        }

        [Test]
        public void LidarProblem_FlagsBadEntries()
        {
            var lidar = ValidLidar(); lidar.topic = ""; Assert.IsNotNull(lidar.Problem());
            lidar = ValidLidar(); lidar.frameId = ""; Assert.IsNotNull(lidar.Problem());
            lidar = ValidLidar(); lidar.rotationHz = 4; Assert.IsNotNull(lidar.Problem());
            lidar = ValidLidar(); lidar.rotationHz = 21; Assert.IsNotNull(lidar.Problem());
            lidar = ValidLidar(); lidar.minRange = -1; Assert.IsNotNull(lidar.Problem());
            lidar = ValidLidar(); lidar.maxRange = lidar.minRange; Assert.IsNotNull(lidar.Problem());
            lidar = ValidLidar(); lidar.rangeNoiseStdDev = -0.01f; Assert.IsNotNull(lidar.Problem());
        }

        [Test]
        public void ShippedConfig_HasUsableEntryForTheLidar()
        {
            var lidar = SimConfig.Load(SimConfig.DefaultPath).FindLidar("Lidar");
            Assert.IsNotNull(lidar);
            Assert.IsNull(lidar.Problem());
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
