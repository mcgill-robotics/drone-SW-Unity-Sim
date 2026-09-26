using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    public class GeoOriginTests
    {
        GameObject go;
        GeoOrigin geo;

        [SetUp]
        public void SetUp()
        {
            go = new GameObject("GeoOrigin");
            geo = go.AddComponent<GeoOrigin>();
        }

        [TearDown]
        public void TearDown() => Object.DestroyImmediate(go);

        [Test]
        public void North_IncreasesLatitude_East_IncreasesLongitude_Up_IncreasesAltitude()
        {
            geo.ToGeodetic(new Vector3(0, 0, 1000), out double lat, out double lon, out double alt);
            Assert.AreEqual(geo.latitude + 1000.0 / 6378137.0 * 180.0 / System.Math.PI, lat, 1e-6);
            Assert.AreEqual(geo.longitude, lon, 1e-9);

            geo.ToGeodetic(new Vector3(1000, 50, 0), out lat, out lon, out alt);
            Assert.Greater(lon, geo.longitude);
            Assert.AreEqual(geo.altitude + 50, alt, 1e-3);
        }

        [Test]
        public void FromGeodetic_InvertsToGeodetic_WithoutAwake()
        {
            // Edit-mode AddComponent does not run Awake, so this also covers the lazy cos(lat).
            var p = new Vector3(812.5f, 37.25f, -440f);
            geo.ToGeodetic(p, out double lat, out double lon, out double alt);
            Vector3 back = geo.FromGeodetic(lat, lon, alt);
            Assert.Less(Vector3.Distance(p, back), 0.01f, $"round trip gave {back}");
        }

        [Test]
        public void Px4HomeEnvironment_MovesOrigin()
        {
            try
            {
                System.Environment.SetEnvironmentVariable("PX4_HOME_LAT", "45.5048");
                System.Environment.SetEnvironmentVariable("PX4_HOME_LON", "-73.5772");
                System.Environment.SetEnvironmentVariable("PX4_HOME_ALT", "30");
                geo.ApplyEnvironmentOverrides();
                geo.ToGeodetic(Vector3.zero, out double lat, out double lon, out double alt);
                Assert.AreEqual(45.5048, lat, 1e-9);
                Assert.AreEqual(-73.5772, lon, 1e-9);
                Assert.AreEqual(30.0, alt, 1e-9);
            }
            finally
            {
                System.Environment.SetEnvironmentVariable("PX4_HOME_LAT", null);
                System.Environment.SetEnvironmentVariable("PX4_HOME_LON", null);
                System.Environment.SetEnvironmentVariable("PX4_HOME_ALT", null);
            }
        }

        [Test]
        public void NedToUnity_IsEastUpNorth()
        {
            Assert.AreEqual(new Vector3(0.01f, -0.45f, 0.2f), GeoOrigin.NedToUnity(new Vector3(0.2f, 0.01f, 0.45f)));
        }

        [Test]
        public void MagneticField_FollowsTheOrigin()
        {
            // Montreal: west declination, steep inclination. Values from PX4's own table.
            geo.latitude = 45.5048;
            geo.longitude = -73.5772;
            Vector3 ned = geo.MagneticFieldNed;
            Assert.Less(ned.y, 0f, "east component should be negative for west declination");
            Assert.Greater(ned.z, 0f, "field points down in the northern hemisphere");
            Assert.AreEqual(WorldMagneticModel.StrengthGauss(45.5048, -73.5772), ned.magnitude, 1e-4f);
        }
    }
}
