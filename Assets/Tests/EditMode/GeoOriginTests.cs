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
        public void MagneticField_NedToUnity()
        {
            geo.magneticFieldNED = new Vector3(0.2f, 0.01f, 0.45f);
            // Unity (east, up, north) = (NED y, -NED z, NED x)
            Assert.AreEqual(new Vector3(0.01f, -0.45f, 0.2f), geo.MagneticFieldUnity);
        }
    }
}
