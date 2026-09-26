using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    /// <summary>
    /// Spot checks against the WMM at grid points and PX4's interpolation. Grid values are exact
    /// table entries, so these catch a mis-copied table or a transposed lookup.
    /// </summary>
    public class WorldMagneticModelTests
    {
        [Test]
        public void GridPoint_MatchesTable()
        {
            // Entries at lat 0, lon 0 in PX4 v1.17's geo_magnetic_tables.hpp: -724, -11228, 15645.
            Assert.AreEqual(-724 * 0.00545143529f, WorldMagneticModel.DeclinationDeg(0, 0), 1e-4f);
            Assert.AreEqual(-11228 * 0.00269892697f, WorldMagneticModel.InclinationDeg(0, 0), 1e-4f);
            Assert.AreEqual(15645 * 2.04183477f * 1e-5f, WorldMagneticModel.StrengthGauss(0, 0), 1e-5f);
            Assert.AreEqual(WorldMagneticModel.DeclinationDeg(0, 0), WorldMagneticModel.DeclinationDeg(0, 360), 1e-4f);
        }

        [Test]
        public void Zurich_IsPlausible()
        {
            // PX4's default SITL home. WMM: declination about +3 deg, inclination about 63 deg, 0.48 gauss.
            Assert.AreEqual(3f, WorldMagneticModel.DeclinationDeg(47.397742, 8.545594), 2f);
            Assert.AreEqual(63.5f, WorldMagneticModel.InclinationDeg(47.397742, 8.545594), 2f);
            Assert.AreEqual(0.48f, WorldMagneticModel.StrengthGauss(47.397742, 8.545594), 0.03f);
        }

        [Test]
        public void Montreal_IsPlausible()
        {
            // WMM: declination about -14 deg, inclination about 70 deg, 0.53 gauss.
            Assert.AreEqual(-14f, WorldMagneticModel.DeclinationDeg(45.5048, -73.5772), 2.5f);
            Assert.AreEqual(70f, WorldMagneticModel.InclinationDeg(45.5048, -73.5772), 2.5f);
            Assert.AreEqual(0.53f, WorldMagneticModel.StrengthGauss(45.5048, -73.5772), 0.03f);
        }

        [Test]
        public void Poles_DoNotThrow()
        {
            Assert.DoesNotThrow(() => WorldMagneticModel.FieldNed(90, 180));
            Assert.DoesNotThrow(() => WorldMagneticModel.FieldNed(-90, -180));
        }
    }
}
