using System;
using NUnit.Framework;
using UnityEngine;

namespace DroneSim.Tests
{
    public class Vlp16ModelTests
    {
        [Test]
        public void Elevation_RunsFromMinus15To15InTwoDegreeSteps()
        {
            Assert.AreEqual(-15f, Vlp16Model.ElevationDeg(0));
            Assert.AreEqual(-13f, Vlp16Model.ElevationDeg(1));
            Assert.AreEqual(15f, Vlp16Model.ElevationDeg(Vlp16Model.Rings - 1));
        }

        [Test]
        public void ColumnsPerRevolution_FollowTheFiringRate()
        {
            // One firing sequence every 55.296 us: 0.2 degrees per column at 10 Hz.
            Assert.AreEqual(1808, Vlp16Model.ColumnsPerRevolution(10f));
            Assert.AreEqual(3617, Vlp16Model.ColumnsPerRevolution(5f));
            Assert.AreEqual(904, Vlp16Model.ColumnsPerRevolution(20f));
        }

        [Test]
        public void FiringOrder_AlternatesBetweenTheLowerAndUpperLasers()
        {
            // The sensor fires -15, +1, -13, +3, ... degrees: rings 0, 8, 1, 9, ...
            Assert.AreEqual(0, Vlp16Model.FiringOrder(0));
            Assert.AreEqual(1, Vlp16Model.FiringOrder(8));
            Assert.AreEqual(2, Vlp16Model.FiringOrder(1));
            Assert.AreEqual(14, Vlp16Model.FiringOrder(7));
            Assert.AreEqual(15, Vlp16Model.FiringOrder(15));

            var seen = new bool[Vlp16Model.Rings];
            for (int ring = 0; ring < Vlp16Model.Rings; ring++) seen[Vlp16Model.FiringOrder(ring)] = true;
            CollectionAssert.DoesNotContain(seen, false, "every slot of the firing sequence is used once");
        }

        [Test]
        public void PointTime_CountsFiringSequencesAndLasersFromTheStartOfTheRevolution()
        {
            Assert.AreEqual(0f, Vlp16Model.PointTime(0, 0));
            Assert.AreEqual(2.304e-6f, Vlp16Model.PointTime(0, 8), 1e-10f);
            Assert.AreEqual(55.296e-6f, Vlp16Model.PointTime(1, 0), 1e-10f);
            // The last laser of a 10 Hz revolution fires just under 0.1 s after the first.
            float last = Vlp16Model.PointTime(Vlp16Model.ColumnsPerRevolution(10f) - 1, 15);
            Assert.Less(last, 0.1f);
            Assert.Greater(last, 0.0999f);
        }

        [Test]
        public void Direction_AzimuthZeroIsForward_AndGrowsClockwiseToTheRight()
        {
            // Ring 8 is the first laser above the horizon, at +1 degree.
            Vector3 ahead = Vlp16Model.Direction(8, 0f);
            Assert.AreEqual(0f, ahead.x, 1e-6f);
            Assert.AreEqual(Mathf.Sin(Mathf.Deg2Rad), ahead.y, 1e-6f);
            Assert.AreEqual(Mathf.Cos(Mathf.Deg2Rad), ahead.z, 1e-6f);

            Vector3 right = Vlp16Model.Direction(8, 90f);
            Assert.Greater(right.x, 0.99f);
            Assert.AreEqual(0f, right.z, 1e-6f);
        }

        [Test]
        public void Direction_IsAUnitVector_PointingDownForTheLowRings()
        {
            for (int ring = 0; ring < Vlp16Model.Rings; ring++)
                Assert.AreEqual(1f, Vlp16Model.Direction(ring, 37f).magnitude, 1e-5f);
            Assert.AreEqual(-Mathf.Sin(15f * Mathf.Deg2Rad), Vlp16Model.Direction(0, 0f).y, 1e-6f);
            Assert.AreEqual(Mathf.Sin(15f * Mathf.Deg2Rad), Vlp16Model.Direction(15, 0f).y, 1e-6f);
        }

        [Test]
        public void WritePoint_PacksRosAxesAndFieldsAtTheirOffsets()
        {
            var buffer = new byte[2 * Vlp16Model.PointStep];
            // Unity: 1 right, 2 up, 3 forward. ROS: 3 forward, -1 left, 2 up.
            Vlp16Model.WritePoint(buffer, Vlp16Model.PointStep, new Vector3(1f, 2f, 3f), 100f, 9, 0.25f);

            int o = Vlp16Model.PointStep;
            Assert.AreEqual(3f, BitConverter.ToSingle(buffer, o + 0));
            Assert.AreEqual(-1f, BitConverter.ToSingle(buffer, o + 4));
            Assert.AreEqual(2f, BitConverter.ToSingle(buffer, o + 8));
            Assert.AreEqual(100f, BitConverter.ToSingle(buffer, o + 12));
            Assert.AreEqual(9, BitConverter.ToUInt16(buffer, o + 16));
            Assert.AreEqual(0.25f, BitConverter.ToSingle(buffer, o + 18));
            // The point before it is untouched.
            for (int i = 0; i < o; i++) Assert.AreEqual(0, buffer[i]);
        }

        [Test]
        public void PointStep_IsFourFloatsAUShortAndAFloat()
        {
            Assert.AreEqual(4 * 4 + 2 + 4, Vlp16Model.PointStep);
        }
    }
}
