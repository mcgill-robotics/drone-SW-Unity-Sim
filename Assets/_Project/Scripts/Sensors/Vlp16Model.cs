using System;
using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// The geometry and point layout of a Velodyne VLP-16 (Puck), with no scene or ROS types so it
    /// can be unit tested: 16 lasers from -15 to +15 degrees in 2 degree steps, spinning clockwise
    /// seen from above, each firing sequence of all 16 taking 55.296 microseconds.
    ///
    /// Points are packed the way the velodyne ROS 2 driver publishes them: x, y, z, intensity
    /// (float32), ring (uint16), time (float32), 22 bytes, little-endian, in ROS axes (x forward,
    /// y left, z up).
    /// </summary>
    public static class Vlp16Model
    {
        // The numbers below are from the VLP-16 user manual. The firing times are the ones the
        // velodyne ROS driver uses to compute each point's time.
        public const int Rings = 16;
        public const float LowestElevationDeg = -15f;
        public const float RingSpacingDeg = 2f;

        /// <summary>Seconds for one firing sequence: all 16 lasers and the recharge.</summary>
        public const double FiringCycleSeconds = 55.296e-6;
        /// <summary>Seconds between one laser and the next inside a firing sequence.</summary>
        public const double LaserSpacingSeconds = 2.304e-6;

        public const float MinRotationHz = 5f, MaxRotationHz = 20f;

        // Byte offsets inside one point. They must match the PointField list in LidarPublisher,
        // which is what tells ROS how to read the bytes.
        public const int XOffset = 0, YOffset = 4, ZOffset = 8, IntensityOffset = 12, RingOffset = 16, TimeOffset = 18;
        public const int PointStep = 22;

        /// <summary>Elevation of a ring. Ring 0 is the lowest laser (-15 degrees), ring 15 the highest (+15).</summary>
        public static float ElevationDeg(int ring) => LowestElevationDeg + ring * RingSpacingDeg;

        /// <summary>
        /// Position of a ring in the firing order. The lasers don't fire bottom to top: the sensor
        /// alternates between the lower and the upper half (-15, +1, -13, +3, ... degrees), so rings
        /// 0 to 7 fire on the even slots and rings 8 to 15 on the odd ones.
        /// </summary>
        public static int FiringOrder(int ring) => ring < Rings / 2 ? 2 * ring : 2 * (ring - Rings / 2) + 1;

        /// <summary>
        /// Seconds from the start of a revolution to the moment a laser fires, for the firing
        /// sequence (column) it belongs to. This is the point's time field.
        /// </summary>
        public static float PointTime(int column, int ring) =>
            (float)(column * FiringCycleSeconds + FiringOrder(ring) * LaserSpacingSeconds);

        /// <summary>
        /// Firing sequences in one revolution. The lasers fire at a fixed rate, so a slower spin
        /// gives a finer azimuth step: 0.1 degrees at 5 Hz, 0.2 at 10 Hz, 0.4 at 20 Hz.
        /// </summary>
        // At 10 Hz: 1 / (10 x 55.296e-6) = 1808.4, so 1808 firing sequences, 360 / 1808 = 0.2 degrees apart.
        public static int ColumnsPerRevolution(float rotationHz) =>
            Mathf.RoundToInt((float)(1.0 / (rotationHz * FiringCycleSeconds)));

        /// <summary>
        /// Unit direction of a laser in the sensor's Unity axes (x right, y up, z forward).
        /// Azimuth 0 is straight ahead and grows clockwise seen from above, towards the right.
        /// </summary>
        public static Vector3 Direction(int ring, float azimuthDeg)
        {
            // Spherical to Cartesian: the elevation splits the unit vector into a vertical part
            // (sin) and a horizontal part (cos), and the azimuth turns the horizontal part from
            // forward (+z) towards the right (+x).
            float el = ElevationDeg(ring) * Mathf.Deg2Rad, az = azimuthDeg * Mathf.Deg2Rad;
            float horizontal = Mathf.Cos(el);
            return new Vector3(horizontal * Mathf.Sin(az), Mathf.Sin(el), horizontal * Mathf.Cos(az));
        }

        /// <summary>
        /// Writes one point at offset. The point is in the sensor's Unity axes and is stored in ROS
        /// axes: x = Unity z, y = -Unity x, z = Unity y (the same mapping as the ROS connector's FLU).
        /// </summary>
        public static void WritePoint(byte[] buffer, int offset, Vector3 unityLocalPoint, float intensity, int ring, float time)
        {
            WriteFloat(buffer, offset + XOffset, unityLocalPoint.z);
            WriteFloat(buffer, offset + YOffset, -unityLocalPoint.x);
            WriteFloat(buffer, offset + ZOffset, unityLocalPoint.y);
            WriteFloat(buffer, offset + IntensityOffset, intensity);
            // uint16, little-endian: low byte first.
            buffer[offset + RingOffset] = (byte)ring;
            buffer[offset + RingOffset + 1] = (byte)(ring >> 8);
            WriteFloat(buffer, offset + TimeOffset, time);
        }

        // Writes a float32 little-endian, byte by byte. BitConverter.GetBytes would do the same but
        // allocates a new array for every value: four per point, over a million a second here.
        static void WriteFloat(byte[] buffer, int offset, float value)
        {
            int bits = BitConverter.SingleToInt32Bits(value);
            buffer[offset] = (byte)bits;
            buffer[offset + 1] = (byte)(bits >> 8);
            buffer[offset + 2] = (byte)(bits >> 16);
            buffer[offset + 3] = (byte)(bits >> 24);
        }
    }
}
