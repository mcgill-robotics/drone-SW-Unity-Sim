using System;
using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Conversions between Unity's body axes (x right, y up, z forward, left-handed) and the FRD
    /// body frame (x forward, y right, z down) that vehicle specs and PX4 use.
    /// </summary>
    public static class Frames
    {
        /// <summary>FRD (x forward, y right, z down) to Unity (x right, y up, z forward).</summary>
        public static Vector3 FrdToUnity(Vector3 frd) => new Vector3(frd.y, -frd.z, frd.x);
        public static Vector3 FrdToUnity(float[] frd) => FrdToUnity(VehicleSpec.ToVector(frd));

        /// <summary>Unity body (x right, y up, z forward) to FRD (x forward, y right, z down). Inverse of FrdToUnity.</summary>
        public static Vector3 BodyToFrd(Vector3 v) => new Vector3(v.z, v.x, -v.y);
        /// <summary>Angular rate or torque is a pseudovector: axis swap then handedness flip.</summary>
        public static Vector3 RateToFrd(Vector3 w) => new Vector3(-w.z, -w.x, w.y);

        /// <summary>
        /// The inertia tensor in Unity's body axes, as Rigidbody wants it: principal moments and the
        /// rotation of the principal axes. A diagonal tensor maps straight across.
        /// </summary>
        public static void InertiaToUnity(InertiaSpec i, out Vector3 principal, out Quaternion rotation)
        {
            // Unity axes in terms of FRD: x = y_frd, y = -z_frd, z = x_frd.
            double[,] r = { { 0, 1, 0 }, { 0, 0, -1 }, { 1, 0, 0 } };
            double[,] m = i.Matrix();
            var u = new double[3, 3];
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                {
                    double s = 0;
                    for (int c = 0; c < 3; c++)
                        for (int d = 0; d < 3; d++) s += r[a, c] * m[c, d] * r[b, d];
                    u[a, b] = s;
                }

            if (u[0, 1] == 0 && u[0, 2] == 0 && u[1, 2] == 0)
            {
                principal = new Vector3((float)u[0, 0], (float)u[1, 1], (float)u[2, 2]);
                rotation = Quaternion.identity;
                return;
            }

            Eigen(u, out double[] values, out double[,] vectors);
            // Columns are the principal axes; make them a proper rotation.
            Vector3 c0 = new Vector3((float)vectors[0, 0], (float)vectors[1, 0], (float)vectors[2, 0]);
            Vector3 c1 = new Vector3((float)vectors[0, 1], (float)vectors[1, 1], (float)vectors[2, 1]);
            Vector3 c2 = Vector3.Cross(c0, c1);
            rotation = Quaternion.LookRotation(c2, c1);
            principal = new Vector3((float)values[0], (float)values[1], (float)values[2]);
            // LookRotation maps x to c1 x c2 = c0 for an orthonormal right-handed set, so the
            // moments stay paired with their axes.
        }

        /// <summary>Eigenvalues and eigenvectors (as columns) of a symmetric 3x3 matrix, by Jacobi rotations.</summary>
        public static void Eigen(double[,] matrix, out double[] values, out double[,] vectors)
        {
            var a = (double[,])matrix.Clone();
            var v = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
            for (int sweep = 0; sweep < 50; sweep++)
            {
                double off = Math.Abs(a[0, 1]) + Math.Abs(a[0, 2]) + Math.Abs(a[1, 2]);
                if (off < 1e-15) break;
                for (int p = 0; p < 2; p++)
                    for (int q = p + 1; q < 3; q++)
                    {
                        if (Math.Abs(a[p, q]) < 1e-18) continue;
                        double theta = (a[q, q] - a[p, p]) / (2 * a[p, q]);
                        double t = Math.Sign(theta == 0 ? 1 : theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1));
                        double c = 1 / Math.Sqrt(t * t + 1), s = t * c;
                        for (int k = 0; k < 3; k++)
                        {
                            double akp = a[k, p], akq = a[k, q];
                            a[k, p] = c * akp - s * akq;
                            a[k, q] = s * akp + c * akq;
                        }
                        for (int k = 0; k < 3; k++)
                        {
                            double apk = a[p, k], aqk = a[q, k];
                            a[p, k] = c * apk - s * aqk;
                            a[q, k] = s * apk + c * aqk;
                        }
                        for (int k = 0; k < 3; k++)
                        {
                            double vkp = v[k, p], vkq = v[k, q];
                            v[k, p] = c * vkp - s * vkq;
                            v[k, q] = s * vkp + c * vkq;
                        }
                    }
            }
            values = new[] { a[0, 0], a[1, 1], a[2, 2] };
            vectors = v;
        }
    }
}
