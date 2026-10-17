using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Whole-airframe aerodynamics: lift with a linear alpha curve and stall cutoff, parasitic and
    /// induced drag, sideforce, and rate damping. Forces act at the aerodynamic centre offset.
    /// Deliberately simple; PX4 does the controlling, this only has to be plausible.
    /// </summary>
    public class Wing : MonoBehaviour
    {
        /// <summary>ISA sea level, kg/m^3.</summary>
        public const float SeaLevelDensity = 1.225f;

        [Header("Air")]
        public float airDensity = SeaLevelDensity;

        [Header("Geometry")]
        [Tooltip("Wing area (m^2)")] public float area = 0.45f;
        [Tooltip("Wing span (m)")] public float span = 2.0f;
        [Tooltip("Mean chord (m)")] public float chord = 0.22f;
        [Tooltip("Aerodynamic centre in body coordinates (Unity axes: x right, y up, z forward)")]
        public Vector3 aeroCenter = new Vector3(0f, 0f, 0.02f);

        [Header("Lift")]
        [Tooltip("Lift coefficient at zero alpha")] public float cl0 = 0.3f;
        [Tooltip("Lift slope per radian")] public float clAlpha = 5.0f;
        [Tooltip("Stall angle (deg)")] public float stallAlphaDeg = 15f;
        [Tooltip("Max lift coefficient past stall")] public float clMaxPostStall = 0.6f;

        [Header("Drag")]
        public float cd0 = 0.04f;
        [Tooltip("Oswald efficiency")] public float oswald = 0.8f;
        [Tooltip("Sideforce coefficient per radian of sideslip")] public float cyBeta = -0.3f;

        [Header("Static stability (per radian)")]
        public float cmAlpha = -0.6f;
        public float cnBeta = 0.08f;
        public float clBeta = -0.05f;

        [Header("Damping (per rad/s, normalised)")]
        public float clp = -0.5f;
        public float cmq = -8f;
        public float cnr = -0.1f;

        public float Alpha { get; private set; }
        public float Beta { get; private set; }
        public float Airspeed { get; private set; }

        private Rigidbody rb;

        private void Awake() => rb = GetComponentInParent<Rigidbody>();

        private void FixedUpdate()
        {
            if (rb == null) return;

            Vector3 acWorld = rb.transform.TransformPoint(aeroCenter);
            Vector3 airVelWorld = rb.GetPointVelocity(acWorld) - Wind.At(acWorld);
            Vector3 v = rb.transform.InverseTransformDirection(airVelWorld); // body: x right, y up, z forward
            float V = v.magnitude;
            Airspeed = V;
            if (V < 0.5f) return;

            Alpha = Mathf.Atan2(-v.y, v.z);            // nose above the relative wind => positive alpha
            Beta = Mathf.Asin(Mathf.Clamp(v.x / V, -1f, 1f));
            float q = 0.5f * airDensity * V * V;

            float stall = stallAlphaDeg * Mathf.Deg2Rad;
            float cl;
            if (Mathf.Abs(Alpha) < stall) cl = cl0 + clAlpha * Alpha;
            else cl = Mathf.Sign(Alpha) * clMaxPostStall * Mathf.Cos(Alpha);

            float ar = span * span / area;
            float cd = cd0 + cl * cl / (Mathf.PI * oswald * ar);
            float cy = cyBeta * Beta;

            Vector3 dragDir = -airVelWorld / V;
            Vector3 liftDir = Vector3.Cross(Vector3.Cross(dragDir, rb.transform.up), dragDir).normalized;
            Vector3 sideDir = rb.transform.right;

            Vector3 force = liftDir * (q * area * cl) + dragDir * (q * area * cd) + sideDir * (q * area * cy);
            rb.AddForceAtPosition(force, acWorld, ForceMode.Force);

            // Body rates in aero convention (p right-wing-down, q nose-up, r nose-right).
            // Unity: +z rotation lifts the right wing, +x rotation drops the nose, +y rotation yaws right.
            Vector3 w = rb.transform.InverseTransformDirection(rb.angularVelocity);
            float p = -w.z, qRate = -w.x, r = w.y;
            float b2v = span / (2f * V), c2v = chord / (2f * V);

            float cRoll = clBeta * Beta + clp * p * b2v;
            float cPitch = cmAlpha * Alpha + cmq * qRate * c2v;
            float cYaw = cnBeta * Beta + cnr * r * b2v;

            // Right-handed aero convention -> Unity left-handed axes (roll about +Z, pitch about +X, yaw about +Y).
            Vector3 torque =
                rb.transform.forward * (-q * area * span * cRoll) +
                rb.transform.right * (-q * area * chord * cPitch) +
                rb.transform.up * (q * area * span * cYaw);
            rb.AddTorque(torque, ForceMode.Force);
        }
    }
}
