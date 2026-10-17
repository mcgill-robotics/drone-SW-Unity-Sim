using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// One propeller. Thrust along the local +Y axis of this transform, applied at its position,
    /// with a first-order spin-up lag and a reaction torque about the axis.
    /// PX4's pwm_out_sim sends non-reversible motors as 0..1, so the channel value is the throttle,
    /// raised by idleSpeed when armed the way Gazebo's motor model does.
    ///
    /// Thrust is maxThrust x speed^2, or read from a thrust table (throttle -> newtons) when one is
    /// set, for motors with test-stand or datasheet data. Reaction torque follows the torque table
    /// if there is one, else thrust x maxTorque / maxThrust.
    ///
    /// Spin direction follows PX4's CA_ROTORn_KM sign: +1 for a CCW rotor (seen from above), whose
    /// reaction torque yaws the body nose-right, which is +Y torque in Unity's left-handed axes.
    /// </summary>
    public class Rotor : MonoBehaviour, IActuator
    {
        [Header("PX4 channel")]
        public int channel = 0;

        [Header("Motor")]
        [Tooltip("Thrust at full throttle (N)")] public float maxThrust = 25f;
        [Tooltip("Reaction torque at full throttle (N m)")] public float maxTorque = 0.3f;
        [Tooltip("+1 for CCW seen from above (PX4 CA_ROTORn_KM > 0), -1 for CW")] public int spinDirection = 1;
        [Tooltip("Spin-up time constant (s)")] public float timeConstant = 0.05f;
        [Tooltip("Fraction of full speed when armed at zero command (Gazebo's SIM_GZ_EC_MIN / MAX)")]
        [Range(0f, 0.5f)] public float idleSpeed = 0f;
        [Tooltip("Runtime multiplier used by domain randomization")] public float efficiency = 1f;

        [Header("Thrust table (optional)")]
        [Tooltip("Throttle points 0..1, increasing")] public float[] tableThrottle;
        [Tooltip("Thrust at each throttle point (N)")] public float[] tableThrust;
        [Tooltip("Reaction torque at each throttle point (N m); empty to use maxTorque / maxThrust")] public float[] tableTorque;

        public bool HasTable => tableThrottle != null && tableThrust != null && tableThrottle.Length >= 2 && tableThrust.Length == tableThrottle.Length;

        /// <summary>Speed PX4 is asking for as a fraction of full, after idle, before the spin-up lag.</summary>
        public float Commanded => commanded;
        public float Throttle { get; private set; }
        public float Thrust { get; private set; }

        private Rigidbody rb;
        private float commanded;

        private void Awake() => rb = GetComponentInParent<Rigidbody>();

        public void Apply(float[] controls, bool armed)
        {
            commanded = armed && channel < controls.Length ? idleSpeed + (1f - idleSpeed) * Mathf.Clamp01(controls[channel]) : 0f;
        }

        public void ResetActuator() { commanded = 0f; Throttle = 0f; Thrust = 0f; }

        private void FixedUpdate()
        {
            if (rb == null) return;
            float alpha = Time.fixedDeltaTime / Mathf.Max(timeConstant, 1e-3f);
            Throttle = Mathf.Lerp(Throttle, commanded, Mathf.Clamp01(alpha));

            float thrust, torque;
            if (HasTable)
            {
                thrust = Interpolate(tableThrottle, tableThrust, Throttle);
                torque = tableTorque != null && tableTorque.Length == tableThrottle.Length
                    ? Interpolate(tableThrottle, tableTorque, Throttle)
                    : thrust * maxTorque / Mathf.Max(maxThrust, 1e-6f);
            }
            else
            {
                float w2 = Throttle * Throttle;       // thrust ~ omega^2, omega ~ throttle
                thrust = maxThrust * w2;
                torque = maxTorque * w2;
            }
            Thrust = thrust * efficiency;
            Vector3 axis = transform.up;
            rb.AddForceAtPosition(axis * Thrust, transform.position, ForceMode.Force);
            rb.AddTorque(axis * (torque * spinDirection), ForceMode.Force);
        }

        /// <summary>Piecewise-linear lookup, clamped at both ends. xs must be increasing.</summary>
        public static float Interpolate(float[] xs, float[] ys, float x)
        {
            if (x <= xs[0]) return ys[0];
            for (int i = 1; i < xs.Length; i++)
                if (x <= xs[i]) return Mathf.Lerp(ys[i - 1], ys[i], (x - xs[i - 1]) / (xs[i] - xs[i - 1]));
            return ys[ys.Length - 1];
        }
    }
}
