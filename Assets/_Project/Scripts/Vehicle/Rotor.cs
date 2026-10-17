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
    /// A thrust table is measured against the throttle PX4 commands, idle included, so idleSpeed
    /// doesn't apply to it and the lag acts on its thrust and torque; a disarmed motor makes none
    /// whatever the table's first row says.
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
        [Tooltip("Fraction of full speed when armed at zero command (Gazebo's SIM_GZ_EC_MIN / MAX). Not used with a thrust table")]
        [Range(0f, 0.5f)] public float idleSpeed = 0f;
        [Tooltip("Runtime multiplier used by domain randomization")] public float efficiency = 1f;

        [Header("Thrust table (optional)")]
        [Tooltip("Throttle points 0..1 as PX4 commands them, increasing")] public float[] tableThrottle;
        [Tooltip("Thrust at each throttle point (N)")] public float[] tableThrust;
        [Tooltip("Reaction torque at each throttle point (N m); empty to use maxTorque / maxThrust")] public float[] tableTorque;

        public bool HasTable => tableThrottle != null && tableThrust != null && tableThrottle.Length >= 2 && tableThrust.Length == tableThrottle.Length;

        /// <summary>
        /// What PX4 is asking for, before the lag: speed as a fraction of full, after idle, or with a
        /// thrust table the throttle itself. 0 when disarmed.
        /// </summary>
        public float Commanded => commanded;
        /// <summary>Commanded after the spin-up lag.</summary>
        public float Throttle { get; private set; }
        public float Thrust { get; private set; }
        /// <summary>Reaction torque magnitude (N m); its sign comes from spinDirection.</summary>
        public float Torque { get; private set; }

        private Rigidbody rb;
        private float commanded;
        private bool running;
        private float tableThrustLagged, tableTorqueLagged;

        private void Awake() => rb = GetComponentInParent<Rigidbody>();

        public void Apply(float[] controls, bool armed)
        {
            running = armed && channel < controls.Length;
            float c = running ? Mathf.Clamp01(controls[channel]) : 0f;
            commanded = !running ? 0f : HasTable ? c : idleSpeed + (1f - idleSpeed) * c;
        }

        public void ResetActuator()
        {
            running = false;
            commanded = 0f; Throttle = 0f; Thrust = 0f; Torque = 0f;
            tableThrustLagged = 0f; tableTorqueLagged = 0f;
        }

        private void FixedUpdate()
        {
            if (rb == null) return;
            Step(Time.fixedDeltaTime);
            Vector3 axis = transform.up;
            rb.AddForceAtPosition(axis * Thrust, transform.position, ForceMode.Force);
            rb.AddTorque(axis * (Torque * spinDirection), ForceMode.Force);
        }

        /// <summary>Advances the spin-up lag by dt and sets Throttle, Thrust and Torque.</summary>
        public void Step(float dt)
        {
            float alpha = Mathf.Clamp01(dt / Mathf.Max(timeConstant, 1e-3f));
            Throttle = Mathf.Lerp(Throttle, commanded, alpha);

            float thrust, torque;
            if (HasTable)
            {
                float targetThrust = 0f, targetTorque = 0f;
                if (running)
                {
                    targetThrust = Interpolate(tableThrottle, tableThrust, commanded);
                    targetTorque = tableTorque != null && tableTorque.Length == tableThrottle.Length
                        ? Interpolate(tableThrottle, tableTorque, commanded)
                        : targetThrust * maxTorque / Mathf.Max(maxThrust, 1e-6f);
                }
                tableThrustLagged = Mathf.Lerp(tableThrustLagged, targetThrust, alpha);
                tableTorqueLagged = Mathf.Lerp(tableTorqueLagged, targetTorque, alpha);
                thrust = tableThrustLagged;
                torque = tableTorqueLagged;
            }
            else
            {
                float w2 = Throttle * Throttle;       // thrust ~ omega^2, omega ~ throttle
                thrust = maxThrust * w2;
                torque = maxTorque * w2;
            }
            Thrust = thrust * efficiency;
            Torque = torque;
        }

        /// <summary>Piecewise-linear lookup, clamped at both ends. xs must be increasing.</summary>
        public static float Interpolate(float[] xs, float[] ys, float x)
        {
            int n = xs.Length;
            if (x <= xs[0]) return ys[0];
            if (x >= xs[n - 1]) return ys[n - 1];
            int lo = 0, hi = n - 1;               // xs[lo] < x <= xs[hi]
            while (hi - lo > 1)
            {
                int mid = (lo + hi) / 2;
                if (xs[mid] < x) lo = mid; else hi = mid;
            }
            return Mathf.Lerp(ys[lo], ys[hi], (x - xs[lo]) / (xs[hi] - xs[lo]));
        }
    }
}
