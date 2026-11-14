using System.Collections.Generic;
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
        [Tooltip("Spin-up and spin-down time constant (s), unless timeConstantUp or timeConstantDown is set")] public float timeConstant = 0.05f;
        [Tooltip("Spin-up time constant (s). 0 uses timeConstant")] public float timeConstantUp = 0f;
        [Tooltip("Spin-down time constant (s). 0 uses timeConstant")] public float timeConstantDown = 0f;
        [Tooltip("Dead time between a command arriving and the motor responding to it (s). 0 for none")] public float latency = 0f;
        [Tooltip("Fraction of full speed when armed at zero command (Gazebo's SIM_GZ_EC_MIN / MAX). Not used with a thrust table")]
        [Range(0f, 0.5f)] public float idleSpeed = 0f;
        [Tooltip("Runtime multiplier used by domain randomization")] public float efficiency = 1f;

        [Header("Rotor drag (optional)")]
        [Tooltip("Rotor speed at full throttle (rad/s). Rotor drag needs it; 0 = unset")] public float maxSpeed = 0f;
        [Tooltip("Drag from air flowing across the disc, per m/s of airflow and rad/s of rotor speed (kg/rad); PX4's rotorDragCoefficient. 0 = off")]
        public float dragCoefficient = 0f;

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
        /// <summary>Rotor speed as a fraction of full speed. Thrust goes as speed squared, so this is sqrt(thrust / maxThrust).</summary>
        public float SpeedFraction { get; private set; }

        private Rigidbody rb;
        private float commanded;
        private bool running;
        private float tableThrustLagged, tableTorqueLagged;

        // Commands that have arrived but are still inside the dead time, oldest first. The clock is a
        // double: a float loses millisecond resolution after a few hours of simulated time.
        private readonly Queue<(double time, float value)> pending = new Queue<(double time, float value)>();
        private double clock;

        /// <summary>
        /// Most commands kept waiting. latency is at most 0.5 s, so even a 1 ms step needs only 500; the
        /// cap only matters when this rotor stops stepping (disabled) while Apply keeps arriving.
        /// </summary>
        public const int MaxPendingCommands = 600;
        /// <summary>Commands received but still inside the dead time.</summary>
        public int PendingCommands => pending.Count;

        private void Awake() => rb = GetComponentInParent<Rigidbody>();

        public void Apply(float[] controls, bool armed)
        {
            running = armed && channel < controls.Length;
            float c = running ? Mathf.Clamp01(controls[channel]) : 0f;
            float target = !running ? 0f : HasTable ? c : idleSpeed + (1f - idleSpeed) * c;

            if (!running)
            {
                // A disarmed motor is cut at once, and nothing still waiting may restart it later.
                pending.Clear();
                commanded = 0f;
            }
            else if (latency > 0f)
            {
                pending.Enqueue((clock, target));
                if (pending.Count > MaxPendingCommands) pending.Dequeue();   // drop the oldest
            }
            else commanded = target;
        }

        public void ResetActuator()
        {
            running = false;
            commanded = 0f; Throttle = 0f; Thrust = 0f; Torque = 0f;
            tableThrustLagged = 0f; tableTorqueLagged = 0f;
            pending.Clear();
            clock = 0.0;
        }

        private void FixedUpdate()
        {
            if (rb == null) return;
            Step(Time.fixedDeltaTime);
            Vector3 axis = transform.up;
            rb.AddForceAtPosition(axis * Thrust, transform.position, ForceMode.Force);
            rb.AddTorque(axis * (Torque * spinDirection), ForceMode.Force);

            if (dragCoefficient > 0f && maxSpeed > 0f)
            {
                // Air moving across the spinning disc pushes back on it, at the rotor position.
                Vector3 airVelocity = rb.GetPointVelocity(transform.position) - Wind.At(transform.position);
                Vector3 drag = DragForce(airVelocity, axis, maxSpeed * SpeedFraction, dragCoefficient);
                rb.AddForceAtPosition(drag, transform.position, ForceMode.Force);
            }
        }

        /// <summary>
        /// Rotor drag, as in PX4's gz MulticopterMotorModel: air flowing across the rotor disc (the part
        /// of airVelocity perpendicular to axis) pushes back on it with a force proportional to that
        /// airflow and to the rotor speed omega (rad/s). Flow along the axis is thrust's business, not
        /// drag's. Returns a force in the same frame as airVelocity and axis.
        /// </summary>
        public static Vector3 DragForce(Vector3 airVelocity, Vector3 axis, float omega, float coefficient)
        {
            Vector3 unitAxis = axis.normalized;
            Vector3 across = airVelocity - unitAxis * Vector3.Dot(airVelocity, unitAxis);
            return -Mathf.Abs(omega) * coefficient * across;
        }

        /// <summary>
        /// Fraction of the remaining gap a first-order lag with time constant tau closes in dt. This is
        /// the exact solution of dx/dt = (target - x) / tau, so the response doesn't depend on the step
        /// size; dt / tau is only its first-order approximation.
        /// </summary>
        public static float LagAlpha(float dt, float tau) => 1f - Mathf.Exp(-dt / Mathf.Max(tau, 1e-3f));

        /// <summary>LagAlpha with the up constant if target is above current, the down constant otherwise.</summary>
        private float LagAlphaToward(float current, float target, float dt)
        {
            float up = timeConstantUp > 0f ? timeConstantUp : timeConstant;
            float down = timeConstantDown > 0f ? timeConstantDown : timeConstant;
            return LagAlpha(dt, target > current ? up : down);
        }

        /// <summary>Advances the clock, the command dead time and the spin lag by dt; sets Throttle, Thrust and Torque.</summary>
        public void Step(float dt)
        {
            // Hand over every queued command that has waited the full latency; the newest of them wins.
            // This runs before the clock moves on: a command received at time t is used by the first step
            // that starts at t + latency or later, so the dead time is the latency rounded up to whole
            // steps. (Advancing the clock first would make it one step short.) The 1e-6 absorbs the
            // float rounding of summing several dt.
            while (pending.Count > 0 && clock - pending.Peek().time >= latency - 1e-6)
                commanded = pending.Dequeue().value;
            clock += dt;

            Throttle = Mathf.Lerp(Throttle, commanded, LagAlphaToward(Throttle, commanded, dt));

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
                tableThrustLagged = Mathf.Lerp(tableThrustLagged, targetThrust, LagAlphaToward(tableThrustLagged, targetThrust, dt));
                tableTorqueLagged = Mathf.Lerp(tableTorqueLagged, targetTorque, LagAlphaToward(tableTorqueLagged, targetTorque, dt));
                thrust = tableThrustLagged;
                torque = tableTorqueLagged;
            }
            else
            {
                float w2 = Throttle * Throttle;       // thrust ~ omega^2, omega ~ throttle
                thrust = maxThrust * w2;
                torque = maxTorque * w2;
            }
            SpeedFraction = maxThrust > 0f ? Mathf.Sqrt(Mathf.Max(thrust, 0f) / maxThrust) : 0f;
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
