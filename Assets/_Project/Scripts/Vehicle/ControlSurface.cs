using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// A control surface modelled as a moment scaled by dynamic pressure. Deflection comes straight
    /// from a PX4 channel (-1..1). torque is what a positive deflection does, in the same terms as
    /// PX4's CA_SV_CSn_TRQ_R/P/Y (positive = roll right, pitch up, yaw right): (-0.5, 0, 0) for
    /// standard_vtol's left aileron, (0, 1, 0) for its elevator.
    /// </summary>
    public class ControlSurface : MonoBehaviour, IActuator
    {
        [Header("PX4 channel")]
        public int channel = StandardVtolChannels.Elevator;

        [Header("Aero")]
        [Tooltip("Roll, pitch and yaw effect of a positive deflection, as PX4's CA_SV_CSn_TRQ_R/P/Y")]
        public Vector3 torque = new Vector3(0f, 1f, 0f);
        [Tooltip("Moment per unit torque and deflection, divided by dynamic pressure (m^3)")]
        public float momentCoefficient = 0.02f;
        [Tooltip("Servo time constant (s)")] public float timeConstant = 0.05f;

        /// <summary>Deflection PX4 is asking for, before the servo lag.</summary>
        public float Commanded => commanded;
        public float Deflection { get; private set; }

        private Rigidbody rb;
        private Wing wing;
        private float commanded;

        private void Awake()
        {
            rb = GetComponentInParent<Rigidbody>();
            wing = GetComponentInParent<Wing>();
        }

        public void Apply(float[] controls, bool armed)
        {
            commanded = channel < controls.Length ? Mathf.Clamp(controls[channel], -1f, 1f) : 0f;
        }

        public void ResetActuator() { commanded = 0f; Deflection = 0f; }

        private void FixedUpdate()
        {
            if (rb == null) return;
            Deflection = Mathf.Lerp(Deflection, commanded, Mathf.Clamp01(Time.fixedDeltaTime / Mathf.Max(timeConstant, 1e-3f)));

            Vector3 airVel = rb.linearVelocity - Wind.At(rb.position);
            float q = 0.5f * (wing != null ? wing.airDensity : 1.225f) * airVel.sqrMagnitude;
            float m = q * momentCoefficient * Deflection;

            // Unity body axes are left-handed. Positive roll = right wing down, positive pitch = nose up,
            // positive yaw = nose right; the signs below make a positive torque component do that.
            rb.AddTorque(rb.transform.forward * (-m * torque.x) + rb.transform.right * (-m * torque.y) +
                         rb.transform.up * (m * torque.z), ForceMode.Force);
        }
    }
}
