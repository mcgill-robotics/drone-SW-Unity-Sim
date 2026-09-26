using UnityEngine;

/// <summary>
/// A control surface modelled as a moment about a body axis, scaled by dynamic pressure.
/// Deflection comes straight from a PX4 channel (-1..1). A positive deflection produces a
/// positive aero moment (roll right, pitch up, yaw right); set invert for surfaces whose PX4
/// torque sign is negative, like standard_vtol's left aileron.
/// </summary>
public class ControlSurface : MonoBehaviour, IActuator
{
    public enum Axis { Roll, Pitch, Yaw }

    [Header("PX4 channel")]
    public int channel = StandardVtolChannels.Elevator;
    public bool invert = false;

    [Header("Aero")]
    public Axis axis = Axis.Pitch;
    [Tooltip("Moment coefficient per unit deflection, times reference area and length (m^3)")]
    public float momentCoefficient = 0.02f;
    [Tooltip("Servo time constant (s)")] public float timeConstant = 0.05f;

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
        commanded = channel < controls.Length ? Mathf.Clamp(controls[channel], -1f, 1f) * (invert ? -1f : 1f) : 0f;
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
        // positive yaw = nose right; the signs below make a positive PX4 deflection do that.
        Vector3 torque = axis switch
        {
            Axis.Roll => rb.transform.forward * -m,
            Axis.Pitch => rb.transform.right * -m,
            _ => rb.transform.up * m
        };
        rb.AddTorque(torque, ForceMode.Force);
    }
}
