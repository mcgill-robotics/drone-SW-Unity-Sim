using UnityEngine;

/// <summary>
/// One propeller. Thrust along the local +Y axis of this transform, applied at its position,
/// with a first-order spin-up lag and a reaction torque about the axis.
/// PX4's pwm_out_sim sends non-reversible motors as 0..1, so the channel value is the throttle.
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
    [Tooltip("Runtime multiplier used by domain randomization")] public float efficiency = 1f;

    /// <summary>Throttle PX4 is asking for, before the spin-up lag.</summary>
    public float Commanded => commanded;
    public float Throttle { get; private set; }
    public float Thrust { get; private set; }

    private Rigidbody rb;
    private float commanded;

    private void Awake() => rb = GetComponentInParent<Rigidbody>();

    public void Apply(float[] controls, bool armed)
    {
        commanded = armed && channel < controls.Length ? Mathf.Clamp01(controls[channel]) : 0f;
    }

    public void ResetActuator() { commanded = 0f; Throttle = 0f; Thrust = 0f; }

    private void FixedUpdate()
    {
        if (rb == null) return;
        float alpha = Time.fixedDeltaTime / Mathf.Max(timeConstant, 1e-3f);
        Throttle = Mathf.Lerp(Throttle, commanded, Mathf.Clamp01(alpha));

        float w2 = Throttle * Throttle;           // thrust ~ omega^2, omega ~ throttle
        Thrust = maxThrust * efficiency * w2;
        Vector3 axis = transform.up;
        rb.AddForceAtPosition(axis * Thrust, transform.position, ForceMode.Force);
        rb.AddTorque(axis * (maxTorque * w2 * spinDirection), ForceMode.Force);
    }
}
