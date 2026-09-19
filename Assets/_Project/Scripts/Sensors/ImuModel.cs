using UnityEngine;
using Utils;

/// <summary>
/// Simulated IMU: proper acceleration and angular rate at the sensor location, with white noise
/// and Gauss-Markov bias drift. Output is in Unity body axes; consumers convert to FLU or FRD.
/// Shared by the ROS IMU publisher and the PX4 HIL bridge so both see the same instrument.
/// </summary>
public class ImuModel : MonoBehaviour, IResettable
{
    [Header("Noise")]
    public bool enableNoise = true;
    [Tooltip("Accelerometer white noise std dev (m/s^2)")] public float accelNoise = 0.05f;
    [Tooltip("Accelerometer bias correlation time (s)")] public float accelBiasTau = 300f;
    [Tooltip("Accelerometer bias std dev (m/s^2)")] public float accelBiasSigma = 0.02f;
    [Tooltip("Gyro white noise std dev (rad/s)")] public float gyroNoise = 0.005f;
    [Tooltip("Gyro bias correlation time (s)")] public float gyroBiasTau = 100f;
    [Tooltip("Gyro bias std dev (rad/s)")] public float gyroBiasSigma = 0.002f;

    [Header("Latency")]
    [Tooltip("Delay applied through the SensorDelayBuffer (s). 0 = read current state.")]
    public float delay = 0f;

    /// <summary>Proper acceleration in Unity body axes (x right, y up, z forward), includes -gravity.</summary>
    public Vector3 Acceleration { get; private set; }
    /// <summary>Angular rate in Unity body axes.</summary>
    public Vector3 AngularRate { get; private set; }
    /// <summary>True (noise-free) orientation of the sensor in world frame.</summary>
    public Quaternion Orientation { get; private set; } = Quaternion.identity;

    private Rigidbody rb;
    private SensorDelayBuffer delayBuffer;
    private Vector3 localOffset;
    private Quaternion localRot;
    private Vector3 lastPointVelocity;
    private GaussMarkovVector gyroBias, accelBias;

    private void Start()
    {
        rb = GetComponentInParent<Rigidbody>();
        if (rb == null) rb = SimSettings.Vehicle;
        if (rb == null) { Debug.LogError("[ImuModel] No rigidbody"); enabled = false; return; }

        delayBuffer = SensorDelayBuffer.GetOrCreate(rb);
        localOffset = rb.transform.InverseTransformPoint(transform.position);
        localRot = Quaternion.Inverse(rb.rotation) * transform.rotation;
        lastPointVelocity = rb.GetPointVelocity(transform.position);

        float dt = Time.fixedDeltaTime;
        gyroBias = new GaussMarkovVector(gyroBiasTau, gyroBiasSigma, dt);
        accelBias = new GaussMarkovVector(accelBiasTau, accelBiasSigma, dt);
    }

    public void ResetState()
    {
        lastPointVelocity = rb != null ? rb.GetPointVelocity(transform.position) : Vector3.zero;
        gyroBias?.Reset();
        accelBias?.Reset();
    }

    private void FixedUpdate()
    {
        if (rb == null) return;
        if (gyroBias == null || accelBias == null || delayBuffer == null) Start(); // survive a domain reload mid-play
        gyroBias.Step();
        accelBias.Step();

        RigidbodyStateSample s = delayBuffer.GetDelayedState(delay);
        Quaternion sensorRot = s.rotation * localRot;
        Orientation = sensorRot;

        Vector3 pointVel = s.linearVelocity + Vector3.Cross(s.angularVelocity, s.rotation * localOffset);
        Vector3 worldAccel = (pointVel - lastPointVelocity) / Time.fixedDeltaTime;
        lastPointVelocity = pointVel;

        Vector3 properWorld = worldAccel - Physics.gravity;
        Vector3 accelBody = Quaternion.Inverse(sensorRot) * properWorld;
        Vector3 rateBody = Quaternion.Inverse(sensorRot) * s.angularVelocity;

        if (enableNoise)
        {
            accelBody += Stochastic.GenerateWhiteNoiseVector(accelNoise) + accelBias.CurrentBias;
            rateBody += Stochastic.GenerateWhiteNoiseVector(gyroNoise) + gyroBias.CurrentBias;
        }

        Acceleration = accelBody;
        AngularRate = rateBody;
    }
}
