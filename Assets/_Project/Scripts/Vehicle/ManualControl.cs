using UnityEngine;

/// <summary>
/// Keyboard flying for when PX4 is not connected. Press M to toggle. A small PD attitude hold
/// keeps the airframe level in hover, otherwise a raw quad is unflyable from a keyboard.
///
///   Shift / Ctrl   collective up / down (lift rotors)
///   W / S          pitch nose down / up
///   A / D          roll left / right
///   Q / E          yaw left / right
///   Up / Down      pusher throttle
///   Space          cut everything
/// </summary>
public class ManualControl : MonoBehaviour
{
    private static ManualControl _instance;
    public static ManualControl Instance { get => _instance != null ? _instance : _instance = FindAnyObjectByType<ManualControl>(); private set => _instance = value; }
    public bool Active => active;

    [Header("Channels (match VehicleActuators layout)")]
    public int[] liftRotors = { 0, 1, 2, 3 };
    [Tooltip("Sign of each lift rotor's contribution to roll (positive = right wing down) and pitch (positive = nose up)")]
    public float[] rollMix = { -1f, +1f, +1f, -1f };   // FR, RL, FL, RR
    public float[] pitchMix = { +1f, -1f, +1f, -1f };
    public float[] yawMix = { +1f, +1f, -1f, -1f };
    public int pusherChannel = 4, aileronChannel = 5, elevatorChannel = 6, rudderChannel = 7;

    [Header("Feel")]
    public float collectiveRate = 0.6f;     // per second
    public float pusherRate = 0.6f;
    public float maxTiltDeg = 15f;
    public float yawRateDegPerSec = 60f;
    public float attitudeP = 0.04f, attitudeD = 0.01f;
    public float yawD = 0.02f;

    private bool active;
    private float collective, pusher;
    private readonly float[] controls = new float[VehicleActuators.ChannelCount];
    private Rigidbody rb;
    private VehicleActuators actuators;

    private void Awake() => Instance = this;

    private void Start()
    {
        rb = SimSettings.Vehicle;
        actuators = rb != null ? rb.GetComponent<VehicleActuators>() : null;
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.M))
        {
            active = !active;
            collective = 0f; pusher = 0f;
            Debug.Log($"[Manual] {(active ? "on" : "off")}");
        }
    }

    private void FixedUpdate()
    {
        if (!active || rb == null || actuators == null) return;
        float dt = Time.fixedDeltaTime;

        if (Input.GetKey(KeyCode.Space)) { collective = 0f; pusher = 0f; }
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)) collective += collectiveRate * dt;
        if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)) collective -= collectiveRate * dt;
        if (Input.GetKey(KeyCode.UpArrow)) pusher += pusherRate * dt;
        if (Input.GetKey(KeyCode.DownArrow)) pusher -= pusherRate * dt;
        collective = Mathf.Clamp01(collective);
        pusher = Mathf.Clamp01(pusher);

        float rollCmd = (Input.GetKey(KeyCode.D) ? 1f : 0f) - (Input.GetKey(KeyCode.A) ? 1f : 0f);
        float pitchCmd = (Input.GetKey(KeyCode.S) ? 1f : 0f) - (Input.GetKey(KeyCode.W) ? 1f : 0f);
        float yawCmd = (Input.GetKey(KeyCode.E) ? 1f : 0f) - (Input.GetKey(KeyCode.Q) ? 1f : 0f);

        // Attitude hold in aero convention: roll +right wing down, pitch +nose up.
        Vector3 e = rb.rotation.eulerAngles;
        float roll = -Wrap(e.z);          // Unity +z roll lifts the right wing
        float pitch = -Wrap(e.x);         // Unity +x rotation drops the nose
        Vector3 w = rb.transform.InverseTransformDirection(rb.angularVelocity);
        float p = -w.z, q = -w.x, r = w.y;

        float rollOut = attitudeP * (rollCmd * maxTiltDeg - roll) - attitudeD * p * Mathf.Rad2Deg;
        float pitchOut = attitudeP * (pitchCmd * maxTiltDeg - pitch) - attitudeD * q * Mathf.Rad2Deg;
        float yawOut = yawD * (yawCmd * yawRateDegPerSec - r * Mathf.Rad2Deg);

        System.Array.Clear(controls, 0, controls.Length);
        for (int i = 0; i < liftRotors.Length; i++)
        {
            float t = collective + rollMix[i] * rollOut + pitchMix[i] * pitchOut + yawMix[i] * yawOut;
            controls[liftRotors[i]] = Mathf.Clamp01(t);              // motors are 0..1, as PX4 sends them
        }
        controls[pusherChannel] = pusher;
        controls[aileronChannel] = Mathf.Clamp(rollCmd + rollOut, -1f, 1f);
        controls[elevatorChannel] = Mathf.Clamp(pitchCmd + pitchOut, -1f, 1f);
        controls[rudderChannel] = Mathf.Clamp(yawCmd, -1f, 1f);

        actuators.SetControls(controls, true);
    }

    private static float Wrap(float deg) => deg > 180f ? deg - 360f : deg;

    private void OnGUI()
    {
        if (!active) return;
        GUI.Label(new Rect(10, 30, 600, 20), $"MANUAL  collective={collective:F2} pusher={pusher:F2}   (M toggles, Space cuts)");
    }
}
