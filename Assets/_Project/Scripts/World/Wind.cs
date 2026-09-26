using UnityEngine;

/// <summary>
/// Global wind: a mean vector plus slow gusts. Aerodynamic components subtract this from
/// their velocity to get airspeed.
/// </summary>
public class Wind : MonoBehaviour
{
    private static Wind _instance;
    public static Wind Instance { get => _instance != null ? _instance : _instance = FindAnyObjectByType<Wind>(); private set => _instance = value; }

    [Tooltip("Mean wind vector in Unity world axes (m/s), direction the air moves toward")]
    public Vector3 meanWind = Vector3.zero;
    [Tooltip("Gust amplitude (m/s)")]
    public float gustAmplitude = 0f;
    [Tooltip("Gust period (s)")]
    public float gustPeriod = 5f;

    public Vector3 Current { get; private set; }

    private void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;
    }

    private void FixedUpdate()
    {
        float t = Time.fixedTime;
        float g = gustAmplitude * (Mathf.PerlinNoise(t / gustPeriod, 0.37f) - 0.5f) * 2f;
        Vector3 dir = meanWind.sqrMagnitude > 1e-6f ? meanWind.normalized : Vector3.forward;
        Current = meanWind + dir * g;
    }

    public static Vector3 At(Vector3 worldPosition) => Instance != null ? Instance.Current : Vector3.zero;
}
