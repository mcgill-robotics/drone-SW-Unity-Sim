using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Records the rigidbody state every physics step so sensors can read a delayed sample,
/// simulating bus, middleware and bridge latency. Ported from auv-sim-unity without the
/// DVL ping throttling.
/// </summary>
[DefaultExecutionOrder(-75)]
public class SensorDelayBuffer : MonoBehaviour, IResettable
{
    [Tooltip("Sensor latency in seconds")]
    public float delayTime = 0.02f;

    [Header("Randomization")]
    public bool randomizeLatencyOnReset = false;
    public float minDelayTime = 0.01f;
    public float maxDelayTime = 0.05f;

    private Rigidbody rb;
    private readonly List<RigidbodyStateSample> buffer = new List<RigidbodyStateSample>();

    private void Awake() => rb = GetComponent<Rigidbody>();

    public static SensorDelayBuffer GetOrCreate(Rigidbody target)
    {
        if (target == null) return null;
        var buf = target.GetComponent<SensorDelayBuffer>();
        return buf != null ? buf : target.gameObject.AddComponent<SensorDelayBuffer>();
    }

    public void ResetState()
    {
        if (randomizeLatencyOnReset) delayTime = Random.Range(minDelayTime, maxDelayTime);
        buffer.Clear();
        if (rb != null) RecordSample(Time.time);
    }

    private void FixedUpdate()
    {
        if (rb == null) return;
        RecordSample(Time.time);
        while (buffer.Count > 1 && (Time.time - buffer[0].timestamp) > 2f) buffer.RemoveAt(0);
    }

    private void RecordSample(float t)
    {
        buffer.Add(new RigidbodyStateSample
        {
            timestamp = t,
            position = rb.position,
            rotation = rb.rotation,
            linearVelocity = rb.linearVelocity,
            angularVelocity = rb.angularVelocity
        });
    }

    public RigidbodyStateSample GetDelayedState(float delaySeconds = -1f)
    {
        if (delaySeconds < 0f) delaySeconds = delayTime;
        if (buffer.Count == 0)
        {
            return rb != null
                ? new RigidbodyStateSample { timestamp = Time.time, position = rb.position, rotation = rb.rotation, linearVelocity = rb.linearVelocity, angularVelocity = rb.angularVelocity }
                : new RigidbodyStateSample { rotation = Quaternion.identity };
        }

        float target = Time.time - delaySeconds;
        int best = 0;
        float minDiff = Mathf.Abs(buffer[0].timestamp - target);
        for (int i = 1; i < buffer.Count; i++)
        {
            float d = Mathf.Abs(buffer[i].timestamp - target);
            if (d < minDiff) { minDiff = d; best = i; } else break;
        }
        return buffer[best];
    }

    /// <summary>Velocity of a point fixed to the body at a local offset, from the delayed sample.</summary>
    public Vector3 GetDelayedVelocityAtLocalOffset(Vector3 localOffset, float delaySeconds = -1f)
    {
        var s = GetDelayedState(delaySeconds);
        return s.linearVelocity + Vector3.Cross(s.angularVelocity, s.rotation * localOffset);
    }
}

public struct RigidbodyStateSample
{
    public float timestamp;
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 linearVelocity;
    public Vector3 angularVelocity;
}
