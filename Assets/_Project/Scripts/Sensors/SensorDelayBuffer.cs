using System.Collections.Generic;
using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Records the rigidbody state every physics step so sensors can read a delayed sample,
    /// simulating bus, middleware and bridge latency.
    /// </summary>
    [DefaultExecutionOrder(-75)]
    public class SensorDelayBuffer : MonoBehaviour, IResettable
    {
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

        public RigidbodyStateSample GetDelayedState(float delaySeconds)
        {
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
    }

    public struct RigidbodyStateSample
    {
        public float timestamp;
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 linearVelocity;
        public Vector3 angularVelocity;
    }
}
