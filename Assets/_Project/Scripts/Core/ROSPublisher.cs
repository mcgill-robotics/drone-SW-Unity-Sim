using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace DroneSim
{
    /// <summary>
    /// Base class for ROS publishers. Handles connection, registration and rate-limited publishing
    /// from FixedUpdate. Subclasses that need their own cadence set useBaseRateLimiting = false.
    /// Ported from auv-sim-unity.
    /// </summary>
    public abstract class ROSPublisher : MonoBehaviour
    {
        [Header("ROS")]
        [Tooltip("Target publish rate in Hz (only used when the base class drives timing)")]
        [SerializeField] private float _publishRate = 10f;

        public abstract string Topic { get; }

        public float PublishRate
        {
            get => _publishRate;
            set
            {
                _publishRate = Mathf.Clamp(value, 0.1f, 500f);
                timeBetweenPublishes = 1f / _publishRate;
            }
        }

        protected bool useBaseRateLimiting = true;
        protected ROSConnection ros;
        protected float timeBetweenPublishes;
        protected float timeSinceLastPublish;

        protected virtual void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            timeBetweenPublishes = 1f / _publishRate;
            RegisterPublisher();
        }

        protected virtual void FixedUpdate()
        {
            if (!useBaseRateLimiting) return;
            timeSinceLastPublish += Time.fixedDeltaTime;
            if (SimSettings.PublishROS && timeSinceLastPublish >= timeBetweenPublishes)
            {
                PublishMessage();
                timeSinceLastPublish = 0f;
            }
        }

        protected abstract void RegisterPublisher();
        public abstract void PublishMessage();
    }
}
