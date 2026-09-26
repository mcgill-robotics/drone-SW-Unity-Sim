using UnityEngine;

namespace DroneSim
{
    /// <summary>Chase camera that orbits the vehicle. Right drag rotates, scroll zooms. From auv-sim-unity.</summary>
    public class OrbitFollowCamera : MonoBehaviour
    {
        [SerializeField] private Transform targetOverride;
        private Transform Target => targetOverride != null ? targetOverride : SimSettings.Vehicle != null ? SimSettings.Vehicle.transform : null;

        public float distance = 8f, minDistance = 2f, maxDistance = 200f, zoomSpeed = 10f;
        public float xSpeed = 200f, ySpeed = 120f;
        public float yMinLimit = -20f, yMaxLimit = 80f;
        public float initialYaw = 0f, initialPitch = 20f;
        public float movementSmoothing = 10f, rotationSmoothing = 10f;

        private float x, y, curDist;
        private Quaternion curRot;
        private bool init;

        private void OnEnable()
        {
            if (Target == null) return;
            if (!init) { x = initialYaw; y = initialPitch; init = true; }
            curRot = transform.rotation;
            curDist = Vector3.Distance(transform.position, Target.position);
        }

        private void LateUpdate()
        {
            var t = Target;
            if (t == null) return;
            if (Input.GetMouseButton(1))
            {
                x += Input.GetAxis("Mouse X") * xSpeed * 0.02f;
                y -= Input.GetAxis("Mouse Y") * ySpeed * 0.02f;
            }
            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (scroll != 0) distance = Mathf.Clamp(distance - scroll * zoomSpeed, minDistance, maxDistance);
            y = Mathf.Clamp(y, yMinLimit, yMaxLimit);

            // Orbit in world yaw so the camera does not roll with the airframe.
            float heading = t.rotation.eulerAngles.y;
            Quaternion target = Quaternion.Euler(y, heading + x, 0);
            float dt = Time.deltaTime;
            curRot = Quaternion.Slerp(curRot, target, dt * rotationSmoothing);
            curDist = Mathf.Lerp(curDist, distance, dt * movementSmoothing);
            transform.rotation = curRot;
            transform.position = t.position + curRot * new Vector3(0, 0, -curDist);
        }
    }
}
