using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Drag on the airframe itself, separate from the wing and the rotors: a force that opposes the
    /// air moving past the body and grows with the square of its speed, along each body axis, and a
    /// moment that opposes rotation in proportion to the spin rate. Both act on the centre of mass.
    /// Built from the vehicle spec's bodyDrag section; without it only the rigidbody's built-in
    /// angular damping slows the aircraft's rotation.
    /// </summary>
    public class BodyDrag : MonoBehaviour
    {
        [Tooltip("Drag coefficient x reference area along each body axis (m^2). Unity axes: x right, y up, z forward")]
        public Vector3 dragArea;
        [Tooltip("Angular damping about each body axis (N m s/rad). Unity axes: x pitch, y yaw, z roll")]
        public Vector3 angularDamping;

        private Rigidbody rb;
        private Wing wing;

        private void Awake()
        {
            rb = GetComponentInParent<Rigidbody>();
            wing = GetComponentInParent<Wing>();
        }

        private void FixedUpdate()
        {
            if (rb == null) return;

            float density = wing != null ? wing.airDensity : Wing.SeaLevelDensity;
            Vector3 airVelocity = rb.linearVelocity - Wind.At(rb.worldCenterOfMass);
            Vector3 airBody = rb.transform.InverseTransformDirection(airVelocity);
            rb.AddForce(rb.transform.TransformDirection(Force(airBody, dragArea, density)), ForceMode.Force);

            Vector3 spinBody = rb.transform.InverseTransformDirection(rb.angularVelocity);
            rb.AddTorque(rb.transform.TransformDirection(Moment(spinBody, angularDamping)), ForceMode.Force);
        }

        /// <summary>
        /// Quadratic drag per body axis: -1/2 x density x (Cd x A) x |v| x v. Using |v| x v rather than v
        /// squared keeps the sign, so the force always opposes the motion along that axis.
        /// </summary>
        public static Vector3 Force(Vector3 airVelocityBody, Vector3 dragArea, float airDensity)
        {
            return new Vector3(
                AxisForce(airVelocityBody.x, dragArea.x, airDensity),
                AxisForce(airVelocityBody.y, dragArea.y, airDensity),
                AxisForce(airVelocityBody.z, dragArea.z, airDensity));
        }

        private static float AxisForce(float velocity, float area, float density) =>
            -0.5f * density * area * Mathf.Abs(velocity) * velocity;

        /// <summary>Linear angular damping per body axis: -damping x angular velocity.</summary>
        public static Vector3 Moment(Vector3 angularVelocityBody, Vector3 angularDamping) =>
            -Vector3.Scale(angularDamping, angularVelocityBody);
    }
}
