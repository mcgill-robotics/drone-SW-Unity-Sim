using System.Collections.Generic;
using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Receives the 16 normalized actuator controls from PX4 and hands each channel to whichever
    /// Rotor or ControlSurface claims it. Also lets the keyboard drive channels when PX4 is absent.
    /// </summary>
    public class VehicleActuators : MonoBehaviour, IResettable
    {
        public const int ChannelCount = 16;

        [Tooltip("Latest controls from PX4, -1..1")]
        public float[] controls = new float[ChannelCount];
        public bool armed;

        private readonly List<IActuator> actuators = new List<IActuator>();

        private void Awake()
        {
            actuators.AddRange(GetComponentsInChildren<IActuator>());
        }

        public void SetControls(float[] values, bool isArmed)
        {
            int n = Mathf.Min(values.Length, ChannelCount);
            for (int i = 0; i < n; i++) controls[i] = values[i];
            armed = isArmed;
        }

        private void FixedUpdate()
        {
            foreach (var a in actuators) a.Apply(controls, armed);
        }

        public void ResetState()
        {
            System.Array.Clear(controls, 0, controls.Length);
            armed = false;
            foreach (var a in actuators) a.ResetActuator();
        }
    }

    /// <summary>
    /// Output channels of PX4's standard_vtol airframe (1040_gazebo-classic_standard_vtol,
    /// PWM_MAIN_FUNC1..8). Motors arrive 0..1, servos -1..1. The scene builder and manual control
    /// both read from here so there is one place to change when the real airframe differs.
    /// </summary>
    public static class StandardVtolChannels
    {
        public const int RotorFrontRight = 0, RotorRearLeft = 1, RotorFrontLeft = 2, RotorRearRight = 3;
        public const int Pusher = 4;
        /// <summary>CA_SV_CS0: left aileron, CA_SV_CS0_TRQ_R = -0.5 (positive rolls left).</summary>
        public const int AileronLeft = 5;
        /// <summary>CA_SV_CS1: right aileron, CA_SV_CS1_TRQ_R = +0.5 (positive rolls right).</summary>
        public const int AileronRight = 6;
        /// <summary>CA_SV_CS2: elevator, CA_SV_CS2_TRQ_P = +1 (positive pitches nose up).</summary>
        public const int Elevator = 7;
    }

    public interface IActuator
    {
        void Apply(float[] controls, bool armed);
        void ResetActuator();
    }
}
