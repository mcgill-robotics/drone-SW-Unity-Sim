using System.Collections.Generic;
using UnityEngine;

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

public interface IActuator
{
    void Apply(float[] controls, bool armed);
    void ResetActuator();
}
