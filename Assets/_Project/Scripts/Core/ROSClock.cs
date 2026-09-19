using UnityEngine;
using RosMessageTypes.Rosgraph;
using RosMessageTypes.BuiltinInterfaces;

/// <summary>
/// Publishes /clock from simulated time and hands out timestamps to every other publisher,
/// so stamps stay consistent when Time.timeScale != 1. Ported from auv-sim-unity.
/// </summary>
public class ROSClock : ROSPublisher
{
    public override string Topic => ROSSettings.Instance.ClockTopic;

    private ClockMsg clockMsg;
    private double simTime;
    private static ROSClock _instance;

    /// <summary>Seconds of simulated time since the scene started.</summary>
    public static double SimTime => _instance != null ? _instance.simTime : 0.0;

    public static TimeMsg GetROSTimestamp()
    {
        double t = SimTime;
        int secs = (int)t;
        uint nsecs = (uint)((t - secs) * 1e9);
        return new TimeMsg { sec = secs, nanosec = nsecs };
    }

    public static ulong GetMicroseconds() => (ulong)(SimTime * 1e6);

    protected override void Start()
    {
        _instance = this;
        base.Start();
        clockMsg = new ClockMsg { clock = new TimeMsg() };
    }

    protected override void RegisterPublisher() => ros.RegisterPublisher<ClockMsg>(Topic);

    protected override void FixedUpdate()
    {
        simTime += Time.fixedDeltaTime;
        base.FixedUpdate();
    }

    public override void PublishMessage()
    {
        clockMsg.clock = GetROSTimestamp();
        ros.Publish(Topic, clockMsg);
    }
}
