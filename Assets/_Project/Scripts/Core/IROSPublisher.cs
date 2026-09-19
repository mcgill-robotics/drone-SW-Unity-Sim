/// <summary>Contract for anything that publishes to a ROS topic.</summary>
public interface IROSPublisher
{
    string Topic { get; }
    float PublishRate { get; set; }
    bool IsActive { get; }
    void SetActive(bool active);
    void PublishMessage();
}
