using UnityEngine;

/// <summary>
/// The handful of global switches the sim needs. Deliberately small; add fields only when
/// more than one script needs them.
/// </summary>
[DefaultExecutionOrder(-100)]
public class SimSettings : MonoBehaviour
{
    public static SimSettings Instance { get; private set; }

    [Header("Vehicle")]
    [Tooltip("The rigidbody every sensor reads from")]
    public Rigidbody vehicle;

    [Header("ROS")]
    [Tooltip("Master switch for all ROS publishers")]
    public bool publishROS = true;
    public bool useJPEGCompression = true;
    [Range(1, 100)] public int jpegQuality = 75;

    public static bool PublishROS => Instance != null && Instance.publishROS;
    public static Rigidbody Vehicle => Instance != null ? Instance.vehicle : null;

    private void Awake()
    {
        if (Instance == null) { Instance = this; DontDestroyOnLoad(gameObject); }
        else Destroy(gameObject);
    }
}
