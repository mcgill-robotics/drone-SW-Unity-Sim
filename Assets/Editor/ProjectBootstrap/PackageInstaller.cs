using System.Linq;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEditor.PackageManager.Requests;
using UnityEngine;

namespace ProjectBootstrap
{
    // Headless package install. Run with:
    //   Unity -batchmode -projectPath <proj> -executeMethod ProjectBootstrap.PackageInstaller.Install -logFile -
    // No -quit: the request completes on later editor ticks and the script exits itself.
    public static class PackageInstaller
    {
        static readonly string[] PackagesToAdd =
        {
            "https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector#v0.7.1",
        };

        static readonly string[] PackagesToRemove =
        {
            "com.unity.visualscripting",
            "com.unity.collab-proxy",
        };

        const double TimeoutSeconds = 900;
        static AddAndRemoveRequest _request;
        static double _deadline;

        public static void Install()
        {
            Debug.Log($"[PackageInstaller] Adding: {string.Join(", ", PackagesToAdd)}");
            _request = Client.AddAndRemove(packagesToAdd: PackagesToAdd, packagesToRemove: PackagesToRemove);
            _deadline = EditorApplication.timeSinceStartup + TimeoutSeconds;
            EditorApplication.update += Poll;
        }

        static void Poll()
        {
            if (_request == null) return;
            if (!_request.IsCompleted)
            {
                if (EditorApplication.timeSinceStartup > _deadline)
                {
                    EditorApplication.update -= Poll;
                    Debug.LogError("[PackageInstaller] Timed out waiting for UPM.");
                    EditorApplication.Exit(2);
                }
                return;
            }
            EditorApplication.update -= Poll;
            if (_request.Status == StatusCode.Success)
            {
                var names = _request.Result.Select(p => $"{p.name}@{p.version}");
                Debug.Log($"[PackageInstaller] Resolved: {string.Join(", ", names)}");
                EditorApplication.Exit(0);
            }
            else
            {
                Debug.LogError($"[PackageInstaller] Failed: {_request.Error?.message}");
                EditorApplication.Exit(1);
            }
        }
    }
}
