using System;
using System.IO;
using System.Runtime.CompilerServices;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

[assembly: InternalsVisibleTo("DroneSim.Tests.EditMode")]

namespace DroneSim.Editor
{
    [InitializeOnLoad]
    static class VehiclePreview
    {
        static VehicleLoader loader;
        static string vehicle;
        static string path;
        static DateTime lastWrite;
        static double nextCheck;
        static GameObject preview;

        static VehiclePreview()
        {
            EditorApplication.update += Update;
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
            AssemblyReloadEvents.beforeAssemblyReload += Clear;
            EditorApplication.quitting += Clear;
        }

        static void Update()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode) return;
            if (EditorApplication.timeSinceStartup < nextCheck) return;
            nextCheck = EditorApplication.timeSinceStartup + 0.5;

            Refresh(loader != null ? loader : Object.FindFirstObjectByType<VehicleLoader>());
        }

        internal static void Refresh(VehicleLoader current)
        {
            if (current == null || !current.isActiveAndEnabled)
            {
                Clear();
                return;
            }
            bool changed = loader != current || vehicle != current.vehicle;
            if (changed)
            {
                Clear();
                loader = current;
                vehicle = current.vehicle;
            }
            try
            {
                if (changed) path = VehicleSpec.Resolve(vehicle);
                if (path == null) return;
                var modified = File.GetLastWriteTimeUtc(path);
                if (!changed && modified == lastWrite) return;
                lastWrite = modified;

                if (preview != null) Object.DestroyImmediate(preview);
                preview = null;
                var spec = VehicleSpec.Load(path);
                var problems = spec.GeometryProblems();
                if (problems.Count > 0)
                {
                    Debug.LogWarning("[Vehicle Preview] no preview built:\n  " + string.Join("\n  ", problems), loader);
                    return;
                }
                preview = VehicleBuilder.BuildPreview(spec);
                SceneManager.MoveGameObjectToScene(preview, loader.gameObject.scene);
                SceneView.RepaintAll();
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[Vehicle Preview] can't preview {vehicle}: {e.Message}", loader);
            }
        }

        internal static void OnPlayModeChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.ExitingEditMode) Clear();
        }

        internal static void Clear()
        {
            if (preview != null) Object.DestroyImmediate(preview);
            preview = null;
            loader = null;
            vehicle = null;
            path = null;
            lastWrite = default;
        }
    }
}
