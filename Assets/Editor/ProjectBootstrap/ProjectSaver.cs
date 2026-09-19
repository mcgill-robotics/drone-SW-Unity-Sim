using UnityEditor;
using UnityEngine;

namespace ProjectBootstrap
{
    public static class ProjectSaver
    {
        // unity run <proj> -- -executeMethod ProjectBootstrap.ProjectSaver.SaveAll
        public static void SaveAll()
        {
            AssetDatabase.Refresh(ImportAssetOptions.ForceUpdate);
            AssetDatabase.SaveAssets();
            Debug.Log("[ProjectSaver] Assets imported and saved.");
            EditorApplication.Exit(0);
        }
    }
}
