using UnityEngine;

/// <summary>Editor-style fly camera: left drag pans, right drag orbits, scroll zooms. From auv-sim-unity.</summary>
public class FreeCameraController : MonoBehaviour
{
    public Camera cam;
    [Range(50f, 1000f)] public float xRotateSpeed = 400f;
    [Range(-1000f, 1000f)] public float yRotateSpeed = -400f;
    [Range(1f, 50f)] public float scrollSpeed = 6f;
    public float defaultPivotDistance = 20f;

    private bool dragging, panning;
    private Vector3 pivot, camDir;

    private void Start() { if (cam == null) cam = GetComponent<Camera>(); }

    private void Update()
    {
        if (Input.GetMouseButtonDown(0)) { pivot = PointUnderMouse(); camDir = cam.transform.forward; dragging = true; }
        if (Input.GetMouseButtonUp(0)) dragging = false;
        if (Input.GetMouseButtonDown(1) || Input.GetMouseButtonDown(2)) { pivot = PointUnderMouse(); panning = true; }
        if (Input.GetMouseButtonUp(1) || Input.GetMouseButtonUp(2)) panning = false;

        if (dragging)
        {
            Vector3 dir = cam.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, 1f)) - cam.transform.position;
            Vector3 p = Intersect(cam.transform.position, dir, pivot, camDir);
            cam.transform.position += pivot - p;
        }
        if (panning)
        {
            cam.transform.RotateAround(pivot, Vector3.up, Input.GetAxis("Mouse X") * xRotateSpeed * Time.deltaTime);
            cam.transform.RotateAround(pivot, cam.transform.right, Input.GetAxis("Mouse Y") * yRotateSpeed * Time.deltaTime);
        }
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.01f)
        {
            float d = Mathf.Max(Vector3.Distance(cam.transform.position, PointUnderMouse()), 5f);
            cam.transform.position += cam.transform.forward * scroll * scrollSpeed * d / 5f;
        }
    }

    private Vector3 PointUnderMouse()
    {
        Ray ray = cam.ScreenPointToRay(Input.mousePosition);
        return Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity) ? hit.point : ray.GetPoint(defaultPivotDistance);
    }

    private Vector3 Intersect(Vector3 origin, Vector3 dir, Vector3 planePoint, Vector3 planeNormal)
    {
        float denom = Vector3.Dot(planeNormal, dir);
        if (Mathf.Abs(denom) < Mathf.Epsilon) return origin + dir * defaultPivotDistance;
        return origin + dir * (Vector3.Dot(planeNormal, planePoint - origin) / denom);
    }
}
