using UnityEngine;

namespace DroneSim
{
    /// <summary>
    /// Owns the geodetic location of the Unity world origin and converts both ways.
    /// Unity +Z is north, +X is east, +Y is up. Flat-earth approximation, fine for a few km.
    /// Nothing else in the project does its own geo math.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class GeoOrigin : MonoBehaviour
    {
        private static GeoOrigin _instance;
        public static GeoOrigin Instance { get => _instance != null ? _instance : _instance = FindAnyObjectByType<GeoOrigin>(); private set => _instance = value; }

        // Defaults are PX4 SITL's stock home, so a plain PX4 launch lands where PX4's docs expect.
        [Header("Origin (WGS84)")]
        public double latitude = 47.397742;
        public double longitude = 8.545594;
        [Tooltip("Altitude of Unity y=0 above mean sea level (m)")]
        public double altitude = 488.0;

        private const double EarthRadius = 6378137.0;
        private double cosLat;
        private double CosLat => cosLat != 0 ? cosLat : cosLat = System.Math.Cos(latitude * Mathf.Deg2Rad);

        private void Awake()
        {
            if (Instance != null && Instance != this) { Destroy(gameObject); return; }
            Instance = this;
            ApplyEnvironmentOverrides();
            cosLat = System.Math.Cos(latitude * Mathf.Deg2Rad);
        }

        /// <summary>
        /// With Unity as the simulator, PX4's home comes from the GPS we send, so PX4_HOME_LAT/LON/ALT
        /// on the PX4 side do nothing. Read the same variables here so the usual way of moving home
        /// still works.
        /// </summary>
        public void ApplyEnvironmentOverrides()
        {
            latitude = EnvDouble("PX4_HOME_LAT", latitude);
            longitude = EnvDouble("PX4_HOME_LON", longitude);
            altitude = EnvDouble("PX4_HOME_ALT", altitude);
            cosLat = 0;
        }

        private static double EnvDouble(string name, double fallback)
        {
            string v = System.Environment.GetEnvironmentVariable(name);
            return double.TryParse(v, System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out double d) ? d : fallback;
        }

        /// <summary>Unity world position relative to this object's position (still Unity axes).</summary>
        public Vector3 ToLocal(Vector3 world) => world - transform.position;

        public void ToGeodetic(Vector3 world, out double lat, out double lon, out double alt)
        {
            Vector3 d = ToLocal(world);
            lat = latitude + (d.z / EarthRadius) * Mathf.Rad2Deg;
            lon = longitude + (d.x / (EarthRadius * CosLat)) * Mathf.Rad2Deg;
            alt = altitude + d.y;
        }

        public Vector3 FromGeodetic(double lat, double lon, double alt)
        {
            float north = (float)((lat - latitude) * Mathf.Deg2Rad * EarthRadius);
            float east = (float)((lon - longitude) * Mathf.Deg2Rad * EarthRadius * CosLat);
            float up = (float)(alt - altitude);
            return transform.position + new Vector3(east, up, north);
        }

        /// <summary>Earth's field at the origin (NED, gauss), from the table PX4 checks its compass against.</summary>
        public Vector3 MagneticFieldNed => WorldMagneticModel.FieldNed(latitude, longitude);

        /// <summary>Earth's field at the origin as a Unity world vector (east, up, north), gauss.</summary>
        public Vector3 MagneticFieldUnity => NedToUnity(MagneticFieldNed);

        public static Vector3 NedToUnity(Vector3 ned) => new Vector3(ned.y, -ned.z, ned.x);
    }
}
