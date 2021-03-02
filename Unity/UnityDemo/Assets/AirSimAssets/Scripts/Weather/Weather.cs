using System.Collections.Generic;
using UnityEngine;

namespace AirSimUnity
{
    /// <summary>
    /// Stores global weather settings and applies them to instances of weather effects in the scene.
    /// </summary>
    public class Weather : MonoBehaviour
    {
        public enum WeatherParamScalar {
            Snow,
            Count
        }

        [Tooltip("WeatherFX prefab that will be spawned as a child of each vehicle.")]
        [SerializeField] private WeatherFX WeatherFXPrefab = default;

        private bool isEnabled;
        private float[] weatherParamScalars = new float[(int)WeatherParamScalar.Count];
        private List<WeatherFX> weatherFXInstances = new List<WeatherFX>();

        public bool IsWeatherEnabled
        {
            get { return isEnabled; }

            set {
                isEnabled = value;
                RemoveNullWeatherFXInstances();
                foreach (WeatherFX weatherFX in weatherFXInstances) {
                    weatherFX.gameObject.SetActive(value);
                }
            }
        }

        public WeatherParamScalarCollection ParamScalars { get; private set; } = new WeatherParamScalarCollection();

        /// <summary>
        /// Removes all weatherFX from the list that have been destroyed (references set to null by Unity).
        /// </summary>
        private void RemoveNullWeatherFXInstances() {
            weatherFXInstances.RemoveAll(delegate (WeatherFX weatherFX) { return weatherFX == null; });
        }

        /// <summary>
        /// Attach an instance of WeatherFX to the vehicle.
        /// This is necessary because it would be computationally inefficient to render a weather particle system
        /// across a potentially very large environment. Instead, we render weather particles in the local area
        /// of each vehicle. In practice, this produces convincing weather effects because particles in the
        /// distance wouldn't be noticeable.
        /// </summary>
        /// <param name="vehicle">Vehicle to which weather effects will be attached.</param>
        public void AttachToVehicle(Vehicle vehicle) {
            if (WeatherFXPrefab != null) {
                WeatherFX weatherFX = Instantiate(WeatherFXPrefab, vehicle.transform);
                weatherFX.gameObject.SetActive(isEnabled);

                // Apply weather effect at absolute world scale regardless of the parent vehicle's scale.
                // Note that this won't work if the parent is unevenly stretched along different axes.
                Vector3 worldScale = weatherFX.transform.lossyScale;
                weatherFX.transform.localScale = Vector3.Scale(weatherFX.transform.localScale, new Vector3(1.0f / worldScale.x, 1.0f / worldScale.y, 1.0f / worldScale.z));

                weatherFXInstances.Add(weatherFX);
            }
        }
    }
}
