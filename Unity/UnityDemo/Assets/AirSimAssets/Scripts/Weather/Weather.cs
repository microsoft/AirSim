using System.Collections.Generic;
using UnityEngine;

namespace AirSimUnity
{
    /// <summary>
    /// Stores global weather settings and applies them to instances of weather effects in the scene.
    /// </summary>
    public class Weather : MonoBehaviour
    {
        [Tooltip("WeatherFX prefab that will be spawned as a child of each vehicle.")]
        [SerializeField] private WeatherFX WeatherFXPrefab;

        private bool isEnabled;
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

        /// <summary>
        /// Removes all weatherFX from the list that have been destroyed (references set to null by Unity).
        /// </summary>
        private void RemoveNullWeatherFXInstances() {
            weatherFXInstances.RemoveAll(delegate (WeatherFX weatherFX) { return weatherFX == null; });
        }

        public void AttachToVehicle(Vehicle vehicle) {
            if (WeatherFXPrefab != null) {
                WeatherFX weatherFX = Instantiate(WeatherFXPrefab, vehicle.transform);
                weatherFX.gameObject.SetActive(isEnabled);
                weatherFXInstances.Add(weatherFX);
            }
        }
    }
}
