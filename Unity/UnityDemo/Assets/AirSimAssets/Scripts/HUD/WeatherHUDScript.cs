using UnityEditor.VersionControl;
using UnityEngine;
using UnityEngine.UI;

namespace AirSimUnity {
    /// <summary>
    /// HUD class that manage weather-related UI elements.
    /// </summary>
    public class WeatherHUDScript : MonoBehaviour {
        [SerializeField] private RectTransform rootPanel;
        [SerializeField] private Toggle weatherEnabledToggle;

        private void Start() {
            rootPanel.gameObject.SetActive(false);
            weatherEnabledToggle.onValueChanged.AddListener(OnWeatherEnabledChanged);
        }

        private void OnValidate() {
            Debug.Assert(rootPanel != null);
            Debug.Assert(weatherEnabledToggle != null);
        }

        private void Update() {
            if (Input.GetKeyDown(KeyCode.F10)) {
                rootPanel.gameObject.SetActive(!rootPanel.gameObject.activeSelf);
            }

            if (!rootPanel.gameObject.activeSelf)
                return;

            Weather weather = AirSimGlobal.Instance.Weather;
            weatherEnabledToggle.isOn = weather.IsWeatherEnabled;
        }

        public void OnWeatherEnabledChanged(bool isEnabled) {
            AirSimGlobal.Instance.Weather.IsWeatherEnabled = isEnabled;
        }
    }
}
