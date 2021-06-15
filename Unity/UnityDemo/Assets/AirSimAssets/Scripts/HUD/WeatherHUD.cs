using UnityEngine;
using UnityEngine.UI;

namespace AirSimUnity {
    /// <summary>
    /// HUD class that manage weather-related UI elements.
    /// </summary>
    public class WeatherHUD : MonoBehaviour {
        [SerializeField] private RectTransform rootPanel = default;
        [SerializeField] private Toggle weatherEnabledToggle = default;
        [SerializeField] private WeatherHUDSlider snowSlider = default;

        private void Start() {
            rootPanel.gameObject.SetActive(false);
            weatherEnabledToggle.onValueChanged.AddListener(OnWeatherEnabledToggleChanged);
            snowSlider.OnValueChanged.AddListener(OnSnowSliderChanged);
        }

        private void OnValidate() {
            Debug.Assert(rootPanel != null);
            Debug.Assert(weatherEnabledToggle != null);
            Debug.Assert(snowSlider != null);
        }

        private void Update() {
            if (Input.GetKeyDown(KeyCode.F10)) {
                rootPanel.gameObject.SetActive(!rootPanel.gameObject.activeSelf);
            }

            if (!rootPanel.gameObject.activeSelf) {
                return;
            }

            Weather weather = AirSimGlobal.Instance.Weather;
            weatherEnabledToggle.isOn = weather.IsWeatherEnabled;
            snowSlider.Value = weather.ParamScalars[WeatherParamScalarCollection.WeatherParamScalar.Snow];
        }

        public void OnWeatherEnabledToggleChanged(bool isEnabled) {
            AirSimGlobal.Instance.Weather.IsWeatherEnabled = isEnabled;
        }

        private void OnSnowSliderChanged(float value) {
            AirSimGlobal.Instance.Weather.ParamScalars[WeatherParamScalarCollection.WeatherParamScalar.Snow] = value;
        }
    }
}
