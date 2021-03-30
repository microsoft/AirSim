using UnityEngine;
using UnityEngine.UI;

namespace AirSimUnity {
    /// <summary>
    /// HUD class that manage weather-related UI elements.
    /// </summary>
    public class WeatherHUDSlider : MonoBehaviour {
        [SerializeField] private Slider slider = default;
        [SerializeField] private InputField inputField = default;

        public Slider.SliderEvent OnValueChanged;

        public float Value
        { 
            get {
                return slider.value;
            }

            set {
                slider.value = value;
            }
        }

        private void OnValidate() {
            Debug.Assert(slider != null);
            Debug.Assert(inputField != null);
        }

        private void Start() {
            slider.onValueChanged.AddListener(OnSliderValueChanged);
            inputField.onEndEdit.AddListener(OnInputFieldValueEdited);
        }

        private void OnSliderValueChanged(float value) {
            inputField.text = value.ToString("P0").Replace(" ", ""); // percentage P0 format includes a space that we don't want
            OnValueChanged.Invoke(value);
        }

        private void OnInputFieldValueEdited(string value) {
            float floatValue;
            if (!float.TryParse(value.TrimEnd('%'), out floatValue)) {
                // Invalid text entered. Reset to the slider's value.
                OnSliderValueChanged(slider.value);
                return;
            }

            // Convert from percentage and assign to slider, triggering OnSliderValueChanged.
            slider.value = floatValue / 100.0f;
        }
    }
}
