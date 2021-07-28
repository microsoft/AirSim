using UnityEngine;

namespace AirSimUnity
{
    /// <summary>
    /// Renders an instance of weather visual effects.
    /// </summary>
    public class WeatherFX : MonoBehaviour
    {
        [SerializeField] private ParticleSystem snowParticleSystem = default;
        private float snowParticleSystemMaximumRate;

        private void OnValidate() {
            Debug.Assert(snowParticleSystem != null);
        }

        private void Start() {
            snowParticleSystemMaximumRate = snowParticleSystem.emission.rateOverTime.constant;
        }

        private void Update() {
            Weather weather = AirSimGlobal.Instance.Weather;
            if (!weather.IsWeatherEnabled)
                return;

            ParticleSystem.EmissionModule emission = snowParticleSystem.emission;
            emission.rateOverTime = weather.ParamScalars[WeatherParamScalarCollection.WeatherParamScalar.Snow] * snowParticleSystemMaximumRate;
        }
    }
}
