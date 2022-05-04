namespace AirSimUnity
{
    /// <summary>
    /// Stores a set of float weather settings and applies them to instances of weather effects in the scene.
    /// </summary>
    public class WeatherParamScalarCollection
    {
        public enum WeatherParamScalar
        {
            Snow,
            Count
        }

        private float[] values = new float[(int)WeatherParamScalar.Count];

        public float this[WeatherParamScalar index]
        {
            get => values[(int)index];
            set => values[(int)index] = value;
        }
    }
}
