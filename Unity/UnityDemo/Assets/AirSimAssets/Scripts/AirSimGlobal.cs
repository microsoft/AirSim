using UnityEngine;

namespace AirSimUnity
{
    /// <summary>
    /// Singleton that should be placed in the scene, providing a central point of access
    /// to global behaviors.
    /// </summary>
    public class AirSimGlobal : MonoBehaviour
    {
        public static AirSimGlobal Instance { get; private set; }

        public Weather Weather { get; private set; }

        private void Awake()
        {
            Instance = this;

            Weather = GetComponent<Weather>();
        }
    }
}
