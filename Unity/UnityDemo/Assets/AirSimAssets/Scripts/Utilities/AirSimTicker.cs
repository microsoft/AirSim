using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AirSimUnity
{
    public class AirSimTicker : MonoBehaviour
    {
        void FixedUpdate()
        {
            PInvokeWrapper.CallTick(Time.fixedDeltaTime);
        }
    }
}