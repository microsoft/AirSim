using UnityEngine;
using System;
using AirSimUnity.CarStructs;

namespace AirSimUnity {
    /*
     * Adaptor to transform data between AirLib and Unity.
     */
    public class DataManager {

        public static Vector3 ToUnityVector(AirSimVector src) {
            Vector3 vector = new Vector3();
            SetToUnity(src, ref vector);
            return vector;
        }

        public static AirSimVector ToAirSimVector(Vector3 src) {
            AirSimVector vector = new AirSimVector();
            SetToAirSim(src, ref vector);
            return vector;
        }

        public static Quaternion ToUnityQuaternion(AirSimQuaternion src) {
            Quaternion quat = new Quaternion();
            SetToUnity(src, ref quat);
            return quat;
        }

        public static AirSimQuaternion ToAirSimQuaternion(Quaternion src) {
            AirSimQuaternion quat = new AirSimQuaternion();
            SetToAirSim(src, ref quat);
            return quat;
        }

        public static void SetToUnity(AirSimVector src, ref Vector3 dst) {
            dst.Set(src.y, -src.z, src.x);
        }

        public static void SetToAirSim(Vector3 src, ref AirSimVector dst) {
            dst.Set(src.z, src.x, -src.y);
        }

        public static void SetToUnity(AirSimQuaternion src, ref Quaternion dst) {
            dst.Set(-src.y, src.z, -src.x, src.w);
        }

        public static void SetToAirSim(Quaternion src, ref AirSimQuaternion dst) {
            dst.Set(src.z, -src.x, -src.y, src.w);
        }

        public static void SetCarControls(CarControls src, ref CarControls dst) {
            dst.brake = src.brake;
            dst.gear_immediate = src.gear_immediate;
            dst.handbrake = src.handbrake;
            dst.is_manual_gear = src.is_manual_gear;
            dst.manual_gear = src.manual_gear;
            dst.steering = src.steering;
            dst.throttle = src.throttle;
        }

        public static long GetCurrentTimeInMilli() {
            return DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        }
    }
}