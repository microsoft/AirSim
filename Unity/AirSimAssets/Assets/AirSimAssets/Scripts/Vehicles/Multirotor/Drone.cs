using System;
using System.Collections.Generic;
using AirSimUnity.DroneStructs;
using UnityEngine;

namespace AirSimUnity {
    /*
     * Drone component that is used to control the drone object in the scene. This is based on Vehicle class that is communicating with AirLib.
     * This class depends on the AirLib's drone controllers based on FastPhysics engine. The controller is being used based on setting.json file in Documents\AirSim
     * The drone can be controlled either through keyboard or through client api calls.
     * This data is being constantly exchanged between AirLib and Unity through PInvoke delegates.
     */
    public class Drone : Vehicle {
        public Transform[] rotors;
        private List<RotorInfo> rotorInfos = new List<RotorInfo>();
        private float rotationFactor = 0.1f;
        
        private new void Start() {
            base.Start(); 

            for (int i = 0; i < rotors.Length; i++) {
                rotorInfos.Add(new RotorInfo());
            }
        }

        private new void FixedUpdate() {
            if (isServerStarted)
            {
                if (resetVehicle)
                {
                    rcData.Reset();
                    currentPose = poseFromAirLib;
                    resetVehicle = false;
                }

                base.FixedUpdate();

                DataManager.SetToUnity(poseFromAirLib.position, ref position);
                DataManager.SetToUnity(poseFromAirLib.orientation, ref rotation);

                transform.position = position;
                transform.rotation = rotation;

                for (int i = 0; i < rotors.Length; i++)
                {
                    float rotorSpeed = (float) (rotorInfos[i].rotorSpeed * rotorInfos[i].rotorDirection * 180 /
                                                Math.PI * rotationFactor);
                    rotors[i].Rotate(Vector3.up, rotorSpeed * Time.deltaTime, Space.Self);
                }
            }
        }

        private new void LateUpdate() {
            if (isServerStarted)
            {
                if (!isApiEnabled)
                {
                    rcData.is_valid = true;
                    rcData.roll = Input.GetAxis("Horizontal");
                    rcData.pitch = Input.GetAxis("Vertical");
                    rcData.throttle = Input.GetAxis("Depth");
                    rcData.yaw = Input.GetAxis("Yaw");
                    //rcData.left_z = Input.GetAxis("");
                    //rcData.right_z = Input.GetAxis("");

                    rcData.switch1 = (uint) (Input.GetKeyDown("joystick button 0") ? 1 : 0);
                    rcData.switch2 = (uint) (Input.GetKeyDown("joystick button 1") ? 1 : 0);
                    rcData.switch3 = (uint) (Input.GetKeyDown("joystick button 2") ? 1 : 0);
                    rcData.switch4 = (uint) (Input.GetKeyDown("joystick button 3") ? 1 : 0);
                    rcData.switch5 = (uint) (Input.GetKeyDown("joystick button 4") ? 1 : 0);
                    rcData.switch6 = (uint) (Input.GetKeyDown("joystick button 5") ? 1 : 0);
                    rcData.switch7 = (uint) (Input.GetKeyDown("joystick button 6") ? 1 : 0);
                    rcData.switch8 = (uint) (Input.GetKeyDown("joystick button 7") ? 1 : 0);
                }

                //Image capture is being done in base class
                base.LateUpdate();
            }
        }

        public KinemticState GetKinematicState() {
            return airsimInterface.GetKinematicState();
        }

        #region IVehicleInterface implementation

        // Sets the animation for rotors on the drone. This is being done by AirLib through Pinvoke calls
        public override bool SetRotorSpeed(int rotorIndex, RotorInfo rotorInfo) {
            rotorInfos[rotorIndex] = rotorInfo;
            return true;
        }

        //Gets the data specific to drones for saving in the text file along with the images at the time of recording
        public override DataRecorder.ImageData GetRecordingData() {
            DataRecorder.ImageData data;
            data.pose = currentPose;
            data.carData = new CarStructs.CarData();
            data.image = null;
            return data;
        }

        #endregion IVehicleInterface implementation
    }
}