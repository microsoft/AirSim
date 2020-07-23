using AirSimUnity.DroneStructs;

namespace AirSimUnity {
    /*
     * Contract to be implemented by Unity's vehicle.
     */

    public interface IVehicleInterface {

        bool SetPose(AirSimPose pose, bool ignoreCollision);

        AirSimPose GetPose();

        CollisionInfo GetCollisionInfo();

        AirSimRCData GetRCData();

        ImageResponse GetSimulationImages(ImageRequest request);

        UnityTransform GetTransform();

        bool SetRotorSpeed(int rotorIndex, RotorInfo rotorInfo);

        bool SetCarControls(CarStructs.CarControls controls);

        CarStructs.CarState GetCarState();

        bool SetEnableApi(bool enableApi);

        DataRecorder.ImageData GetRecordingData();

        DataCaptureScript GetCameraCapture(string cameraName);

        CameraInfo GetCameraInfo(string cameraName);

        bool SetCameraPose(string cameraName, AirSimPose pose);

        bool SetCameraFoV(string cameraName, float fov_degrees);

        bool PrintLogMessage(string message, string messageParams, string vehicleName, int severity);

        void ResetVehicle();

        AirSimVector GetVelocity();

        RayCastHitResult GetRayCastHit(AirSimVector start, AirSimVector end);

        bool Pause(float timeScale);
    }
}
