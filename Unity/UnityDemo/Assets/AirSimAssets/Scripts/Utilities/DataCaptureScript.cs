using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace AirSimUnity {
    /*
     * Component attached to the camera for data capturing.
     * This component is being referred by the vehicles to collect and record data from the cameras present in
     * object with "captureCameras" tag.
     */

    [RequireComponent(typeof(Camera), typeof(CameraFiltersScript))]
    public class DataCaptureScript : MonoBehaviour {

        //Assigned based on settings.json file or the default id's just like in Unreal for recording data
        [AddComponentMenu("Camera Name")]
        public string cameraName;

        private float fov;

        private DataRecorder recorder;
        private RenderTexture renderTexture;
        private Texture2D screenShot;
        private Rect captureRect;
        private AirSimPose currentPose;
        private Pose poseFromAirLib;

        private bool isCapturing;
        private bool isPoseOverride;

        private WaitForSeconds waitUntilNext;
        private WaitForEndOfFrame waitForEndOfFrame;

        private Camera renderCam;

        private CameraFiltersScript shaderScript;

        private void FixedUpdate() {
            DataManager.SetToAirSim(transform.position, ref currentPose.position);
            DataManager.SetToAirSim(transform.rotation, ref currentPose.orientation);

            if (isPoseOverride) {
                transform.position = poseFromAirLib.position;
                transform.rotation = poseFromAirLib.rotation;
                isPoseOverride = false;
            }
        }

        private void OnApplicationQuit() {
            if (isCapturing) {
                recorder.StopRecording();
            }
        }

        public void SetUpCamera(string cameraName, bool isDrone) {
            this.cameraName = cameraName;

            renderCam = GetComponent<Camera>();
            fov = renderCam.fieldOfView;
            recorder = new DataRecorder();
            waitUntilNext = new WaitForSeconds(AirSimSettings.GetSettings().Recording.RecordInterval);
            waitForEndOfFrame = new WaitForEndOfFrame();
            shaderScript = GetComponent<CameraFiltersScript>();

            renderCam.enabled = false;
            shaderScript.effect = ImageType.Count;
            recorder.IsDrone(isDrone);
            SetUpRenderTextureForCapture(ImageType.Scene);
            SetUpRenderType(ImageType.Scene);
        }

        public void ResizeRenderTexture(int width, int height) {
            if (renderTexture != null) {
                renderTexture.Release();
            }
            renderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
            renderTexture.Create();
            renderCam.targetTexture = renderTexture;
        }

        public string GetCameraName() {
            return cameraName;
        }

        public float GetFOV() {
            return fov;
        }

        public AirSimPose GetPose() {
            return currentPose;
        }

        public void SetPose(AirSimPose pose) {
            DataManager.SetToUnity(pose.position, ref poseFromAirLib.position);
            DataManager.SetToUnity(pose.orientation, ref poseFromAirLib.rotation);
            isPoseOverride = true;
        }

        public void SetFoV(float fov_degrees) {
            fov = fov_degrees;
        }

        public void ToggleRecording() {
            isCapturing = !isCapturing;
            if (isCapturing) {
                recorder.StartRecording();
                StartCoroutine(CaptureFrames());
            } else {
                recorder.StopRecording();
            }
        }

        public ImageResponse GetImageBasedOnRequest(ImageRequest imageRequest) {
            if (imageRequest.pixels_as_float) {
                return new ImageResponse(null, GetFrameData(imageRequest.image_type), imageRequest.camera_name,
                    DataManager.ToAirSimVector(transform.position), DataManager.ToAirSimQuaternion(transform.rotation),
                    imageRequest.pixels_as_float, imageRequest.compress, GetImageWidth(), GetImageHeight(), imageRequest.image_type);
            } else {
                return new ImageResponse(GetFrameData(imageRequest.image_type, imageRequest.compress), null, imageRequest.camera_name,
                    DataManager.ToAirSimVector(transform.position), DataManager.ToAirSimQuaternion(transform.rotation),
                    imageRequest.pixels_as_float, imageRequest.compress, GetImageWidth(), GetImageHeight(), imageRequest.image_type);
            }
        }

        private int GetImageWidth() {
            return renderTexture.width;
        }

        private int GetImageHeight() {
            return renderTexture.height;
        }

        private List<byte> GetFrameData(ImageType type, bool isCompress) {
            SetUpRenderType(type);
            SetUpRenderTextureForCapture(type);

            RenderTexture.active = renderTexture;
            screenShot.ReadPixels(captureRect, 0, 0);
            screenShot.Apply();

            List<byte> frameData;

            if (isCompress) {
                byte[] bytes = screenShot.EncodeToPNG();
                frameData = new List<byte>(bytes);
            } else {
                Color32[] data = screenShot.GetPixels32();
                frameData = new List<byte>();
                foreach (Color32 c in data) {
                    frameData.Add(c.r);
                    frameData.Add(c.g);
                    frameData.Add(c.b);
                    // frameData.Add(c.a);     // Unreal is just sending RGB images, so don't include Alpha channel here
                }
            }
            RenderTexture.active = null;
            return frameData;
        }

        private List<float> GetFrameData(ImageType type) {
            SetUpRenderType(type);
            SetUpRenderTextureForCapture(type);
            RenderTexture.active = renderTexture;
            screenShot.ReadPixels(captureRect, 0, 0);
            screenShot.Apply();
            Color[] data = screenShot.GetPixels();
            RenderTexture.active = null;
            List<float> frameData = new List<float>();
            foreach (Color c in data) {
                frameData.Add(c.r);
                //frameData.Add(c.g);  //Need to Confirm with Microsoft regarding the other data, as Unreal is just sending the R data.
                //frameData.Add(c.b);
                //frameData.Add(c.a);
            }
            return frameData;
        }

        //Thread to capture the images and save them to documents. This will be started by pressing the Record button on HUD.
        private IEnumerator CaptureFrames() {
            AirSimSettings.CamerasSettings recordCamSettings = AirSimSettings.GetSettings().Recording.Cameras[0];
            SetUpRenderType((ImageType)recordCamSettings.ImageType);
            SetUpRenderTextureForCapture((ImageType)recordCamSettings.ImageType);
            while (isCapturing) {
                yield return waitForEndOfFrame;
                RenderTexture.active = renderTexture;
                screenShot.ReadPixels(captureRect, 0, 0);
                screenShot.Apply();
                byte[] bytes = screenShot.EncodeToPNG();
                RenderTexture.active = null;
                DataRecorder.ImageData data = VehicleCompanion.GetRecordingData();
                data.image = bytes;
                recorder.AddImageDataToQueue(data);
                yield return waitUntilNext;
            }
        }

        private void SetUpRenderType(ImageType type) {
            if (shaderScript.effect == type) {
                return;
            }
            shaderScript.SetShaderEffect(type);
        }

        private void SetUpRenderTextureForCapture(ImageType type) {
            if (shaderScript.effect == type) {
                return;
            }

            AirSimSettings.CameraCaptureSettings captureSettings = AirSimSettings.GetSettings().GetCaptureSettingsBasedOnImageType(type);

            renderTexture = RenderTexture.GetTemporary(captureSettings.Width, captureSettings.Height, 24, RenderTextureFormat.ARGB32);

            renderTexture.Create();
            renderCam.targetTexture = renderTexture;

            renderCam.enabled = true;

            screenShot = new Texture2D(captureSettings.Width, captureSettings.Height, TextureFormat.RGB24, false);
            captureRect = new Rect(0, 0, captureSettings.Width, captureSettings.Height);
        }
    }
}
