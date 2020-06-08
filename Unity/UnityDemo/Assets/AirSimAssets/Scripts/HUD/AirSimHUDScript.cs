using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace AirSimUnity {
    /*
     * HUD class responsible for all the UI related elements. The Settings initialization is also being done in this class.
     * To Enable UI in the simulation, make sure this class is attached to one of the game objects in the scene.
     * Else use the AirSimHUD prefab in the scene which has all the UI related components.
     */

    public class AirSimHUDScript : MonoBehaviour {
        private readonly GameObject[] cameraViews = new GameObject[3];
        private bool isViewCamerasSet;

        private void Awake() {
            //Needs to be the first initialization in the Simulation if not done.
            if (AirSimSettings.GetSettings() == null)
            {
                AirSimSettings.Initialize();
            }
        }

        private void Start() {
            SetUpViewCameras();
        }

        private void LateUpdate() {
            //Input setup for toggling the Window views on the screen.
            if (Input.GetKeyDown(KeyCode.Alpha0) & isViewCamerasSet)
            {
                for (int i = 0; i < 3; i++)
                {
                    cameraViews[i].SetActive(!cameraViews[i].activeInHierarchy);
                }
            }
            else if (Input.GetKeyDown(KeyCode.Alpha1) & isViewCamerasSet)
            {
                cameraViews[0].SetActive(!cameraViews[0].activeInHierarchy);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha2) & isViewCamerasSet)
            {
                cameraViews[1].SetActive(!cameraViews[1].activeInHierarchy);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha3) & isViewCamerasSet)
            {
                cameraViews[2].SetActive(!cameraViews[2].activeInHierarchy);
            }
            else if (Input.GetKeyDown(KeyCode.Escape))
            {
                Application.Quit();
            }
        }


        //This method is linked through OnClick for RecordButton in AirSimHUD prefab.
        public void ToggleRecording() {
            VehicleCompanion.GetCameraCaptureForRecording().ToggleRecording();
        }

        //Sets up the cameras based on the settings.json file to render on the sub windows.
        //Make sure the cameras are child of object with tag "ViewCameras"
        public void SetUpViewCameras() {
            GameObject viewCameras = GameObject.FindGameObjectWithTag("ViewCameras");

            cameraViews[0] = transform.GetChild(0).gameObject;
            cameraViews[1] = transform.GetChild(1).gameObject;
            cameraViews[2] = transform.GetChild(2).gameObject;

            cameraViews[0].SetActive(false);
            cameraViews[1].SetActive(false);
            cameraViews[2].SetActive(false);

            if (!viewCameras) {
                isViewCamerasSet = false;
                return;
            }

            isViewCamerasSet = viewCameras.activeInHierarchy;

            if (isViewCamerasSet) {
                for (int i = 0; i < 3; i++) {
                    AirSimSettings.SubWindowsSettings windowSettings = AirSimSettings.GetSettings().SubWindows[i];
                    ImageType imageType = (ImageType)windowSettings.ImageType;
                    viewCameras.transform.GetChild(windowSettings.WindowID).GetComponent<CameraFiltersScript>().SetShaderEffect(imageType);
                    cameraViews[windowSettings.WindowID].SetActive(windowSettings.Visible);
                    if (imageType == ImageType.DepthVis) {
                        cameraViews[windowSettings.WindowID].transform.GetChild(0).gameObject.SetActive(true);
                    }
                }
            }
        }
    }
}
