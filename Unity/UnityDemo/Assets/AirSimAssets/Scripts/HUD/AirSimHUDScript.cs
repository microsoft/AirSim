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
        public GameObject VehiclePrefab;
        public SimMode simModeSelected;

        private readonly GameObject[] cameraViews = new GameObject[3];
        private bool isViewCamerasSet;

        private void Awake() {
            //Needs to be the first initialization in the Simulation if not done.
            if (AirSimSettings.GetSettings() == null)
            {
                Debug.Log("AirSimSettings not initialized.");
                AirSimSettings.Initialize();
                if(AirSimSettings.GetSettings().SimMode != "")
                {
                    AirSimSettings.GetSettings().ValidateSettingsForSimMode();
                }
                else
                {
                    Debug.Log("Notice: SimMode was not specified in 'settings.json' file. " +
                        "Using what was specified in AirSimHud game object property. for this scene.");
                    if (simModeSelected == SimMode.Car)
                    {
                        AirSimSettings.GetSettings().SimMode = "Car";
                    }
                    else if (simModeSelected == SimMode.Multirotor)
                    {
                        AirSimSettings.GetSettings().SimMode = "Multirotor";
                    }
                    AirSimSettings.GetSettings().ValidateSettingsForSimMode();
                }
            }

            var simMode = AirSimSettings.GetSettings().SimMode;
            InstantiateVehicle(simMode);            
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

        /// <summary>
        /// Will instantiate vehicle prefab(s) (currently only one as multiple vehicles not yet implemented)
        /// based on SimMode.  If SimMode is multirotor, we check for SimpleFlight first, then PX4 and instantiate
        /// the vehicle(s) if AutoCreate == true.
        /// The VehicleName (from settings.json) is assigned to each Vehicle object and that name is what is used
        /// by VehicleCompanion when to setting the vehicles companion name (which is used along with port number to
        /// interact with AirSim lib.)
        /// Also, we can use any settings pulled from the settings.json file to modify the initial environment or
        /// camera, or vehicle pose.  (As example the initial pose is set below)
        /// </summary>
        /// <param name="simmode"></param>
        private void InstantiateVehicle(string simmode)
        {
            if (simmode == "Car")
            {
                bool carCreated = false;
                foreach (var car in AirSimSettings.GetSettings().Vehicles.Vehicles_PhysXCar)
                {
                    if (car.AutoCreate == true)
                    {
                        carCreated = true;
                        GameObject go = Instantiate(VehiclePrefab) as GameObject;
                        if (go.GetComponent<Vehicle>() != null)
                        {
                            if (go.GetComponent<Vehicle>().SetVehicleName(car.VehicleName))
                            {
                                go.name = car.VehicleName;
                                if (!car.Position.HasNan())
                                {
                                    Vector3 pos = new Vector3(car.Position.X, car.Position.Y, car.Position.Z);
                                    go.transform.position = pos;
                                }
                                if (!car.Rotation.HasNan())
                                {
                                    Vector3 rot = new Vector3(car.Rotation.Pitch, car.Rotation.Yaw, car.Rotation.Roll);
                                    go.transform.eulerAngles = rot;
                                }
                            }
                            else
                            {
                                Debug.Log("Notice: AirSimHUD could not set vehicle name.");
                                Destroy(go);
                            }
                        }
                        else
                        {
                            Debug.Log("Notice: Did not find GetComponet<Vehicle> for this prefab.");
                            Destroy(go);
                        }
                    }
                    if (carCreated)
                    {
                        //Currently multiple vehicles not supported.  Only allowing one vehicle to be created.
                        break;
                    }
                }
            }
            else if (simmode == "Multirotor")
            {
                bool simpleFlightCreated = false;
                bool px4Created = false;
                bool computerVisionCreated = false;
                foreach (var sfDrone in AirSimSettings.GetSettings().Vehicles.Vehicles_SimpleFlight)
                {
                    if (sfDrone.AutoCreate == true)
                    {
                        simpleFlightCreated = true;
                        GameObject go = Instantiate(VehiclePrefab) as GameObject;
                        if (go.GetComponent<Vehicle>() != null)
                        {
                            if (go.GetComponent<Vehicle>().SetVehicleName(sfDrone.VehicleName))
                            {
                                go.name = sfDrone.VehicleName;
                                if (!sfDrone.Position.HasNan())
                                {
                                    Vector3 position = Vector3.zero;
                                    DataManager.SetToUnity(new AirSimVector(sfDrone.Position.X, sfDrone.Position.Y, sfDrone.Position.Z), ref position);
                                    //Vector3 pos = new Vector3(sfDrone.Position.X, sfDrone.Position.Y, sfDrone.Position.Z);
                                    go.transform.position = position;
                                }
                                if (!sfDrone.Rotation.HasNan())
                                {
                                    Vector3 rot = new Vector3(sfDrone.Rotation.Pitch, sfDrone.Rotation.Yaw, sfDrone.Rotation.Roll);
                                    go.transform.eulerAngles = rot;
                                }
                            }
                            else
                            {
                                Debug.Log("Notice: AirSimHUD could not set vehicle name.");
                                Destroy(go);
                            }
                        }
                        else
                        {
                            Debug.Log("Notice: Did not find GetComponet<Vehicle> for this prefab.");
                            Destroy(go);
                        }
                    }
                    if (simpleFlightCreated)
                    {
                        //Currently multiple vehicles not supported.  Only allowing one vehicle to be created.
                        break;
                    }
                }
                if (!simpleFlightCreated)
                {
                    foreach (var px4Drone in AirSimSettings.GetSettings().Vehicles.Vehicles_PX4Multirotor)
                    {
                        if (px4Drone.AutoCreate == true)
                        {
                            px4Created = true;
                            GameObject go = Instantiate(VehiclePrefab) as GameObject;
                            if (go.GetComponent<Vehicle>() != null)
                            {
                                if (go.GetComponent<Vehicle>().SetVehicleName(px4Drone.VehicleName))
                                {
                                    go.name = px4Drone.VehicleName;
                                    if (!px4Drone.Position.HasNan())
                                    {
                                        Vector3 pos = new Vector3(px4Drone.Position.X, px4Drone.Position.Y, px4Drone.Position.Z);
                                        go.transform.position = pos;
                                    }
                                    if (!px4Drone.Rotation.HasNan())
                                    {
                                        Vector3 rot = new Vector3(px4Drone.Rotation.Pitch, px4Drone.Rotation.Yaw, px4Drone.Rotation.Roll);
                                        go.transform.eulerAngles = rot;
                                    }
                                }
                                else
                                {
                                    Debug.Log("Notice: AirSimHUD could not set vehicle name.");
                                    Destroy(go);
                                }
                            }
                            else
                            {
                                Debug.Log("Notice: Did not find GetComponet<Vehicle> for this prefab.");
                                Destroy(go);
                            }
                        }
                        if (px4Created)
                        {
                            //Currently multiple vehicles not supported.  Only allowing one vehicle to be created.
                            break;
                        }
                    }
                }
                if (!simpleFlightCreated || !px4Created)
                {
                    // TODO: Add support (prefab) for ComputerVision
                }
                if (!simpleFlightCreated && !px4Created && !computerVisionCreated)
                {
                    Debug.Log("Notice: 'settings.json' file did not have AutoCreate true for any Vehicles.  No Vehicles created.");
                }
            }
        }

        //This method is linked through OnClick for RecordButton in AirSimHUD prefab.
        public void ToggleRecording() {
            VehicleCompanion.GetCameraCaptureForRecording().ToggleRecording();
        }

        //Sets up the cameras based on the settings.json file to render on the sub windows.
        //Make sure the cameras are child of object with tag "ViewCameras"
        public void SetUpViewCameras() {
            Debug.Log("Setting up ViewCameras.");
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
