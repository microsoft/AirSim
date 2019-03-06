using System;
using UnityEngine;
using System.IO;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using UnityEditor;
using SimpleJSON;

namespace AirSimUnity {
    /*
     * Container class for settings.json file. Provides:
     * 1. Initialization of all default settings
     * 2. Read settings.json user file and merge settings with all defaults
     * 3. Validation and final check (for conflicts) of settings once SimMode is known.
     *
     * Current file location is Documents\AirSim\settings.json
     *
     * NOTE : All the variable names has to match the attribute names(case sensitive) in the json file
     * struct  names are customizable unlike variable names, and make sure all the structs are serializable.
     */

    public class AirSimSettings {

        #region Serializable Structs
        [Serializable]
        public struct RecordingSettings {
            public bool RecordOnMove;
            public double RecordInterval;
            public List<CamerasSettings> Cameras;
        }

        [Serializable]
        public struct CamerasSettings {
            public string CameraName;
            public int ImageType;
            public bool PixelAsFloat;
            public bool Compress;
            public List<CaptureSettingsSettings> CaptureSettings;
        }

        [Serializable]
        public struct CaptureSettingsSettings {
            public int ImageType;
            public int Width;
            public int Height;
            public int FOV_Degrees;
            public int AutoExposureSpeed;
            public int AutoExposureBias;
            public double AutoExposureMaxBrightness;
            public double AutoExposureMinBrightness;
            public int MotionBlurAmount;
            public double TargetGamma;
            public string ProjectionMode;
            public double OrthoWidth;
        }

        [Serializable]
        public struct NoiseSettingsSettings {
            public bool Enabled;
            public int ImageType;
            public double RandContrib;
            public double RandSpeed;
            public double RandSize;
            public double RandDensity;
            public double HorzWaveContrib;
            public double HorzWaveStrength;
            public double HorzWaveVertSize;
            public double HorzWaveScreenSize;
            public double HorzNoiseLinesContrib;
            public double HorzNoiseLinesDensityY;
            public double HorzNoiseLinesDensityXY;
            public double HorzDistortionContrib;
            public double HorzDistortionStrength;
        }

        [Serializable]
        public struct GimbalSettings
        {
            public int Stabilization;
            public Rotation Rotation;
        }

        [Serializable]
        public struct CameraDefaultsSettings {
            public List<CaptureSettingsSettings> CaptureSettings;
            public List<NoiseSettingsSettings> NoiseSettings;
            public GimbalSettings Gimbal;
            public Position Position;
            public Rotation Rotation;
        }

        [Serializable]
        public struct OriginGeopointSettings {
            public double Latitude;
            public double Longitude;
            public double Altitude;
        }

        [Serializable]
        public struct TimeOfDaySettings {
            public bool Enabled;
            public string StartDateTime;
            public double CelestialClockSpeed;
            public bool StartDateTimeDst;
            public double UpdateIntervalSecs;
            public bool MoveSun;
        }

        [Serializable]
        public struct SubWindowsSettings {
            public int WindowID;
            public string CameraName;
            public int ImageType;
            public bool Visible;
        }

        [Serializable]
        public struct SegmentationSettingsSettings {
            public string InitMethod;
            public string MeshNamingMethod;
            public bool OverrideExisting;
        }

        [Serializable]
        public struct PawnPath {
            public string PawnBP;
            public string SlipperyMat;
            public string NonSlipperyMat;
        }

        [Serializable]
        public struct PawnPathsSettings {
            public PawnPath BareboneCar;
            public PawnPath DefaultCar;
            public PawnPath DefaultQuadrotor;
            public PawnPath DefaultComputerVision;
        }

        [Serializable]
        public struct SimpleFlightSettings {
            public string VehicleName;
            public string VehicleType;
            public string DefaultVehicleState;
            public bool AutoCreate;
            public bool EnableTrace;  // Missing from Settings.doc
            public string PawnPath;
            public bool EnableCollisionPassthrough;  // Is there a typo in Settings.doc ?
            public bool EnableCollisions;
            public bool IsFpvVehicle; // Missing from Settings.doc
            public bool AllowAPIAlways;
            public RCSettings RC;
            public CamerasSettings Cameras;
            public Position Position;
            public Rotation Rotation;
            public DefaultSensorsSettings Sensors;
        }

        [Serializable]
        public struct PhysXCarSettings {
            public string VehicleName;
            public string VehicleType;
            public string DefaultVehicleState;
            public bool AutoCreate;
            public bool EnableTrace;  // Missing from Settings.doc
            public string PawnPath;
            public bool EnableCollisionPassthrogh;  // Is there a typo in Settings.doc ?
            public bool EnableCollisions;
            public RCSettings RC;
            public CamerasSettings Cameras;
            public Position Position;
            public Rotation Rotation;
            public DefaultSensorsSettings Sensors;
        }

        [Serializable]
        public struct ComputerVisionSettings
        {
            public string VehicleName;
            public string VehicleType;
            public string DefaultVehicleState;
            public bool AutoCreate;
            public bool EnableTrace;  // Missing from Settings.doc
            public string PawnPath;
            public bool EnableCollisionPassthrogh;  // Is there a typo in Settings.doc ?
            public bool EnableCollisions;
            public bool IsFpvVehicle; // Missing from Settings.doc
            public bool AllowAPIAlways;
            public RCSettings RC;
            public CamerasSettings Cameras;
            public Position Position;
            public Rotation Rotation;
        }

        [Serializable]
        public struct PX4MultirotorSettings {
            public string VehicleName;
            public string VehicleType;
            //PX4 Specific
            public string LogViewerHostIp;
            public int LogViewerPort;
            //public int LogViewerSendPort; // Missing from Settings.doc
            public int OffboardCompID; // Missing from Settings.doc
            public int OffboardSysID;
            public string QgcHostIp;
            public int QgcPort;
            public int SerialBaudRate;
            public string SerialPort;
            public int SimCompID;
            public int SimSysID;
            public string SitlIp;
            public int SitlPort;
            public string UdpIp;
            public int UdpPort;
            public bool UseSerial;
            public int VehicleCompID;
            public int VehicleSysID;
            public string Model;
            public string LocalHostIp;
            // End PX4 Specific
            public string DefaultVehicleState;
            public bool AutoCreate;
            public bool EnableTrace;  // Missing from Settings.doc
            public string PawnPath;
            public bool EnableCollisionPassthrogh;  // Is there a typo in Settings.doc ?
            public bool EnableCollisions;
            public bool IsFpvVehicle;
            public bool AllowAPIAlways;
            public RCSettings RC;
            public CamerasSettings Cameras;
            public Position Position;
            public Rotation Rotation;
            public DefaultSensorsSettings Sensors;
        }

        [Serializable]
        public struct RCSettings {
            public int RemoteControlID;
            public bool AllowAPIWhenDisconnected;
        }

        [Serializable]
        public struct VehiclesSettings {
            public List<SimpleFlightSettings> Vehicles_SimpleFlight;
            public List<PhysXCarSettings> Vehicles_PhysXCar;
            public List<PX4MultirotorSettings> Vehicles_PX4Multirotor;
            public List<ComputerVisionSettings> Vehicles_ComputerVision;
        }

        [Serializable]
        public struct BarometerSettings {
            public SensorType SensorType;
            public bool Enabled;
        }

        [Serializable]
        public struct ImuSettings
        {
            public SensorType SensorType;
            public bool Enabled;
        }

        [Serializable]
        public struct GpsSettings {
            public SensorType SensorType;
            public bool Enabled;
        }

        [Serializable]
        public struct MagnetometerSettings
        {
            public SensorType SensorType;
            public bool Enabled;
        }

        [Serializable]
        public struct DistanceSettings
        {
            public SensorType SensorType;
            public bool Enabled;
        }

        [Serializable]
        public struct LidarSettings {
            public SensorType SensorType;
            public bool Enabled;
            public int NumberOfChannels;
            public float Range;
            public int RotationsPerSecond;
            public int PointsPerSecond;
            public Position Position;
            public Rotation Rotation;
            public float VerticalFOVUpper;
            public float VerticalFOVLower;
            public float HorizontalFOVStart;
            public float HorizontalFOVEnd;
            public bool DrawDebugPoints;
            public string DataFrame;
        }

        [Serializable]
        public struct DefaultSensorsSettings {
            public List<BarometerSettings> BarometerList;
            public List<ImuSettings> ImuList;
            public List<GpsSettings> GpsList;
            public List<MagnetometerSettings> MagnetometerList;
            public List<DistanceSettings> DistanceList;
            public List<LidarSettings> LidarList;
        }

        [Serializable]
        public struct CameraDirectorSettings
        {
            public Position Position;
            public Rotation Rotation;
            public float FollowDistance;
        }
        #endregion Structs

        #region Public Fields
        /// <summary>
        /// Fields to hold current state of AirSimSettings
        /// </summary>
        [SerializeField]
        public double SettingsVersion;
        [SerializeField]
        public string SimMode;
        public string ClockType;
        public double ClockSpeed;
        public string LocalHostIp; // No longer used?
        public string ApiServerAddress;  // No longer part of settings.json file format
        public bool RecordUIVisible;
        public bool LogMessagesVisible;
        public string ViewMode;
        public bool RpcEnabled;
        public bool EngineSound;
        public string PhysicsEngineName; // No longer used?
        public double SpeedUnitFactor;
        public string SpeedUnitLabel;
        public RecordingSettings Recording;
        public CameraDefaultsSettings CameraDefaults;
        public OriginGeopointSettings OriginGeopoint; //The geo-coordinate assigned to Unreal coordinate 0,0,0
        public TimeOfDaySettings TimeOfDay;
        public List<SubWindowsSettings> SubWindows;
        public SegmentationSettingsSettings SegmentationSettings;
        public PawnPathsSettings PawnPaths;
        public VehiclesSettings Vehicles;
        public DefaultSensorsSettings DefaultSensors; // Default sensors
        public CameraDirectorSettings CameraDirector; // New settings?
        #endregion
        public int InitialViewMode;

        /// <summary>
        /// Private Static Field
        /// </summary>
        private static AirSimSettings _settings;
        private const int DEFAULT_DRONE_PORT = 41451;
        private const int DEFAULT_CAR_PORT = 41451;

        public static AirSimSettings GetSettings()
        {
            return _settings;
        }

        public static void Initialize()
        {
            _settings = new AirSimSettings();
            _settings.InitializeDefaults();
            _settings.MergeWithUserSettings();
        }

        public bool ValidateSettingsForSimMode()
        {
            bool NoError = true;

            // Make sure user is not mixing simpleflight and px4
            if (SimMode == "Multirotor")
            {
                if (Vehicles.Vehicles_PX4Multirotor.Count > 0)
                {
                    foreach (var px4 in Vehicles.Vehicles_PX4Multirotor)
                    {
                        if (px4.AutoCreate)
                        {
                            if (Vehicles.Vehicles_SimpleFlight.Count > 0)
                            {
                                foreach (var simpleflight in Vehicles.Vehicles_SimpleFlight)
                                {
                                    if (simpleflight.AutoCreate)
                                    {
                                        Debug.Log("Notice: 'settings.json' file does not currently allow AutoCreate true for both PX4Multirotor " +
                                            "and SimpleFlight. Please set AutoCreate to false for one of them.");
                                        NoError = false;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Update ViewMode based on SimMode
            if (ViewMode == "")
            {
                switch (SimMode)
                {
                    case "Multirotor":
                        ViewMode = "FlyWithMe";
                        break;
                    case "ComputerVision":
                        ViewMode = "Fpv";
                        break;
                    default:
                        ViewMode = "SpringArmChase";
                        break;
                }
            }

            // Not sure if/when this will be implemented in Unity, but stubbed out here
            switch (ViewMode)
            {
                case "Fpv":
                    InitialViewMode = 1;
                    break;
                case "GroundObserver":
                    InitialViewMode = 2;
                    break;
                case "FlyWithMe":
                    InitialViewMode = 3;
                    break;
                case "Manual":
                    InitialViewMode = 4;
                    break;
                case "SpringArmChase":
                    InitialViewMode = 5;
                    break;
                case "Backup":
                    InitialViewMode = 6;
                    break;
                case "NoDisplay":
                    InitialViewMode = 7;
                    break;
                case "Front":
                    InitialViewMode = 8;
                    break;
                default:
                    NoError = false;
                    Debug.Log("Notice: 'settings.json' ViewMode setting '" + ViewMode + "' is not a valid view mode.");
                    break;
            }

            // Action on CameraDirector position based on specified FollowDistance
            // Not yet implemented in Unity yet so not sure if these numbers should be changed
            if (float.IsNaN(CameraDirector.FollowDistance))
            {
                if (SimMode == "Car")
                {
                    CameraDirector.FollowDistance = -8;
                }
                else
                {
                    CameraDirector.FollowDistance = -3;
                }
            }
            if (float.IsNaN(CameraDirector.Position.X))
            {
                CameraDirector.Position.X = CameraDirector.FollowDistance;
            }
            if (float.IsNaN(CameraDirector.Position.Y))
            {
                CameraDirector.Position.Y = 0;
            }
            if (float.IsNaN(CameraDirector.Position.Z))
            {
                if (SimMode == "Car")
                {
                    CameraDirector.Position.Z = -4;
                }
                else
                {
                    CameraDirector.Position.Z = -2;
                }
            }

            if (TimeOfDay.StartDateTime == "")
            {
                var localDate = DateTime.Now;
                string formattedDate = localDate.ToString("u", System.Globalization.CultureInfo.CreateSpecificCulture("en-US"));
                TimeOfDay.StartDateTime = formattedDate.TrimEnd('Z');
            }

            if (ClockType == "")
            {
                ClockType = "ScalableClock";
                if (SimMode == "Multirotor")
                {
                    //TODO: this won't work if simple_flight and PX4 is combined together!

                    //for multirotors we select steppable fixed interval clock unless we have
                    //PX4 enabled vehicle
                    ClockType = "SteppableClock";
                    if (Vehicles.Vehicles_PX4Multirotor.Count > 0)
                    {
                        foreach (var px4 in Vehicles.Vehicles_PX4Multirotor)
                        {
                            if (px4.AutoCreate)
                            {
                                ClockType = "ScalableClock";
                                if (Vehicles.Vehicles_SimpleFlight.Count > 0)
                                {
                                    // Shouldn't get here as we already checked for this above, but...
                                    foreach (var simpleflight in Vehicles.Vehicles_SimpleFlight)
                                    {
                                        if (simpleflight.AutoCreate)
                                        {
                                            Debug.Log("Notice: 'settings.json' file cannot currently support both SimpleFlight and PX4 simultaneously");
                                            NoError = false;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Make correction to lidar sensor default settings if simMode is Car
            if (SimMode == "Car")
            {
                var Vehicles_PhysXCar_List = new List<PhysXCarSettings>();
                foreach (var physXCarSetting in Vehicles.Vehicles_PhysXCar)
                {
                    var physXCar = physXCarSetting;
                    var lidarList = new List<LidarSettings>();

                    foreach (var lidarSetting in physXCarSetting.Sensors.LidarList)
                    {
                        var lidar = lidarSetting;
                        if (lidarSetting.VerticalFOVUpper == -15.0f)
                        {
                            lidar.VerticalFOVUpper = 10.0f;
                        }
                        if (lidarSetting.VerticalFOVLower == -45.0f)
                        {
                            lidar.VerticalFOVLower = -10.0f;
                        }
                        lidarList.Add(lidar);
                    }
                    
                    physXCar.Sensors.LidarList.Clear();
                    physXCar.Sensors.LidarList = lidarList;
                    Vehicles_PhysXCar_List.Add(physXCar);
                }
                Vehicles.Vehicles_PhysXCar.Clear();
                Vehicles.Vehicles_PhysXCar = Vehicles_PhysXCar_List;
            }

            // What else should we check?
            // Do we want to check for NaN Rotations or Positions?

            CreateSettingsFileWithAllMergedSettings("OKAY_AGAIN.json");

            return NoError;
        }

        public CaptureSettingsSettings GetCaptureSettingsBasedOnImageType(ImageType type)
        {
            if (CameraDefaults.CaptureSettings.Count == 1)
            {
                return CameraDefaults.CaptureSettings[0];
            }

            foreach (CaptureSettingsSettings capSettings in CameraDefaults.CaptureSettings)
            {
                if (capSettings.ImageType == (int)type)
                {
                    return capSettings;
                }
            }
            return CameraDefaults.CaptureSettings[0];
        }

        public int GetPortIDForVehicle(bool isDrone)
        {
            if (!isDrone)
            {
                return DEFAULT_CAR_PORT;
            }
            else
            {
                return DEFAULT_DRONE_PORT;
            }
        }
        /********** Methods internal to AirSimSettngs ********/

        /// <summary>
        /// Assign all possible airsim default settings
        /// </summary>
        private void InitializeDefaults()
        {
            // Top level settings
            SimMode = "";
            ClockType = "";
            ClockSpeed = 1.0;
            LocalHostIp = "127.0.0.1";  // No longer used?
            ApiServerAddress = ""; // No longer used?
            RecordUIVisible = true;
            LogMessagesVisible = true;
            ViewMode = "";
            InitialViewMode = 3; // Not a settings.json attribute ?
            RpcEnabled = true;
            EngineSound = true;
            PhysicsEngineName = ""; // No longer used?
            SpeedUnitFactor = 1.0;
            SpeedUnitLabel = "m//s";
            Recording = GetDefaultRecordingSettings();
            CameraDefaults = GetDefaultCameraDefaultSettings();
            OriginGeopoint = GetDefaultOriginGeopointSettings();
            TimeOfDay = GetDefaultTimeOfDaySettings();
            SubWindows = GetDefaultSubWindowsSettings();
            SegmentationSettings = GetDefaultSegmentationSettingsSettings();
            PawnPaths = GetDefaultPawnPathsSettings();
            Vehicles = new VehiclesSettings
            {
                Vehicles_SimpleFlight = new List<SimpleFlightSettings>
                {
                    GetDefaultSimpleFlightSettings()
                },
                Vehicles_PhysXCar = new List<PhysXCarSettings>
                {
                    GetDefaultPhysXCarSettings()
                },
                Vehicles_PX4Multirotor = new List<PX4MultirotorSettings>(),
                Vehicles_ComputerVision = new List<ComputerVisionSettings>()
            };
            CameraDirector = GetDefaultCameraDirectorSettings();
            DefaultSensors = GetDefaultDefaultSensorsSettingsDrone(); // Use drone settings for now - update in validation method once
        }

        /// <summary>
        /// Methods for producing default values for all the serialized structs
        /// </summary>
        /// <returns></returns>
        private static RecordingSettings GetDefaultRecordingSettings()
        {
            var recordingsettings = new RecordingSettings
            {
                RecordOnMove = false,
                RecordInterval = 0.05,
                Cameras = new List<CamerasSettings>
                {
                    GetDefaultCamerasSettings()
                }
            };
            return recordingsettings;
        }

        private static CamerasSettings GetDefaultCamerasSettings()
        {
            var camera = new CamerasSettings
            {
                CameraName = "0",
                ImageType = 0,
                PixelAsFloat = false,
                Compress = true,
                CaptureSettings = new List<CaptureSettingsSettings>
                {
                    GetDefaultCaptureSettingsSettings()
                }
            };
            return camera;
        }

        private static CameraDefaultsSettings GetDefaultCameraDefaultSettings()
        {
            var cameradefaults = new CameraDefaultsSettings
            {
                CaptureSettings = new List<CaptureSettingsSettings>
                {
                    GetDefaultCaptureSettingsSettings()
                },
                NoiseSettings = new List<NoiseSettingsSettings>
                {
                    GetDefaultNoiseSettingsSettings()
                },
                Gimbal = GetDefaultGimbalSettings(),
                Position = Position.NanPosition(),
                Rotation = Rotation.NanRotation()
            };
            return cameradefaults;
        }

        private static CaptureSettingsSettings GetDefaultCaptureSettingsSettings()
        {
            var capture = new CaptureSettingsSettings
            {
                ImageType = 0,
                Width = 256,
                Height = 144,
                FOV_Degrees = 90,
                AutoExposureSpeed = 100,
                AutoExposureBias = 0,
                AutoExposureMaxBrightness = 0.64,
                AutoExposureMinBrightness = 0.03,
                MotionBlurAmount = 0,
                TargetGamma = 1.0,
                ProjectionMode = "",
                OrthoWidth = 5.12
            };
            return capture;
        }

        private static NoiseSettingsSettings GetDefaultNoiseSettingsSettings()
        {
            var noise = new NoiseSettingsSettings
            {
                Enabled = false,
                ImageType = 0,
                RandContrib = 0.2,
                RandSpeed = 100000.0,
                RandSize = 500.0,
                RandDensity = 2.0,
                HorzWaveContrib = 0.03,
                HorzWaveStrength = 0.08,
                HorzWaveVertSize = 1.0,
                HorzWaveScreenSize = 1.0,
                HorzNoiseLinesContrib = 1.0,
                HorzNoiseLinesDensityY = 0.01,
                HorzNoiseLinesDensityXY = 0.5,
                HorzDistortionContrib = 1.0,
                HorzDistortionStrength = 0.002
            };
            return noise;
        }

        private static GimbalSettings GetDefaultGimbalSettings()
        {
            var gimbal = new GimbalSettings
            {
                Stabilization = 0, // Range 0-1 (No gimbal to full stabilization)
                Rotation = Rotation.NanRotation()
            };
            return gimbal;
        }

        private static OriginGeopointSettings GetDefaultOriginGeopointSettings()
        {
            var origin = new OriginGeopointSettings
            {
                Latitude = 47.641468,
                Longitude = -122.140165,
                Altitude = 122.0
            };
            return origin;
        }

        private static TimeOfDaySettings GetDefaultTimeOfDaySettings()
        {
            var timeofday = new TimeOfDaySettings
            {
                Enabled = false,
                StartDateTime = "",
                CelestialClockSpeed = 1.0,
                StartDateTimeDst = false,
                UpdateIntervalSecs = 60.0,
                MoveSun = false
            };
            return timeofday;
        }

        private static List<SubWindowsSettings> GetDefaultSubWindowsSettings()
        {
            var subwindowslist = new List<SubWindowsSettings>
            {
                new SubWindowsSettings {
                    WindowID = 0,
                    CameraName = "0",
                    ImageType = 3,
                    Visible = false
                },
                new SubWindowsSettings {
                    WindowID = 1,
                    CameraName = "0",
                    ImageType = 5,
                    Visible = false
                },
                new SubWindowsSettings {
                    WindowID = 2,
                    CameraName = "0",
                    ImageType = 0,
                    Visible = false
                }
            };
            return subwindowslist;
        }

        private static SegmentationSettingsSettings GetDefaultSegmentationSettingsSettings()
        {
            var segmentation = new SegmentationSettingsSettings
            {
                InitMethod = "",
                MeshNamingMethod = "",
                OverrideExisting = false
            };
            return segmentation;
        }

        private static PawnPathsSettings GetDefaultPawnPathsSettings()
        {
            var pawnpaths = new PawnPathsSettings
            {
                BareboneCar = new PawnPath
                {
                    PawnBP = "Class'/AirSim/VehicleAdv/Vehicle/VehicleAdvPawn.VehicleAdvPawn_C'", // Or Unity equivalent?
                    SlipperyMat = "/AirSim/VehicleAdv/PhysicsMaterials/Slippery.Slippery",
                    NonSlipperyMat = "/AirSim/VehicleAdv/PhysicsMaterials/NonSlippery.NonSlippery"
                },
                DefaultCar = new PawnPath
                {
                    PawnBP = "Class'/AirSim/VehicleAdv/SUV/SuvCarPawn.SuvCarPawn_C'", // Or Unity equivalent?
                    SlipperyMat = "/AirSim/VehicleAdv/PhysicsMaterials/Slippery.Slippery",
                    NonSlipperyMat = "/AirSim/VehicleAdv/PhysicsMaterials/NonSlippery.NonSlippery"
                },
                DefaultQuadrotor = new PawnPath
                {
                    PawnBP = "Class'/AirSim/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C'",  // Or Unity equivalent?
                    SlipperyMat = "/AirSim/VehicleAdv/PhysicsMaterials/Slippery.Slippery",
                    NonSlipperyMat = "/AirSim/VehicleAdv/PhysicsMaterials/NonSlippery.NonSlippery"
                },
                DefaultComputerVision = new PawnPath
                {
                    PawnBP = "Class'/AirSim/Blueprints/BP_ComputerVisionPawn.BP_ComputerVisionPawn_C'", // Or Unity equivalent
                    SlipperyMat = "/AirSim/VehicleAdv/PhysicsMaterials/Slippery.Slippery",
                    NonSlipperyMat = "/AirSim/VehicleAdv/PhysicsMaterials/NonSlippery.NonSlippery"
                }
            };
            return pawnpaths;
        }

        private static RCSettings GetDefaultRCSettings()
        {
            var rc = new RCSettings
            {
                RemoteControlID = 0,
                AllowAPIWhenDisconnected = false
            };
            return rc;
        }

        private static SimpleFlightSettings GetDefaultSimpleFlightSettings()
        {
            var simpleflight = new SimpleFlightSettings
            {
                VehicleName = "SimpleFlight",
                VehicleType = "SimpleFlight",
                DefaultVehicleState = "Armed",
                AutoCreate = true,
                EnableTrace = false,
                PawnPath = "",
                EnableCollisionPassthrough = false, // We can correct the spelling of the var
                EnableCollisions = true,
                IsFpvVehicle = false,
                AllowAPIAlways = true,
                RC = GetDefaultRCSettings(),
                Cameras = GetDefaultCamerasSettings(),
                Position = Position.NanPosition(),
                Rotation = Rotation.NanRotation(),
                Sensors = GetDefaultDefaultSensorsSettingsDrone()
            };
            return simpleflight;
        }

        private static PhysXCarSettings GetDefaultPhysXCarSettings()
        {
            var physXCar = new PhysXCarSettings
            {
                VehicleName = "PhysXCar",
                VehicleType = "PhysXCar",
                DefaultVehicleState = "",
                AutoCreate = true,
                EnableTrace = false,
                PawnPath = "",
                EnableCollisionPassthrogh = false, // We can correct the spelling of the var
                EnableCollisions = true,
                RC = GetDefaultRCSettings(),
                Cameras = GetDefaultCamerasSettings(),
                Position = Position.NanPosition(),
                Rotation = Rotation.NanRotation(),
                Sensors = GetDefaultDefaultSensorsSettingsCar()
            };
            physXCar.RC.RemoteControlID = -1;
            return physXCar;
        }

        private static ComputerVisionSettings GetDefaultComputerVisionSettings()
        {
            var computerVision = new ComputerVisionSettings
            {
                VehicleName = "ComputerVision",
                VehicleType = "ComputerVision",
                DefaultVehicleState = "Armed",
                AutoCreate = true,
                EnableTrace = false,
                PawnPath = "",
                EnableCollisionPassthrogh = false, // We can correct the spelling of the var
                EnableCollisions = true,
                IsFpvVehicle = false,
                AllowAPIAlways = true,
                RC = GetDefaultRCSettings(),
                Cameras = GetDefaultCamerasSettings(),
                Position = Position.NanPosition(),
                Rotation = Rotation.NanRotation(),
            };
            return computerVision;
        }

        private static PX4MultirotorSettings GetDefaultPX4MultirotorSettings()
        {
            var px4 = new PX4MultirotorSettings
            {
                VehicleName = "PX4Multirotor",
                VehicleType = "PX4Multirotor",
                // PX4 Specific
                LogViewerHostIp = "127.0.0.1",
                LogViewerPort = 14388,
                //LogViewerSendPort = ??
                OffboardCompID = 1,
                OffboardSysID = 134,
                QgcHostIp = "127.0.0.1",
                QgcPort = 14550,
                SerialBaudRate = 115200,
                SerialPort = "*",
                SimCompID = 42,
                SimSysID = 142,
                SitlIp = "127.0.0.1",
                SitlPort = 14556,
                UdpIp = "127.0.0.1",
                UdpPort = 14560,
                UseSerial = true,
                VehicleCompID = 1,
                VehicleSysID = 135,
                Model = "Generic",
                LocalHostIp = "127.0.0.1",
                // End PX4 Specific
                DefaultVehicleState = "Armed",
                AutoCreate = true,
                EnableTrace = false,
                PawnPath = "",
                EnableCollisionPassthrogh = false, // We can correct the spelling of the var
                EnableCollisions = true,
                IsFpvVehicle = false,
                AllowAPIAlways = true,
                RC = GetDefaultRCSettings(),
                Cameras = GetDefaultCamerasSettings(),
                Position = Position.NanPosition(),
                Rotation = Rotation.NanRotation(),
                Sensors = GetDefaultDefaultSensorsSettingsDrone()
            };
            return px4;
        }

        private static DefaultSensorsSettings GetDefaultDefaultSensorsSettingsCar()
        {
            var defaultsensors = new DefaultSensorsSettings
            {
                GpsList = new List<GpsSettings>
                {
                    GetDefaultGpsSettings()
                },
                LidarList = new List<LidarSettings>
                {
                    GetDefaultLidarSettingsCar()
                }
            };
            return defaultsensors;
        }

        private static DefaultSensorsSettings GetDefaultDefaultSensorsSettingsDrone()
        {
            var defaultsensors = new DefaultSensorsSettings
            {
                BarometerList = new List<BarometerSettings>
                {
                    GetDefaultBarometerSettings()
                },
                ImuList = new List<ImuSettings>
                {
                    GetDefaultImuSettings()
                },
                GpsList = new List<GpsSettings>
                {
                    GetDefaultGpsSettings()
                },
                MagnetometerList = new List<MagnetometerSettings>
                {
                    GetDefaultMagnetometerSettings()
                },
                DistanceList = new List<DistanceSettings>
                {
                    GetDefaultDistanceSettings()
                },
                LidarList = new List<LidarSettings>
                {
                    GetDefaultLidarSettingsDrone()
                }
            };
            return defaultsensors;
        }

        private static BarometerSettings GetDefaultBarometerSettings()
        {
            var barometer = new BarometerSettings
            {
                SensorType = SensorType.Barometer,
                Enabled = false
            };
            return barometer;
        }

        private static ImuSettings GetDefaultImuSettings()
        {
            var Imu = new ImuSettings
            {
                SensorType = SensorType.Imu,
                Enabled = false
            };
            return Imu;
        }

        private static GpsSettings GetDefaultGpsSettings()
        {
            var gps = new GpsSettings
            {
                SensorType = SensorType.Gps,
                Enabled = false
            };
            return gps;
        }

        private static MagnetometerSettings GetDefaultMagnetometerSettings()
        {
            var magnetometer = new MagnetometerSettings
            {
                SensorType = SensorType.Magnetometer,
                Enabled = false
            };
            return magnetometer;
        }

        private static DistanceSettings GetDefaultDistanceSettings()
        {
            var distance = new DistanceSettings
            {
                SensorType = SensorType.Distance,
                Enabled = false
            };
            return distance;
        }

        private static LidarSettings GetDefaultLidarSettingsCar()
        {
            var lidar = new LidarSettings
            {
                SensorType = SensorType.Lidar,
                Enabled = false,
                NumberOfChannels = 16,
                Range = 100.0f,
                RotationsPerSecond = 10,
                PointsPerSecond = 100000,
                Position = Position.NanPosition(),
                Rotation = Rotation.NanRotation(),
                VerticalFOVUpper = 10.0f, // Default for Car
                VerticalFOVLower = -10.0f, // Default for Car
                HorizontalFOVStart = 0,
                HorizontalFOVEnd = 359,
                DrawDebugPoints = false,
                DataFrame = "SensorLocalFrame"
            };
            return lidar;
        }

        private static LidarSettings GetDefaultLidarSettingsDrone()
        {
            var lidar = new LidarSettings
            {
                SensorType = SensorType.Lidar,
                Enabled = false,
                NumberOfChannels = 16,
                Range = 100.0f,
                RotationsPerSecond = 10,
                PointsPerSecond = 100000,
                Position = Position.NanPosition(),
                Rotation = Rotation.NanRotation(),
                VerticalFOVUpper = -15.0f, // Default for Drone
                VerticalFOVLower = -45.0f, // Default for Drone
                HorizontalFOVStart = 0,
                HorizontalFOVEnd = 359,
                DrawDebugPoints = false,
                DataFrame = "SensorLocalFrame"
            };
            return lidar;
        }

        private static CameraDirectorSettings GetDefaultCameraDirectorSettings()
        {
            var director = new CameraDirectorSettings
            {
                Position = Position.NanPosition(),
                Rotation = Rotation.NanRotation(),
                FollowDistance = float.NaN
            };
            return director;
        }

        /// <summary>
        /// Merges any attributes specified by the user in the settings.json file with the pre-initialized default values.
        /// Utilizes the very handy SimpleJSON script by Markus Göbel (Bunny83) which provides ability to parse the json tree
        /// and check if specific attributes exist with the HasKey and GetValueOrDefault extensions.
        /// Note 1: Default values are not set here - Instead we keep the pre-initialized values that were set in
        /// AirSimSettings.Initialize() method unless the user specified something specific.
        /// Note 2: Most validation is done later once the SimMode is selected from the SimModeSelector UI scene.
        /// Note 3: This merge does allow adding multiple vehicles of the same VehicleType (with different VehicleNames)
        /// but multiple vehicles of the same type has not yet been fully implemented so only the first vehicle
        /// with AutoCreate = true will be simulated - (see AirSimHUDScript)
        /// </summary>
        private void MergeWithUserSettings()
        {
            string filename = GetAirSimSettingsFileName();
            string json_string = "";

            if (filename != string.Empty)
            {
                json_string = File.ReadAllText(filename);
            }
            else
            {
                Debug.LogError("Error: 'settings.json' file either not present or not configured properly.");
#if UNITY_EDITOR
                EditorUtility.DisplayDialog("Missing 'settings.json' file!!!",
                    "'settings.json' file either not present or not configured properly.",
                    "Exit");
#endif
                Application.Quit();
            }

            var userSettings = JSON.Parse(json_string);

            // Verify SettingsVersion (required attribute) is provided
            if (userSettings.HasKey("SettingsVersion"))
            {
                SettingsVersion = userSettings["SettingsVersion"];
            }
            else
            {
                Debug.LogError("Error: 'settings.json' file SettingsVersion missing!  SettingsVersion attribute requried.");
#if UNITY_EDITOR
                EditorUtility.DisplayDialog("'settings.json' file SettingsVersion Error!!!",
                    "'settings.json' file is missing or SettingsVersion", "Exit");
#endif
                Application.Quit();
            }

            // Verify SettingsVersion is greater than 1.2
            if (SettingsVersion != 1.2)
            {
                Debug.LogError("Error: 'settings.json' file SettingsVersion is less than ver 1.2!  SettingsVersion 1.2 or higher is required.");
#if UNITY_EDITOR
                EditorUtility.DisplayDialog("'Settings.json' file SettingsVersion Error!!!",
                    "'settings.json' file is less than ver 1.2!  SettingsVersion 1.2 or higher is required.",
                    "Exit");
#endif
                Application.Quit();
            }

            SimMode = userSettings.GetValueOrDefault("SimMode", SimMode);
            ClockType = userSettings.GetValueOrDefault("ClockType", ClockType);
            ClockSpeed = userSettings.GetValueOrDefault("ClockSpeed", ClockSpeed);
            LocalHostIp = userSettings.GetValueOrDefault("LocalHostIp", LocalHostIp);
            ApiServerAddress = userSettings.GetValueOrDefault("ApiServerAddress", ApiServerAddress);
            RecordUIVisible = userSettings.GetValueOrDefault("RecordUIVisible", RecordUIVisible);
            LogMessagesVisible = userSettings.GetValueOrDefault("LogMessagesVisible", LogMessagesVisible);
            ViewMode = userSettings.GetValueOrDefault("ViewMode", ViewMode);
            RpcEnabled = userSettings.GetValueOrDefault("RpcEnabled", RpcEnabled);
            EngineSound = userSettings.GetValueOrDefault("EngineSound", EngineSound);
            PhysicsEngineName = userSettings.GetValueOrDefault("PhysicsEngineName", PhysicsEngineName);
            SpeedUnitFactor = userSettings.GetValueOrDefault("SpeedUnitFactor", SpeedUnitFactor);
            SpeedUnitLabel = userSettings.GetValueOrDefault("SpeedUnitLabel", SpeedUnitLabel);

            if (userSettings.HasKey("Recording"))
            {
                var recordingNode = userSettings["Recording"];
                Recording.RecordOnMove = recordingNode.GetValueOrDefault("RecordOnMove", Recording.RecordOnMove);
                Recording.RecordInterval = recordingNode.GetValueOrDefault("RecordInterval", Recording.RecordInterval);
                if (recordingNode.HasKey("Cameras"))
                {
                    Recording.Cameras.Clear();
                    foreach (JSONNode camNode in recordingNode["Cameras"])
                    {
                        var camerasSettings = AirSimSettings.GetDefaultCamerasSettings();
                        camerasSettings.CameraName = camNode.GetValueOrDefault("CameraName", camerasSettings.CameraName);
                        camerasSettings.ImageType = camNode.GetValueOrDefault("ImageType", camerasSettings.ImageType);
                        camerasSettings.PixelAsFloat = camNode.GetValueOrDefault("PixelAsFloat", camerasSettings.PixelAsFloat);
                        camerasSettings.Compress = camNode.GetValueOrDefault("Compress", camerasSettings.Compress);
                        Recording.Cameras.Add(camerasSettings);
                    }
                }
            }
            if (userSettings.HasKey("CameraDefaults"))
            {
                var camDefaultsNode = userSettings["CameraDefaults"];
                if (camDefaultsNode.HasKey("CaptureSettings"))
                {
                    CameraDefaults.CaptureSettings.Clear();
                    foreach (JSONNode captureNode in camDefaultsNode["CaptureSettings"])
                    {
                        var captureSettings = AirSimSettings.GetDefaultCaptureSettingsSettings();
                        captureSettings.ImageType = captureNode.GetValueOrDefault("ImageType", captureSettings.ImageType);
                        captureSettings.Width = captureNode.GetValueOrDefault("Width", captureSettings.Width);
                        captureSettings.Height = captureNode.GetValueOrDefault("Height", captureSettings.Height);
                        captureSettings.FOV_Degrees = captureNode.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                        captureSettings.AutoExposureSpeed = captureNode.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                        captureSettings.AutoExposureBias = captureNode.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                        captureSettings.AutoExposureMaxBrightness = captureNode.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                        captureSettings.AutoExposureMinBrightness = captureNode.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                        captureSettings.MotionBlurAmount = captureNode.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                        captureSettings.TargetGamma = captureNode.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                        captureSettings.ProjectionMode = captureNode.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                        captureSettings.OrthoWidth = captureNode.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                        CameraDefaults.CaptureSettings.Add(captureSettings);
                    }
                }
                if (camDefaultsNode.HasKey("NoiseSettings"))
                {
                    CameraDefaults.NoiseSettings.Clear();
                    foreach (JSONNode node in camDefaultsNode["NoiseSettings"])
                    {
                        var noiseSettings = AirSimSettings.GetDefaultNoiseSettingsSettings();
                        noiseSettings.Enabled = node.GetValueOrDefault("Enabled", noiseSettings.Enabled);
                        noiseSettings.ImageType = node.GetValueOrDefault("ImageType", noiseSettings.ImageType);
                        noiseSettings.RandContrib = node.GetValueOrDefault("RandContrib", noiseSettings.RandContrib);
                        noiseSettings.RandSize = node.GetValueOrDefault("RandSize", noiseSettings.RandSize);
                        noiseSettings.RandDensity = node.GetValueOrDefault("RandDensity", noiseSettings.RandDensity);
                        noiseSettings.HorzDistortionContrib = node.GetValueOrDefault("HorzDistortionContrib", noiseSettings.HorzDistortionContrib);
                        noiseSettings.HorzWaveStrength = node.GetValueOrDefault("HorzWaveStrength", noiseSettings.HorzWaveStrength);
                        noiseSettings.HorzWaveVertSize = node.GetValueOrDefault("HorzWaveVertSize", noiseSettings.HorzWaveVertSize);
                        noiseSettings.HorzWaveScreenSize = node.GetValueOrDefault("HorzWaveScreenSize", noiseSettings.HorzWaveScreenSize);
                        noiseSettings.HorzNoiseLinesContrib = node.GetValueOrDefault("HorzNoiseLinesContrib", noiseSettings.HorzNoiseLinesContrib);
                        noiseSettings.HorzNoiseLinesDensityY = node.GetValueOrDefault("HorzNoiseLinesDensityY", noiseSettings.HorzNoiseLinesDensityY);
                        noiseSettings.HorzNoiseLinesDensityXY = node.GetValueOrDefault("HorzNoiseLinesDensityXY", noiseSettings.HorzNoiseLinesDensityXY);
                        noiseSettings.HorzDistortionContrib = node.GetValueOrDefault("HorzDistortionContrib", noiseSettings.HorzDistortionContrib);
                        noiseSettings.HorzDistortionStrength = node.GetValueOrDefault("HorzDistortionStrength", noiseSettings.HorzDistortionStrength);
                        CameraDefaults.NoiseSettings.Add(noiseSettings);
                    }
                }
                if (camDefaultsNode.HasKey("Gimbal"))
                {
                    var gimbalNode = camDefaultsNode["Gimbal"];
                    CameraDefaults.Gimbal.Stabilization = gimbalNode.GetValueOrDefault("Stabilization", CameraDefaults.Gimbal.Stabilization);
                    var gimbalRotation = Rotation.NanRotation();
                    gimbalRotation.Pitch = gimbalNode.GetValueOrDefault("Pitch", gimbalRotation.Pitch);
                    gimbalRotation.Roll = gimbalNode.GetValueOrDefault("Roll", gimbalRotation.Roll);
                    gimbalRotation.Yaw = gimbalNode.GetValueOrDefault("Yaw", gimbalRotation.Yaw);
                    CameraDefaults.Gimbal.Rotation = gimbalRotation;
                }
                var camPosition = Position.NanPosition();
                camPosition.X = camDefaultsNode.GetValueOrDefault("X", camPosition.X);
                camPosition.Y = camDefaultsNode.GetValueOrDefault("Y", camPosition.Y);
                camPosition.Z = camDefaultsNode.GetValueOrDefault("Z", camPosition.Z);
                CameraDefaults.Position = camPosition;
                var camRotation = Rotation.NanRotation();
                camRotation.Pitch = camDefaultsNode.GetValueOrDefault("Pitch", camRotation.Pitch);
                camRotation.Roll = camDefaultsNode.GetValueOrDefault("Roll", camRotation.Roll);
                camRotation.Yaw = camDefaultsNode.GetValueOrDefault("Yaw", camRotation.Yaw);
                CameraDefaults.Rotation = camRotation;
            }
            if (userSettings.HasKey("OriginGeopoint"))
            {
                var originGeopointNode = userSettings["OriginGeopoint"];
                OriginGeopoint.Latitude = originGeopointNode.GetValueOrDefault("Latitude", OriginGeopoint.Latitude);
                OriginGeopoint.Longitude = originGeopointNode.GetValueOrDefault("Longitude", OriginGeopoint.Longitude);
                OriginGeopoint.Altitude = originGeopointNode.GetValueOrDefault("Altitude", OriginGeopoint.Altitude);
            }
            if (userSettings.HasKey("TimeOfDay"))
            {
                var timeOfDayNode = userSettings["TimeOfDay"];
                TimeOfDay.Enabled = timeOfDayNode.GetValueOrDefault("Enabled", TimeOfDay.Enabled);
                TimeOfDay.StartDateTime = timeOfDayNode.GetValueOrDefault("StartDateTime", TimeOfDay.StartDateTime);
                TimeOfDay.CelestialClockSpeed = timeOfDayNode.GetValueOrDefault("CelestialClockSpeed", TimeOfDay.CelestialClockSpeed);
                TimeOfDay.StartDateTimeDst = timeOfDayNode.GetValueOrDefault("StartDateTimeDst", TimeOfDay.StartDateTimeDst);
                TimeOfDay.UpdateIntervalSecs = timeOfDayNode.GetValueOrDefault("UpdateIntervalSecs", TimeOfDay.UpdateIntervalSecs);
                TimeOfDay.MoveSun = timeOfDayNode.GetValueOrDefault("MoveSun", TimeOfDay.MoveSun);
            }
            if (userSettings.HasKey("SubWindows"))
            {
                var subWindowsNode = userSettings["SubWindows"];
                var subWindow0 = SubWindows[0];
                var subWindow1 = SubWindows[1];
                var subWindow2 = SubWindows[2];
                SubWindows.Clear();
                foreach (JSONNode subWindowNode in subWindowsNode)
                {
                    if (subWindowNode.HasKey("WindowID"))
                    {
                        var UserSubWindow = new AirSimSettings.SubWindowsSettings()
                        {
                            WindowID = subWindowNode["WindowID"],
                            CameraName = subWindowNode.GetValueOrDefault("CameraName", "Not Specified"),
                            ImageType = subWindowNode.GetValueOrDefault("ImageType", -1),
                            Visible = subWindowNode.GetValueOrDefault("Visible", false)
                        };
                        switch (UserSubWindow.WindowID)
                        {
                            case 0:
                                subWindow0.CameraName = UserSubWindow.CameraName != "Not Specified" ? UserSubWindow.CameraName : "0";
                                subWindow0.ImageType = UserSubWindow.ImageType != -1 ? UserSubWindow.ImageType : 3;
                                subWindow0.Visible = UserSubWindow.Visible;
                                break;
                            case 1:
                                subWindow1.CameraName = UserSubWindow.CameraName != "Not Specified" ? UserSubWindow.CameraName : "0";
                                subWindow1.ImageType = UserSubWindow.ImageType != -1 ? UserSubWindow.ImageType : 5;
                                subWindow1.Visible = UserSubWindow.Visible;
                                break;
                            case 2:
                                subWindow2.CameraName = UserSubWindow.CameraName != "Not Specified" ? UserSubWindow.CameraName : "0";
                                subWindow2.ImageType = UserSubWindow.ImageType != -1 ? UserSubWindow.ImageType : 0;
                                subWindow2.Visible = UserSubWindow.Visible;
                                break;
                            default:
                                Debug.Log("Notice: 'settings.json' file SubWindow.WindowID out of range.  Use 0, 1, or 2 only.");
                                break;
                        }
                    }
                }
                SubWindows.Add(subWindow0);
                SubWindows.Add(subWindow1);
                SubWindows.Add(subWindow2);
            }
            if (userSettings.HasKey("SegmentationSettings"))
            {
                var segmentationSettingsNode = userSettings["SegmentationSettings"];
                SegmentationSettings.InitMethod = segmentationSettingsNode.GetValueOrDefault("InitMethod", SegmentationSettings.InitMethod);
                SegmentationSettings.MeshNamingMethod = segmentationSettingsNode.GetValueOrDefault("MeshNamingMethod", SegmentationSettings.MeshNamingMethod);
                SegmentationSettings.OverrideExisting = segmentationSettingsNode.GetValueOrDefault("OverrideExisting", SegmentationSettings.OverrideExisting);
            }
            if (userSettings.HasKey("PawnPaths"))
            {
                var pawnPathsNode = userSettings["PawnPaths"];
                if (pawnPathsNode.HasKey("BareboneCar"))
                {
                    var bareboneCarNode = pawnPathsNode["BareboneCar"];
                    PawnPaths.BareboneCar.PawnBP =
                        bareboneCarNode.GetValueOrDefault("PawnBP", PawnPaths.BareboneCar.PawnBP);
                    PawnPaths.BareboneCar.SlipperyMat =
                        bareboneCarNode.GetValueOrDefault("SlipperyMat", PawnPaths.BareboneCar.NonSlipperyMat);
                    PawnPaths.BareboneCar.NonSlipperyMat =
                        bareboneCarNode.GetValueOrDefault("NonSlipperyMat", PawnPaths.BareboneCar.NonSlipperyMat);
                }
                if (pawnPathsNode.HasKey("DefaultCar"))
                {
                    var defaultCarNode = pawnPathsNode["DefaultCar"];
                    PawnPaths.DefaultCar.PawnBP =
                        defaultCarNode.GetValueOrDefault("PawnBP", PawnPaths.DefaultCar.PawnBP);
                    PawnPaths.DefaultCar.SlipperyMat =
                        defaultCarNode.GetValueOrDefault("SlipperyMat", PawnPaths.DefaultCar.NonSlipperyMat);
                    PawnPaths.DefaultCar.NonSlipperyMat =
                        defaultCarNode.GetValueOrDefault("NonSlipperyMat", PawnPaths.DefaultCar.NonSlipperyMat);
                }
                if (pawnPathsNode.HasKey("DefaultQuadrotor"))
                {
                    var defaultQuadrotorNode = pawnPathsNode["DefaultQuadrotor"];
                    PawnPaths.DefaultQuadrotor.PawnBP =
                        defaultQuadrotorNode.GetValueOrDefault("PawnBP", PawnPaths.DefaultQuadrotor.PawnBP);
                    PawnPaths.DefaultQuadrotor.SlipperyMat =
                        defaultQuadrotorNode.GetValueOrDefault("SlipperyMat", PawnPaths.DefaultQuadrotor.NonSlipperyMat);
                    PawnPaths.DefaultQuadrotor.NonSlipperyMat =
                        defaultQuadrotorNode.GetValueOrDefault("NonSlipperyMat", PawnPaths.DefaultQuadrotor.NonSlipperyMat);
                }
                if (pawnPathsNode.HasKey("DefaultComputerVision"))
                {
                    var defaultComputerVisionNode = pawnPathsNode["DefaultComputerVision"];
                    PawnPaths.DefaultComputerVision.PawnBP =
                        defaultComputerVisionNode.GetValueOrDefault("PawnBP", PawnPaths.DefaultComputerVision.PawnBP);
                    PawnPaths.DefaultComputerVision.SlipperyMat =
                        defaultComputerVisionNode.GetValueOrDefault("SlipperyMat", PawnPaths.DefaultComputerVision.NonSlipperyMat);
                    PawnPaths.DefaultComputerVision.NonSlipperyMat =
                        defaultComputerVisionNode.GetValueOrDefault("NonSlipperyMat", PawnPaths.DefaultComputerVision.NonSlipperyMat);
                }
            }
            if (userSettings.HasKey("Vehicles"))
            {
                bool isSimpleFlightDefaultCleared = false;
                bool isPhysXCarDefaultCleared = false;
                var vehiclesNode = userSettings["Vehicles"];
                foreach (string vehicleName in vehiclesNode.Keys)
                {
                    if (vehiclesNode[vehicleName].HasKey("VehicleType"))
                    {
                        var vNode = vehiclesNode[vehicleName];
                        switch (vNode["VehicleType"].Value)  // Add new vehicle depending on the Vehicle Type
                        {
                            case "SimpleFlight":
                                if (!isSimpleFlightDefaultCleared)
                                {
                                    Vehicles.Vehicles_SimpleFlight.Clear();
                                    isSimpleFlightDefaultCleared = true;
                                }
                                var simpleFlightSetting = AirSimSettings.GetDefaultSimpleFlightSettings();                           
                                if (!Vehicles.Vehicles_SimpleFlight.Any(x => x.VehicleName == vehicleName))
                                {
                                    simpleFlightSetting.VehicleName = vehicleName;
                                }
                                else // Remove duplicates - this vehicle replaces the previous one if it didn't have a unique name
                                {   // Note: This shouldn't really happen as json editors will identify duplicate attributes.
                                    Vehicles.Vehicles_SimpleFlight.Remove(Vehicles.Vehicles_SimpleFlight.Find(x => x.VehicleName == vehicleName));
                                    simpleFlightSetting.VehicleName = vehicleName; // Must be unique
                                }
                                simpleFlightSetting.DefaultVehicleState = vNode.GetValueOrDefault("DefaultVehicleState", simpleFlightSetting.DefaultVehicleState);
                                simpleFlightSetting.AutoCreate = vNode.GetValueOrDefault("AutoCreate", simpleFlightSetting.AutoCreate);
                                simpleFlightSetting.EnableTrace = vNode.GetValueOrDefault("EnableTrace", simpleFlightSetting.EnableTrace);
                                simpleFlightSetting.PawnPath = vNode.GetValueOrDefault("PawnPath", simpleFlightSetting.PawnPath);
                                simpleFlightSetting.EnableCollisionPassthrough = vNode.GetValueOrDefault("EnableCollisionPassthrogh", simpleFlightSetting.EnableCollisionPassthrough);
                                simpleFlightSetting.EnableCollisions = vNode.GetValueOrDefault("EnableCollisions", simpleFlightSetting.EnableCollisions);
                                simpleFlightSetting.IsFpvVehicle = vNode.GetValueOrDefault("IsFpvVehicle", simpleFlightSetting.IsFpvVehicle);
                                simpleFlightSetting.AllowAPIAlways = vNode.GetValueOrDefault("AllowAPIAlways", simpleFlightSetting.AllowAPIAlways);
                                var sfPostion = Position.NanPosition();
                                sfPostion.X = vNode.GetValueOrDefault("X", sfPostion.X);
                                sfPostion.Y = vNode.GetValueOrDefault("Y", sfPostion.Y);
                                sfPostion.Z = vNode.GetValueOrDefault("Z", sfPostion.Z);
                                simpleFlightSetting.Position = sfPostion;
                                var sfRotation = Rotation.NanRotation();
                                sfRotation.Pitch = vNode.GetValueOrDefault("Pitch", sfRotation.Pitch);
                                sfRotation.Roll = vNode.GetValueOrDefault("Roll", sfRotation.Roll);
                                sfRotation.Yaw = vNode.GetValueOrDefault("Yaw", sfRotation.Yaw);
                                simpleFlightSetting.Rotation = sfRotation;
                                if (vNode.HasKey("RC"))
                                {
                                    var rcNode = vNode["RC"];
                                    simpleFlightSetting.RC.RemoteControlID = rcNode.GetValueOrDefault("RemoteControlID", simpleFlightSetting.RC.RemoteControlID);
                                    simpleFlightSetting.RC.AllowAPIWhenDisconnected = rcNode.GetValueOrDefault("AllowAPIWhenDisconnected", simpleFlightSetting.RC.AllowAPIWhenDisconnected);
                                }
                                if (vNode.HasKey("Cameras"))
                                {
                                    var camerasNode = vNode["Cameras"];
                                    simpleFlightSetting.Cameras.CameraName = camerasNode.GetValueOrDefault("CameraName", simpleFlightSetting.Cameras.CameraName);
                                    simpleFlightSetting.Cameras.ImageType = camerasNode.GetValueOrDefault("ImageType", simpleFlightSetting.Cameras.ImageType);
                                    simpleFlightSetting.Cameras.PixelAsFloat = camerasNode.GetValueOrDefault("PixelAsFloat", simpleFlightSetting.Cameras.PixelAsFloat);
                                    simpleFlightSetting.Cameras.Compress = camerasNode.GetValueOrDefault("Compress", simpleFlightSetting.Cameras.Compress);

                                    if (camerasNode.HasKey("CaputureSettings"))
                                    {
                                        var capSettingsNode = camerasNode["CaputureSettings"];
                                        simpleFlightSetting.Cameras.CaptureSettings.Clear();
                                        foreach (JSONNode capNode in capSettingsNode)
                                        {
                                            var captureSettings = AirSimSettings.GetDefaultCaptureSettingsSettings();
                                            captureSettings.ImageType = capNode.GetValueOrDefault("ImageType", captureSettings.ImageType);
                                            captureSettings.Width = capNode.GetValueOrDefault("Width", captureSettings.Width);
                                            captureSettings.Height = capNode.GetValueOrDefault("Height", captureSettings.Height);
                                            captureSettings.FOV_Degrees = capNode.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                                            captureSettings.AutoExposureSpeed = capNode.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                                            captureSettings.AutoExposureBias = capNode.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                                            captureSettings.AutoExposureMaxBrightness = capNode.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                                            captureSettings.AutoExposureMinBrightness = capNode.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                                            captureSettings.MotionBlurAmount = capNode.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                                            captureSettings.TargetGamma = capNode.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                                            captureSettings.ProjectionMode = capNode.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                                            captureSettings.OrthoWidth = capNode.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                                            simpleFlightSetting.Cameras.CaptureSettings.Add(captureSettings);
                                        }
                                    }
                                }
                                if (vNode.HasKey("Sensors"))
                                {
                                    bool hasClearedBarometerList = false;
                                    bool hasClearedImuList = false;
                                    bool hasClearedGpsList = false;
                                    bool hasClearedMagnetometerList = false;
                                    bool hasClearedDistanceList = false;
                                    bool hasClearedLidarList = false;

                                    var sensorsNode = vNode["Sensors"];
                                    foreach (string sensorName in sensorsNode.Keys)
                                    {
                                        var sensorNode = sensorsNode[sensorName];
                                        if (sensorNode.HasKey("SensorType"))
                                        {
                                            var typeNode = sensorNode["SensorType"];
                                            var sensorType = (SensorType)typeNode.AsInt;
                                            switch (sensorType)
                                            {
                                                case SensorType.Barometer:
                                                    if (!hasClearedBarometerList)
                                                    {
                                                        simpleFlightSetting.Sensors.BarometerList.Clear();
                                                        hasClearedBarometerList = true;
                                                    }
                                                    var barometerSetting = AirSimSettings.GetDefaultBarometerSettings();
                                                    barometerSetting.Enabled = typeNode.GetValueOrDefault("Enabled", barometerSetting.Enabled);
                                                    simpleFlightSetting.Sensors.BarometerList.Add(barometerSetting);
                                                    break;
                                                case SensorType.Imu:
                                                    if (!hasClearedImuList)
                                                    {
                                                        simpleFlightSetting.Sensors.ImuList.Clear();
                                                        hasClearedImuList = true;
                                                    }
                                                    var imuSetting = AirSimSettings.GetDefaultImuSettings();
                                                    imuSetting.Enabled = typeNode.GetValueOrDefault("Enabled", imuSetting.Enabled);
                                                    simpleFlightSetting.Sensors.ImuList.Add(imuSetting);
                                                    break;
                                                case SensorType.Gps:
                                                    if (!hasClearedGpsList)
                                                    {
                                                        simpleFlightSetting.Sensors.GpsList.Clear();
                                                        hasClearedGpsList = true;
                                                    }
                                                    var gpsSetting = AirSimSettings.GetDefaultGpsSettings();
                                                    gpsSetting.Enabled = typeNode.GetValueOrDefault("Enabled", gpsSetting.Enabled);
                                                    simpleFlightSetting.Sensors.GpsList.Add(gpsSetting);
                                                    break;
                                                case SensorType.Magnetometer:
                                                    if (!hasClearedMagnetometerList)
                                                    {
                                                        simpleFlightSetting.Sensors.MagnetometerList.Clear();
                                                        hasClearedMagnetometerList = true;
                                                    }
                                                    var magnetometerSetting = AirSimSettings.GetDefaultMagnetometerSettings();
                                                    magnetometerSetting.Enabled = typeNode.GetValueOrDefault("Enabled", magnetometerSetting.Enabled);
                                                    simpleFlightSetting.Sensors.MagnetometerList.Add(magnetometerSetting);
                                                    break;
                                                case SensorType.Distance:
                                                    if (!hasClearedDistanceList)
                                                    {
                                                        simpleFlightSetting.Sensors.DistanceList.Clear();
                                                        hasClearedDistanceList = true;
                                                    }
                                                    var distanceSetting = AirSimSettings.GetDefaultDistanceSettings();
                                                    distanceSetting.Enabled = typeNode.GetValueOrDefault("Enabled", distanceSetting.Enabled);
                                                    simpleFlightSetting.Sensors.DistanceList.Add(distanceSetting);
                                                    break;
                                                case SensorType.Lidar:
                                                    if (!hasClearedLidarList)
                                                    {
                                                        simpleFlightSetting.Sensors.LidarList.Clear();
                                                        hasClearedLidarList = true;
                                                    }
                                                    var lidarSetting = AirSimSettings.GetDefaultLidarSettingsDrone();
                                                    lidarSetting.Enabled = typeNode.GetValueOrDefault("Enabled", lidarSetting.Enabled);
                                                    lidarSetting.NumberOfChannels = typeNode.GetValueOrDefault("NumberOfChannels", lidarSetting.NumberOfChannels);
                                                    lidarSetting.Range = typeNode.GetValueOrDefault("Range", lidarSetting.Range);
                                                    lidarSetting.RotationsPerSecond = typeNode.GetValueOrDefault("RotationsPerSecond", lidarSetting.RotationsPerSecond);
                                                    lidarSetting.PointsPerSecond = typeNode.GetValueOrDefault("PointsPerSecond", lidarSetting.PointsPerSecond);
                                                    var lidarPosition = Position.NanPosition();
                                                    lidarPosition.X = typeNode.GetValueOrDefault("X", lidarPosition.X);
                                                    lidarPosition.Y = typeNode.GetValueOrDefault("Y", lidarPosition.Y);
                                                    lidarPosition.Z = typeNode.GetValueOrDefault("Z", lidarPosition.Z);
                                                    lidarSetting.Position = lidarPosition;
                                                    var lidarRotation = Rotation.NanRotation();
                                                    lidarRotation.Pitch = typeNode.GetValueOrDefault("Pitch", lidarRotation.Pitch);
                                                    lidarRotation.Roll = typeNode.GetValueOrDefault("Roll", lidarRotation.Roll);
                                                    lidarRotation.Yaw = typeNode.GetValueOrDefault("Yaw", lidarRotation.Yaw);
                                                    lidarSetting.Rotation = lidarRotation;
                                                    lidarSetting.VerticalFOVUpper = typeNode.GetValueOrDefault("VerticalFOVUpper", lidarSetting.VerticalFOVUpper);
                                                    lidarSetting.VerticalFOVLower = typeNode.GetValueOrDefault("VerticalFOVLower", lidarSetting.VerticalFOVLower);
                                                    lidarSetting.HorizontalFOVStart = typeNode.GetValueOrDefault("HorizontalFOVStart", lidarSetting.HorizontalFOVStart);
                                                    lidarSetting.HorizontalFOVEnd = typeNode.GetValueOrDefault("HorizontalFOVEnd", lidarSetting.HorizontalFOVEnd);
                                                    lidarSetting.DrawDebugPoints = typeNode.GetValueOrDefault("DrawDebugPoints", lidarSetting.DrawDebugPoints);
                                                    lidarSetting.DataFrame = typeNode.GetValueOrDefault("DataFrame", lidarSetting.DataFrame);
                                                    simpleFlightSetting.Sensors.LidarList.Add(lidarSetting);
                                                    break;
                                                default:
                                                    Debug.Log("Notice: 'settings.json' file did not recognize Vehicle '" +
                                                        vehicleName + "' Sensor '" + sensorName + "' SensorType value. (" + sensorType + ") Must be value 1 through 6.");
                                                    break;
                                            }
                                        }
                                    }
                                }
                                Vehicles.Vehicles_SimpleFlight.Add(simpleFlightSetting);
                                break;
                            case "PhysXCar":
                                if (!isPhysXCarDefaultCleared)
                                {
                                    Vehicles.Vehicles_PhysXCar.Clear();
                                    isPhysXCarDefaultCleared = true;
                                }
                                var physXCarSetting = AirSimSettings.GetDefaultPhysXCarSettings();
                                if (!Vehicles.Vehicles_PhysXCar.Any(x => x.VehicleName == vehicleName))
                                {
                                    physXCarSetting.VehicleName = vehicleName;
                                }
                                else // Remove duplicates - this vehicle replaces the previous one if it didn't have a unique name
                                {   // Note: This shouldn't really happen as json editors will identify duplicate attributes.
                                    Vehicles.Vehicles_PhysXCar.Remove(Vehicles.Vehicles_PhysXCar.Find(x => x.VehicleName == vehicleName));
                                    physXCarSetting.VehicleName = vehicleName; // Must be unique
                                }
                                physXCarSetting.DefaultVehicleState = vNode.GetValueOrDefault("DefaultVehicleState", physXCarSetting.DefaultVehicleState);
                                physXCarSetting.AutoCreate = vNode.GetValueOrDefault("AutoCreate", physXCarSetting.AutoCreate);
                                physXCarSetting.EnableTrace = vNode.GetValueOrDefault("EnableTrace", physXCarSetting.EnableTrace);
                                physXCarSetting.PawnPath = vNode.GetValueOrDefault("PawnPath", physXCarSetting.PawnPath);
                                physXCarSetting.EnableCollisionPassthrogh = vNode.GetValueOrDefault("EnableCollisionPassthrogh", physXCarSetting.EnableCollisionPassthrogh);
                                physXCarSetting.EnableCollisions = vNode.GetValueOrDefault("EnableCollisions", physXCarSetting.EnableCollisions);
                                var carPostion = Position.NanPosition();
                                carPostion.X = vNode.GetValueOrDefault("X", carPostion.X);
                                carPostion.Y = vNode.GetValueOrDefault("Y", carPostion.Y);
                                carPostion.Z = vNode.GetValueOrDefault("Z", carPostion.Z);
                                physXCarSetting.Position = carPostion;
                                var carRotation = Rotation.NanRotation();
                                carRotation.Pitch = vNode.GetValueOrDefault("Pitch", carRotation.Pitch);
                                carRotation.Roll = vNode.GetValueOrDefault("Roll", carRotation.Roll);
                                carRotation.Yaw = vNode.GetValueOrDefault("Yaw", carRotation.Yaw);
                                physXCarSetting.Rotation = carRotation;
                                if (vNode.HasKey("RC"))
                                {
                                    var rcNode = vNode["RC"];
                                    physXCarSetting.RC.RemoteControlID = rcNode.GetValueOrDefault("RemoteControlID", physXCarSetting.RC.RemoteControlID);
                                    //physXCarSetting.RC.AllowAPIWhenDisconnected = rcNode.GetValueOrDefault("AllowAPIWhenDisconnected", physXCarSetting.RC.AllowAPIWhenDisconnected);
                                }
                                if (vNode.HasKey("Cameras"))
                                {
                                    var camerasNode = vNode["Cameras"];
                                    physXCarSetting.Cameras.CameraName = camerasNode.GetValueOrDefault("CameraName", physXCarSetting.Cameras.CameraName);
                                    physXCarSetting.Cameras.ImageType = camerasNode.GetValueOrDefault("ImageType", physXCarSetting.Cameras.ImageType);
                                    physXCarSetting.Cameras.PixelAsFloat = camerasNode.GetValueOrDefault("PixelAsFloat", physXCarSetting.Cameras.PixelAsFloat);
                                    physXCarSetting.Cameras.Compress = camerasNode.GetValueOrDefault("Compress", physXCarSetting.Cameras.Compress);
                                    if (camerasNode.HasKey("CaputureSettings"))
                                    {
                                        var capSettingsNode = camerasNode["CaputureSettings"];
                                        physXCarSetting.Cameras.CaptureSettings.Clear();
                                        foreach (JSONNode capNode in capSettingsNode)
                                        {
                                            var captureSettings = AirSimSettings.GetDefaultCaptureSettingsSettings();
                                            captureSettings.ImageType = capNode.GetValueOrDefault("ImageType", captureSettings.ImageType);
                                            captureSettings.Width = capNode.GetValueOrDefault("Width", captureSettings.Width);
                                            captureSettings.Height = capNode.GetValueOrDefault("Height", captureSettings.Height);
                                            captureSettings.FOV_Degrees = capNode.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                                            captureSettings.AutoExposureSpeed = capNode.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                                            captureSettings.AutoExposureBias = capNode.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                                            captureSettings.AutoExposureMaxBrightness = capNode.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                                            captureSettings.AutoExposureMinBrightness = capNode.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                                            captureSettings.MotionBlurAmount = capNode.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                                            captureSettings.TargetGamma = capNode.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                                            captureSettings.ProjectionMode = capNode.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                                            captureSettings.OrthoWidth = capNode.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                                            physXCarSetting.Cameras.CaptureSettings.Add(captureSettings);
                                        }
                                    }
                                }
                                if (vNode.HasKey("Sensors"))
                                {
                                    bool hasClearedGpsList = false;
                                    bool hasClearedLidarList = false;

                                    var sensorsNode = vNode["Sensors"];
                                    foreach (string sensorName in sensorsNode.Keys)
                                    {
                                        var sensorNode = sensorsNode[sensorName];
                                        if (sensorNode.HasKey("SensorType"))
                                        {
                                            var typeNode = sensorNode["SensorType"];
                                            var sensorType = (SensorType)typeNode.AsInt;
                                            switch (sensorType)
                                            {
                                                case SensorType.Barometer:
                                                    Debug.Log("Notice: PhysXCar doesn't support Barometer sensor.");
                                                    break;
                                                case SensorType.Imu:
                                                    Debug.Log("Notice: PhysXCar doesn't support Imu sensor.");
                                                    break;
                                                case SensorType.Gps:
                                                    if (!hasClearedGpsList)
                                                    {
                                                        physXCarSetting.Sensors.GpsList.Clear();
                                                        hasClearedGpsList = true;
                                                    }
                                                    var gpsSetting = AirSimSettings.GetDefaultGpsSettings();
                                                    gpsSetting.Enabled = typeNode.GetValueOrDefault("Enabled", gpsSetting.Enabled);
                                                    physXCarSetting.Sensors.GpsList.Add(gpsSetting);
                                                    break;
                                                case SensorType.Magnetometer:
                                                    Debug.Log("Notice: PhysXCar doesn't support Magnetometer sensor.");
                                                    break;
                                                case SensorType.Distance:
                                                    Debug.Log("Notice: PhysXCar doesn't support Distance sensor.");  // Or does it?
                                                    break;
                                                case SensorType.Lidar:
                                                    if (!hasClearedLidarList)
                                                    {
                                                        physXCarSetting.Sensors.LidarList.Clear();
                                                        hasClearedLidarList = true;
                                                    }
                                                    var lidarSetting = AirSimSettings.GetDefaultLidarSettingsCar();
                                                    lidarSetting.Enabled = typeNode.GetValueOrDefault("Enabled", lidarSetting.Enabled);
                                                    lidarSetting.NumberOfChannels = typeNode.GetValueOrDefault("NumberOfChannels", lidarSetting.NumberOfChannels);
                                                    lidarSetting.Range = typeNode.GetValueOrDefault("Range", lidarSetting.Range);
                                                    lidarSetting.RotationsPerSecond = typeNode.GetValueOrDefault("RotationsPerSecond", lidarSetting.RotationsPerSecond);
                                                    lidarSetting.PointsPerSecond = typeNode.GetValueOrDefault("PointsPerSecond", lidarSetting.PointsPerSecond);
                                                    var lidarPosition = Position.NanPosition();
                                                    lidarPosition.X = typeNode.GetValueOrDefault("X", lidarPosition.X);
                                                    lidarPosition.Y = typeNode.GetValueOrDefault("Y", lidarPosition.Y);
                                                    lidarPosition.Z = typeNode.GetValueOrDefault("Z", lidarPosition.Z);
                                                    lidarSetting.Position = lidarPosition;
                                                    var lidarRotation = Rotation.NanRotation();
                                                    lidarRotation.Pitch = typeNode.GetValueOrDefault("Pitch", lidarRotation.Pitch);
                                                    lidarRotation.Roll = typeNode.GetValueOrDefault("Roll", lidarRotation.Roll);
                                                    lidarRotation.Yaw = typeNode.GetValueOrDefault("Yaw", lidarRotation.Yaw);
                                                    lidarSetting.Rotation = lidarRotation;
                                                    lidarSetting.VerticalFOVUpper = typeNode.GetValueOrDefault("VerticalFOVUpper", lidarSetting.VerticalFOVUpper);
                                                    lidarSetting.VerticalFOVLower = typeNode.GetValueOrDefault("VerticalFOVLower", lidarSetting.VerticalFOVLower);
                                                    lidarSetting.HorizontalFOVStart = typeNode.GetValueOrDefault("HorizontalFOVStart", lidarSetting.HorizontalFOVStart);
                                                    lidarSetting.HorizontalFOVEnd = typeNode.GetValueOrDefault("HorizontalFOVEnd", lidarSetting.HorizontalFOVEnd);
                                                    lidarSetting.DrawDebugPoints = typeNode.GetValueOrDefault("DrawDebugPoints", lidarSetting.DrawDebugPoints);
                                                    lidarSetting.DataFrame = typeNode.GetValueOrDefault("DataFrame", lidarSetting.DataFrame);
                                                    physXCarSetting.Sensors.LidarList.Add(lidarSetting);
                                                    break;
                                                default:
                                                    Debug.Log("Notice: 'settings.json' file did not recognize Vehicle '" +
                                                        vehicleName + "' Sensor '" + sensorName + "' SensorType value. (" + sensorType + ") Must be value 1 through 6.");
                                                    break;
                                            }
                                        }
                                    }
                                }
                                Vehicles.Vehicles_PhysXCar.Add(physXCarSetting);
                                break;
                            case "PX4Multirotor":
                                var pX4Setting = AirSimSettings.GetDefaultPX4MultirotorSettings();
                                if (!Vehicles.Vehicles_PX4Multirotor.Any(x => x.VehicleName == vehicleName))
                                {
                                    pX4Setting.VehicleName = vehicleName;
                                }
                                else // Remove duplicates - this vehicle replaces the previous one if it didn't have a unique name
                                {   // Note: This shouldn't really happen as json editors will identify duplicate attributes.
                                    Vehicles.Vehicles_PX4Multirotor.Remove(Vehicles.Vehicles_PX4Multirotor.Find(x => x.VehicleName == vehicleName));
                                    pX4Setting.VehicleName = vehicleName; // Must be unique
                                }
                                pX4Setting.LogViewerHostIp = vNode.GetValueOrDefault("LogViewerHostIp", pX4Setting.LogViewerHostIp);
                                pX4Setting.LogViewerPort = vNode.GetValueOrDefault("LogViewerPort", pX4Setting.LogViewerPort);
                                // PX4.LogViewerSendPort = vNode.GetValueOrDefault("LogViewerSendPort", PX4.LogViewerSendPort);
                                pX4Setting.OffboardCompID = vNode.GetValueOrDefault("OffboardCompID", pX4Setting.OffboardCompID);
                                pX4Setting.OffboardSysID = vNode.GetValueOrDefault("OffboardSysID", pX4Setting.OffboardSysID);
                                pX4Setting.QgcHostIp = vNode.GetValueOrDefault("QgcHostIp", pX4Setting.QgcHostIp);
                                pX4Setting.QgcPort = vNode.GetValueOrDefault("QgcPort", pX4Setting.QgcPort);
                                pX4Setting.SerialBaudRate = vNode.GetValueOrDefault("SerialBaudRate", pX4Setting.SerialBaudRate);
                                pX4Setting.SerialPort = vNode.GetValueOrDefault("SerialPort", pX4Setting.SerialPort);
                                pX4Setting.SimCompID = vNode.GetValueOrDefault("SimCompID", pX4Setting.SimCompID);
                                pX4Setting.SimSysID = vNode.GetValueOrDefault("SimSysID", pX4Setting.SimSysID);
                                pX4Setting.SitlIp = vNode.GetValueOrDefault("SitlIp", pX4Setting.SitlIp);
                                pX4Setting.SitlPort = vNode.GetValueOrDefault("SitlPort", pX4Setting.SitlPort);
                                pX4Setting.UdpIp = vNode.GetValueOrDefault("UdpIp", pX4Setting.UdpIp);
                                pX4Setting.UdpPort = vNode.GetValueOrDefault("UdpPort", pX4Setting.UdpPort);
                                pX4Setting.UseSerial = vNode.GetValueOrDefault("UseSerial", pX4Setting.UseSerial);
                                pX4Setting.VehicleCompID = vNode.GetValueOrDefault("VehicleCompID", pX4Setting.VehicleCompID);
                                pX4Setting.VehicleSysID = vNode.GetValueOrDefault("VehicleSysID", pX4Setting.VehicleSysID);
                                pX4Setting.Model = vNode.GetValueOrDefault("Model", pX4Setting.Model);
                                pX4Setting.LocalHostIp = vNode.GetValueOrDefault("LocalHostIp", pX4Setting.LocalHostIp);
                                pX4Setting.DefaultVehicleState = vNode.GetValueOrDefault("DefaultVehicleState", pX4Setting.DefaultVehicleState);
                                pX4Setting.AutoCreate = vNode.GetValueOrDefault("AutoCreate", pX4Setting.AutoCreate);
                                var px4Postion = Position.NanPosition();
                                px4Postion.X = vNode.GetValueOrDefault("X", px4Postion.X);
                                px4Postion.Y = vNode.GetValueOrDefault("Y", px4Postion.Y);
                                px4Postion.Z = vNode.GetValueOrDefault("Z", px4Postion.Z);
                                pX4Setting.Position = px4Postion;
                                var px4Rotation = Rotation.NanRotation();
                                px4Rotation.Pitch = vNode.GetValueOrDefault("Pitch", px4Rotation.Pitch);
                                px4Rotation.Roll = vNode.GetValueOrDefault("Roll", px4Rotation.Roll);
                                px4Rotation.Yaw = vNode.GetValueOrDefault("Yaw", px4Rotation.Yaw);
                                pX4Setting.Rotation = px4Rotation;
                                if (vNode.HasKey("RC"))
                                {
                                    var rcNode = vNode["RC"];
                                    pX4Setting.RC.RemoteControlID = rcNode.GetValueOrDefault("RemoteControlID", pX4Setting.RC.RemoteControlID);
                                    pX4Setting.RC.AllowAPIWhenDisconnected = rcNode.GetValueOrDefault("AllowAPIWhenDisconnected", pX4Setting.RC.AllowAPIWhenDisconnected);
                                }
                                if (vNode.HasKey("Cameras"))
                                {
                                    var camerasNode = vNode["Cameras"];
                                    pX4Setting.Cameras.CameraName = camerasNode.GetValueOrDefault("CameraName", pX4Setting.Cameras.CameraName);
                                    pX4Setting.Cameras.ImageType = camerasNode.GetValueOrDefault("ImageType", pX4Setting.Cameras.ImageType);
                                    pX4Setting.Cameras.PixelAsFloat = camerasNode.GetValueOrDefault("PixelAsFloat", pX4Setting.Cameras.PixelAsFloat);
                                    pX4Setting.Cameras.Compress = camerasNode.GetValueOrDefault("Compress", pX4Setting.Cameras.Compress);
                                    if (camerasNode.HasKey("CaputureSettings"))
                                    {
                                        var capSettingsNode = camerasNode["CaputureSettings"];
                                        pX4Setting.Cameras.CaptureSettings.Clear();
                                        foreach (JSONNode capNode in capSettingsNode)
                                        {
                                            var captureSettings = AirSimSettings.GetDefaultCaptureSettingsSettings();
                                            captureSettings.ImageType = capNode.GetValueOrDefault("ImageType", captureSettings.ImageType);
                                            captureSettings.Width = capNode.GetValueOrDefault("Width", captureSettings.Width);
                                            captureSettings.Height = capNode.GetValueOrDefault("Height", captureSettings.Height);
                                            captureSettings.FOV_Degrees = capNode.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                                            captureSettings.AutoExposureSpeed = capNode.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                                            captureSettings.AutoExposureBias = capNode.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                                            captureSettings.AutoExposureMaxBrightness = capNode.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                                            captureSettings.AutoExposureMinBrightness = capNode.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                                            captureSettings.MotionBlurAmount = capNode.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                                            captureSettings.TargetGamma = capNode.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                                            captureSettings.ProjectionMode = capNode.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                                            captureSettings.OrthoWidth = capNode.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                                            pX4Setting.Cameras.CaptureSettings.Add(captureSettings);
                                        }
                                    }
                                }
                                if (vNode.HasKey("Sensors"))
                                {
                                    bool hasClearedBarometerList = false;
                                    bool hasClearedImuList = false;
                                    bool hasClearedGpsList = false;
                                    bool hasClearedMagnetometerList = false;
                                    bool hasClearedDistanceList = false;
                                    bool hasClearedLidarList = false;

                                    var sensorsNode = vNode["Sensors"];
                                    foreach (string sensorName in sensorsNode.Keys)
                                    {
                                        var sensorNode = sensorsNode[sensorName];
                                        if (sensorNode.HasKey("SensorType"))
                                        {
                                            var typeNode = sensorNode["SensorType"];
                                            var sensorType = (SensorType)typeNode.AsInt;
                                            switch (sensorType)
                                            {
                                                case SensorType.Barometer:
                                                    if (!hasClearedBarometerList)
                                                    {
                                                        pX4Setting.Sensors.BarometerList.Clear();
                                                        hasClearedBarometerList = true;
                                                    }
                                                    var barometerSetting = AirSimSettings.GetDefaultBarometerSettings();
                                                    barometerSetting.Enabled = typeNode.GetValueOrDefault("Enabled", barometerSetting.Enabled);
                                                    pX4Setting.Sensors.BarometerList.Add(barometerSetting);
                                                    break;
                                                case SensorType.Imu:
                                                    if (!hasClearedImuList)
                                                    {
                                                        pX4Setting.Sensors.ImuList.Clear();
                                                        hasClearedImuList = true;
                                                    }
                                                    var imuSetting = AirSimSettings.GetDefaultImuSettings();
                                                    imuSetting.Enabled = typeNode.GetValueOrDefault("Enabled", imuSetting.Enabled);
                                                    pX4Setting.Sensors.ImuList.Add(imuSetting);
                                                    break;
                                                case SensorType.Gps:
                                                    if (!hasClearedGpsList)
                                                    {
                                                        pX4Setting.Sensors.GpsList.Clear();
                                                        hasClearedGpsList = true;
                                                    }
                                                    var gpsSetting = AirSimSettings.GetDefaultGpsSettings();
                                                    gpsSetting.Enabled = typeNode.GetValueOrDefault("Enabled", gpsSetting.Enabled);
                                                    pX4Setting.Sensors.GpsList.Add(gpsSetting);
                                                    break;
                                                case SensorType.Magnetometer:
                                                    if (!hasClearedMagnetometerList)
                                                    {
                                                        pX4Setting.Sensors.MagnetometerList.Clear();
                                                        hasClearedMagnetometerList = true;
                                                    }
                                                    var magnetometerSetting = AirSimSettings.GetDefaultMagnetometerSettings();
                                                    magnetometerSetting.Enabled = typeNode.GetValueOrDefault("Enabled", magnetometerSetting.Enabled);
                                                    pX4Setting.Sensors.MagnetometerList.Add(magnetometerSetting);
                                                    break;
                                                case SensorType.Distance:
                                                    if (!hasClearedDistanceList)
                                                    {
                                                        pX4Setting.Sensors.DistanceList.Clear();
                                                        hasClearedDistanceList = true;
                                                    }
                                                    var distanceSetting = AirSimSettings.GetDefaultDistanceSettings();
                                                    distanceSetting.Enabled = typeNode.GetValueOrDefault("Enabled", distanceSetting.Enabled);
                                                    pX4Setting.Sensors.DistanceList.Add(distanceSetting);
                                                    break;
                                                case SensorType.Lidar:
                                                    if (!hasClearedLidarList)
                                                    {
                                                        pX4Setting.Sensors.LidarList.Clear();
                                                        hasClearedLidarList = true;
                                                    }
                                                    var lidarSetting = AirSimSettings.GetDefaultLidarSettingsDrone();
                                                    lidarSetting.Enabled = typeNode.GetValueOrDefault("Enabled", lidarSetting.Enabled);
                                                    lidarSetting.NumberOfChannels = typeNode.GetValueOrDefault("NumberOfChannels", lidarSetting.NumberOfChannels);
                                                    lidarSetting.Range = typeNode.GetValueOrDefault("Range", lidarSetting.Range);
                                                    lidarSetting.RotationsPerSecond = typeNode.GetValueOrDefault("RotationsPerSecond", lidarSetting.RotationsPerSecond);
                                                    lidarSetting.PointsPerSecond = typeNode.GetValueOrDefault("PointsPerSecond", lidarSetting.PointsPerSecond);
                                                    var lidarPosition = Position.NanPosition();
                                                    lidarPosition.X = typeNode.GetValueOrDefault("X", lidarPosition.X);
                                                    lidarPosition.Y = typeNode.GetValueOrDefault("Y", lidarPosition.Y);
                                                    lidarPosition.Z = typeNode.GetValueOrDefault("Z", lidarPosition.Z);
                                                    lidarSetting.Position = lidarPosition;
                                                    var lidarRotation = Rotation.NanRotation();
                                                    lidarRotation.Pitch = typeNode.GetValueOrDefault("Pitch", lidarRotation.Pitch);
                                                    lidarRotation.Roll = typeNode.GetValueOrDefault("Roll", lidarRotation.Roll);
                                                    lidarRotation.Yaw = typeNode.GetValueOrDefault("Yaw", lidarRotation.Yaw);
                                                    lidarSetting.Rotation = lidarRotation;
                                                    lidarSetting.VerticalFOVUpper = typeNode.GetValueOrDefault("VerticalFOVUpper", lidarSetting.VerticalFOVUpper);
                                                    lidarSetting.VerticalFOVLower = typeNode.GetValueOrDefault("VerticalFOVLower", lidarSetting.VerticalFOVLower);
                                                    lidarSetting.HorizontalFOVStart = typeNode.GetValueOrDefault("HorizontalFOVStart", lidarSetting.HorizontalFOVStart);
                                                    lidarSetting.HorizontalFOVEnd = typeNode.GetValueOrDefault("HorizontalFOVEnd", lidarSetting.HorizontalFOVEnd);
                                                    lidarSetting.DrawDebugPoints = typeNode.GetValueOrDefault("DrawDebugPoints", lidarSetting.DrawDebugPoints);
                                                    lidarSetting.DataFrame = typeNode.GetValueOrDefault("DataFrame", lidarSetting.DataFrame);
                                                    pX4Setting.Sensors.LidarList.Add(lidarSetting);
                                                    break;
                                                default:
                                                    Debug.Log("Notice: 'settings.json' file did not recognize Vehicle '" +
                                                        vehicleName + "' Sensor '" + sensorName + "' SensorType value. (" + sensorType + ") Must be value 1 through 6.");
                                                    break;
                                            }
                                        }
                                    }
                                }
                                Vehicles.Vehicles_PX4Multirotor.Add(pX4Setting);
                                break;
                            case "ComputerVision":
                                var computerVisionSetting = AirSimSettings.GetDefaultComputerVisionSettings();
                                if (!Vehicles.Vehicles_ComputerVision.Any(x => x.VehicleName == vehicleName))
                                {
                                    computerVisionSetting.VehicleName = vehicleName;
                                }
                                else // Remove duplicates - this vehicle replaces the previous one if it didn't have a unique name
                                {   // Note: This shouldn't really happen as json editors will identify duplicate attributes.
                                    Vehicles.Vehicles_ComputerVision.Remove(Vehicles.Vehicles_ComputerVision.Find(x => x.VehicleName == vehicleName));
                                    computerVisionSetting.VehicleName = vehicleName; // Must be unique
                                }
                                computerVisionSetting.DefaultVehicleState = vNode.GetValueOrDefault("DefaultVehicleState", computerVisionSetting.DefaultVehicleState);
                                computerVisionSetting.AutoCreate = vNode.GetValueOrDefault("AutoCreate", computerVisionSetting.AutoCreate);
                                computerVisionSetting.EnableTrace = vNode.GetValueOrDefault("EnableTrace", computerVisionSetting.EnableTrace);
                                computerVisionSetting.PawnPath = vNode.GetValueOrDefault("PawnPath", computerVisionSetting.PawnPath);
                                computerVisionSetting.EnableCollisionPassthrogh = vNode.GetValueOrDefault("EnableCollisionPassthrogh", computerVisionSetting.EnableCollisionPassthrogh);
                                computerVisionSetting.EnableCollisions = vNode.GetValueOrDefault("EnableCollisions", computerVisionSetting.EnableCollisions);
                                computerVisionSetting.IsFpvVehicle = vNode.GetValueOrDefault("IsFpvVehicle", computerVisionSetting.IsFpvVehicle);
                                computerVisionSetting.AllowAPIAlways = vNode.GetValueOrDefault("AllowAPIAlways", computerVisionSetting.AllowAPIAlways);
                                var cvPostion = Position.NanPosition();
                                cvPostion.X = vNode.GetValueOrDefault("X", cvPostion.X);
                                cvPostion.Y = vNode.GetValueOrDefault("Y", cvPostion.Y);
                                cvPostion.Z = vNode.GetValueOrDefault("Z", cvPostion.Z);
                                computerVisionSetting.Position = cvPostion;
                                var cvRotation = Rotation.NanRotation();
                                cvRotation.Pitch = vNode.GetValueOrDefault("Pitch", cvRotation.Pitch);
                                cvRotation.Roll = vNode.GetValueOrDefault("Roll", cvRotation.Roll);
                                cvRotation.Yaw = vNode.GetValueOrDefault("Yaw", cvRotation.Yaw);
                                computerVisionSetting.Rotation = cvRotation;
                                if (vNode.HasKey("RC"))
                                {
                                    var rcNode = vNode["RC"];
                                    computerVisionSetting.RC.RemoteControlID = rcNode.GetValueOrDefault("RemoteControlID", computerVisionSetting.RC.RemoteControlID);
                                    computerVisionSetting.RC.AllowAPIWhenDisconnected = rcNode.GetValueOrDefault("AllowAPIWhenDisconnected", computerVisionSetting.RC.AllowAPIWhenDisconnected);
                                }
                                if (vNode.HasKey("Cameras"))
                                {
                                    var camerasNode = vNode["Cameras"];
                                    computerVisionSetting.Cameras.CameraName = camerasNode.GetValueOrDefault("CameraName", computerVisionSetting.Cameras.CameraName);
                                    computerVisionSetting.Cameras.ImageType = camerasNode.GetValueOrDefault("ImageType", computerVisionSetting.Cameras.ImageType);
                                    computerVisionSetting.Cameras.PixelAsFloat = camerasNode.GetValueOrDefault("PixelAsFloat", computerVisionSetting.Cameras.PixelAsFloat);
                                    computerVisionSetting.Cameras.Compress = camerasNode.GetValueOrDefault("Compress", computerVisionSetting.Cameras.Compress);
                                    if (camerasNode.HasKey("CaputureSettings"))
                                    {
                                        var capSettingsNode = camerasNode["CaputureSettings"];
                                        computerVisionSetting.Cameras.CaptureSettings.Clear();
                                        foreach (JSONNode capNode in capSettingsNode)
                                        {
                                            var captureSettings = AirSimSettings.GetDefaultCaptureSettingsSettings();
                                            captureSettings.ImageType = capNode.GetValueOrDefault("ImageType", captureSettings.ImageType);
                                            captureSettings.Width = capNode.GetValueOrDefault("Width", captureSettings.Width);
                                            captureSettings.Height = capNode.GetValueOrDefault("Height", captureSettings.Height);
                                            captureSettings.FOV_Degrees = capNode.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                                            captureSettings.AutoExposureSpeed = capNode.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                                            captureSettings.AutoExposureBias = capNode.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                                            captureSettings.AutoExposureMaxBrightness = capNode.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                                            captureSettings.AutoExposureMinBrightness = capNode.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                                            captureSettings.MotionBlurAmount = capNode.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                                            captureSettings.TargetGamma = capNode.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                                            captureSettings.ProjectionMode = capNode.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                                            captureSettings.OrthoWidth = capNode.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                                            computerVisionSetting.Cameras.CaptureSettings.Add(captureSettings);
                                        }
                                    }
                                }
                                if (vNode.HasKey("Sensors"))
                                {
                                    Debug.Log("Notice: ComputerVision does not implement any vehicle sensor other than cameras.");
                                }
                                Vehicles.Vehicles_ComputerVision.Add(computerVisionSetting);
                                break;
                            default:
                                Debug.Log("Notice: 'settings.json' file Vehicles '" + vehicleName + "' has VehicleType '" +
                                    vNode["VehicleType"].Value + "' that is not recognized.\nVehicleType attribute must be one of " +
                                    "(SimpleFlight, PhysXCar, PX4Multirotor, or ComputerVision)");
                                break;
                        }
                    }
                    else
                    {
                        Debug.Log("Notice: 'settings.json' file Vehicles '" + vehicleName + "' MUST have a VehicleType attribute specified.");
                    }
                }
            }
            if (userSettings.HasKey("CameraDirector"))
            {
                var cameraDirectorNode = userSettings["CameraDirector"];
                CameraDirector.FollowDistance = cameraDirectorNode.GetValueOrDefault("FollowDistance", CameraDirector.FollowDistance);
                var camPosition = Position.NanPosition();
                camPosition.X = cameraDirectorNode.GetValueOrDefault("X", camPosition.X);
                camPosition.Y = cameraDirectorNode.GetValueOrDefault("Y", camPosition.Y);
                camPosition.Z = cameraDirectorNode.GetValueOrDefault("Z", camPosition.Z);
                CameraDirector.Position = camPosition;
                var camRotation = Rotation.NanRotation();
                camRotation.Pitch = cameraDirectorNode.GetValueOrDefault("Pitch", camRotation.Pitch);
                camRotation.Roll = cameraDirectorNode.GetValueOrDefault("Roll", camRotation.Roll);
                camRotation.Yaw = cameraDirectorNode.GetValueOrDefault("Yaw", camRotation.Yaw);
                CameraDirector.Rotation = camRotation;
            }
            if (userSettings.HasKey("DefaultSensors"))
            {

                bool hasClearedBaromterList = false;
                bool hasClearedImuList = false;
                bool hasClearedGpsList = false;
                bool hasClearedMagnetometerList = false;
                bool hasClearedDistanceList = false;
                bool hasClearedLidarList = false;

                var defaultSensorsNode = userSettings["DefaultSensors"];
                foreach (string sensorName in defaultSensorsNode.Keys)
                {
                    var sensorNode = defaultSensorsNode[sensorName];
                    if (sensorNode.HasKey("SensorType"))
                    {
                        var typeNode = sensorNode["SensorType"];
                        var sensorType = (SensorType)typeNode.AsInt;

                        switch (sensorType)  // Add new sensor depending on the Sensor type
                        {
                            case SensorType.Barometer:
                                if (!hasClearedBaromterList)
                                {
                                    DefaultSensors.BarometerList.Clear();
                                    hasClearedBaromterList = true;
                                }
                                var barometerSetting = AirSimSettings.GetDefaultBarometerSettings();
                                barometerSetting.Enabled = typeNode.GetValueOrDefault("Enabled", barometerSetting.Enabled);
                                DefaultSensors.BarometerList.Add(barometerSetting);
                                break;
                            case SensorType.Imu:
                                if (!hasClearedImuList)
                                {
                                    DefaultSensors.ImuList.Clear();
                                    hasClearedImuList = true;
                                }
                                var imuSetting = AirSimSettings.GetDefaultImuSettings();
                                imuSetting.Enabled = typeNode.GetValueOrDefault("Enabled", imuSetting.Enabled);
                                DefaultSensors.ImuList.Add(imuSetting);
                                break;
                            case SensorType.Gps:
                                if (!hasClearedGpsList)
                                {
                                    DefaultSensors.GpsList.Clear();
                                    hasClearedGpsList = true;
                                }
                                var gpsSetting = AirSimSettings.GetDefaultGpsSettings();
                                gpsSetting.Enabled = typeNode.GetValueOrDefault("Enabled", gpsSetting.Enabled);
                                DefaultSensors.GpsList.Add(gpsSetting);
                                break;
                            case SensorType.Magnetometer:
                                if (!hasClearedMagnetometerList)
                                {
                                    DefaultSensors.MagnetometerList.Clear();
                                    hasClearedMagnetometerList = true;
                                }
                                var magnetometerSetting = AirSimSettings.GetDefaultMagnetometerSettings();
                                magnetometerSetting.Enabled = typeNode.GetValueOrDefault("Enabled", magnetometerSetting.Enabled);
                                DefaultSensors.MagnetometerList.Add(magnetometerSetting);
                                break;
                            case SensorType.Distance:
                                if (!hasClearedDistanceList)
                                {
                                    DefaultSensors.DistanceList.Clear();
                                    hasClearedDistanceList = true;
                                }
                                var distanceSetting = AirSimSettings.GetDefaultDistanceSettings();
                                distanceSetting.Enabled = sensorNode["SensorType"].GetValueOrDefault("Enabled", distanceSetting.Enabled);
                                DefaultSensors.DistanceList.Add(distanceSetting);
                                break;
                            case SensorType.Lidar:
                                if (!hasClearedLidarList)
                                {
                                    DefaultSensors.LidarList.Clear();
                                    hasClearedLidarList = true;
                                }
                                var lidarSetting = AirSimSettings.GetDefaultLidarSettingsDrone(); // Use default drone lidar setting for now - update in validate method
                                lidarSetting.Enabled = typeNode.GetValueOrDefault("Enabled", lidarSetting.Enabled);
                                lidarSetting.NumberOfChannels = typeNode.GetValueOrDefault("NumberOfChannels", lidarSetting.NumberOfChannels);
                                lidarSetting.Range = typeNode.GetValueOrDefault("Range", lidarSetting.Range);
                                lidarSetting.RotationsPerSecond = typeNode.GetValueOrDefault("RotationsPerSecond", lidarSetting.RotationsPerSecond);
                                lidarSetting.PointsPerSecond = typeNode.GetValueOrDefault("PointsPerSecond", lidarSetting.PointsPerSecond);
                                var lidarPosition = Position.NanPosition();
                                lidarPosition.X = typeNode.GetValueOrDefault("X", lidarPosition.X);
                                lidarPosition.Y = typeNode.GetValueOrDefault("Y", lidarPosition.Y);
                                lidarPosition.Z = typeNode.GetValueOrDefault("Z", lidarPosition.Z);
                                lidarSetting.Position = lidarPosition;
                                var lidarRotation = Rotation.NanRotation();
                                lidarRotation.Pitch = typeNode.GetValueOrDefault("Pitch", lidarRotation.Pitch);
                                lidarRotation.Roll = typeNode.GetValueOrDefault("Roll", lidarRotation.Roll);
                                lidarRotation.Yaw = typeNode.GetValueOrDefault("Yaw", lidarRotation.Yaw);
                                lidarSetting.Rotation = lidarRotation;
                                lidarSetting.VerticalFOVUpper = typeNode.GetValueOrDefault("VerticalFOVUpper", lidarSetting.VerticalFOVUpper);
                                lidarSetting.VerticalFOVLower = typeNode.GetValueOrDefault("VerticalFOVLower", lidarSetting.VerticalFOVLower);
                                lidarSetting.HorizontalFOVStart = typeNode.GetValueOrDefault("HorizontalFOVStart", lidarSetting.HorizontalFOVStart);
                                lidarSetting.HorizontalFOVEnd = typeNode.GetValueOrDefault("HorizontalFOVEnd", lidarSetting.HorizontalFOVEnd);
                                lidarSetting.DrawDebugPoints = typeNode.GetValueOrDefault("DrawDebugPoints", lidarSetting.DrawDebugPoints);
                                lidarSetting.DataFrame = typeNode.GetValueOrDefault("DataFrame", lidarSetting.DataFrame);
                                DefaultSensors.LidarList.Add(lidarSetting);
                                break;
                            default:
                                Debug.Log("Notice: 'settings.json' file did not recognize DefaultSensors SensorType.  SensorType must be 1 through 6");
                                break;
                        }
                    }
                }
            }
        }

        private string GetAirSimSettingsFileName()
        {
            string fileName = Application.dataPath + "\\..\\settings.json";

            if (File.Exists(fileName))
            {
                return fileName;
            }

            fileName = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), Path.Combine("AirSim", "settings.json"));

            string linuxFileName = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), Path.Combine("Documents/AirSim", "settings.json"));
            if (File.Exists(fileName))
            {
                Debug.Log("Returning fileName: " + fileName);
                return fileName;
            }
            else if (File.Exists(linuxFileName))
            {
                return linuxFileName;
            }

            if (CreateSettingsFileWithMinRequiredValues(fileName))
                return fileName;
            else if (CreateSettingsFileWithMinRequiredValues(linuxFileName))
                return linuxFileName;
            else
                return string.Empty;
        }

        private bool CreateSettingsFileWithMinRequiredValues(string fileName)
        {
            var result = false;
            try
            {
                if (fileName.Substring(0, 5) == "/home")
                    Directory.CreateDirectory(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Documents/AirSim"));
                else
                    Directory.CreateDirectory(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "AirSim"));

                string content = "{\n \"SimMode\" : \"\", \n \"SettingsVersion\" : 1.2, \n \"SeeDocsAt\" : \"https://github.com/Microsoft/AirSim/blob/master/docs/settings.md\"\n}";
                //settings file created at Documents\AirSim with name "setting.json".
                StreamWriter writer = new StreamWriter(File.Open(fileName, FileMode.OpenOrCreate, FileAccess.Write));
                writer.WriteLine(content);
                writer.Close();
                result = true;
                AirSimSettings.GetSettings().SettingsVersion = 1.2;
            }
            catch (Exception ex)
            {
                Debug.LogError("Unable to create settings.json file @ " + fileName + " Error :- " + ex.Message);
                result = false;
            }
            return result;
        }

        private bool CreateSettingsFileWithAllMergedSettings(string fileName)
        {
            var result = false;
            try
            {
                if (fileName.Substring(0, 5) == "/home")
                {
                    Directory.CreateDirectory(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Documents/AirSim"));
                }
                else
                {
                    Directory.CreateDirectory(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "AirSim"));
                }

                int i;

                StringBuilder sb = new StringBuilder(10000);
                sb.Append("{");
                var airSimUri = new Uri("https://github.com/Microsoft/AirSim/blob/master/docs/settings.md");
                sb.Append($"\n\t\"SeeDocsAt\" : \"{airSimUri.ToString()}\",");
                sb.Append($"\n\t\"SettingsVersion\" : {SettingsVersion.ToString()},");
                sb.Append($"\n\t\"SimMode\" : \"{SimMode}\",");
                sb.Append($"\n\t\"ClockType\" : \"{ClockType}\",");
                sb.Append($"\n\t\"ClockSpeed\" : {ClockSpeed.ToString()},");
                sb.Append($"\n\t\"LocalHostIp\" : \"{LocalHostIp}\",");
                sb.Append($"\n\t\"ApiServerAddress\" : \"{ApiServerAddress}\",");
                sb.Append($"\n\t\"RecordUIVisible\" : {RecordUIVisible.ToString().ToLower()},");
                sb.Append($"\n\t\"LogMessagesVisible\" : {LogMessagesVisible.ToString().ToLower()},");
                sb.Append($"\n\t\"ViewMode\" : \"{ViewMode}\",");
                sb.Append($"\n\t\"RpcEnabled\" : {RpcEnabled.ToString().ToLower()},");
                sb.Append($"\n\t\"EngineSound\" : {EngineSound.ToString().ToLower()},");
                sb.Append($"\n\t\"PhysicsEngineName\" : \"{PhysicsEngineName}\",");
                sb.Append($"\n\t\"SpeedUnitFactor\" : {SpeedUnitFactor.ToString()},");
                sb.Append($"\n\t\"SpeedUnitLabel\" : \"{SpeedUnitLabel}\",");

                sb.Append("\n\t\"Recording\" : {");
                sb.Append($"\n\t\t\"RecordOnMove\" : {Recording.RecordOnMove.ToString().ToLower()},");
                sb.Append($"\n\t\t\"RecordInterval\" : {Recording.RecordInterval.ToString()},");
                sb.Append("\n\t\t\"Cameras\" : [");
                foreach (var cam in Recording.Cameras)
                {
                    sb.Append("\n\t\t\t{");
                    sb.Append($"\n\t\t\t\"CameraName\" : \"{cam.CameraName}\",");
                    sb.Append($"\n\t\t\t\"ImageType\" : {cam.ImageType.ToString()},");
                    sb.Append($"\n\t\t\t\"PixelAsFloat\" : {cam.PixelAsFloat.ToString().ToLower()},");
                    sb.Append($"\n\t\t\t\"Compress\" : {cam.Compress.ToString().ToLower()}");
                    sb.Append("\n\t\t\t},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append("\n\t\t]");
                sb.Append("\n\t},");

                sb.Append("\n\t\"CameraDefaults\" : {");
                sb.Append("\n\t\t\"CaptureSettings\" : [");
                foreach (var capSet in CameraDefaults.CaptureSettings)
                {
                    sb.Append("\n\t\t\t{");
                    sb.Append($"\n\t\t\t\"ImageType\" : {capSet.ImageType.ToString()},");
                    sb.Append($"\n\t\t\t\"Width\" : {capSet.Width.ToString()},");
                    sb.Append($"\n\t\t\t\"Height\" : {capSet.Height.ToString()},");
                    sb.Append($"\n\t\t\t\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                    sb.Append($"\n\t\t\t\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                    sb.Append($"\n\t\t\t\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                    sb.Append($"\n\t\t\t\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                    sb.Append($"\n\t\t\t\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                    sb.Append($"\n\t\t\t\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                    sb.Append($"\n\t\t\t\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                    sb.Append($"\n\t\t\t\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                    sb.Append($"\n\t\t\t\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                    sb.Append("\n\t\t\t},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append("\n\t\t],");

                sb.Append("\n\t\t\"NoiseSettings\" : [");
                foreach (var noiseSet in CameraDefaults.NoiseSettings)
                {
                    sb.Append("\n\t\t\t{");
                    sb.Append($"\n\t\t\t\"Enabled\" : {noiseSet.Enabled.ToString().ToLower()},");
                    sb.Append($"\n\t\t\t\"ImageType\" : {noiseSet.ImageType.ToString()},");
                    sb.Append($"\n\t\t\t\"RandContrib\" : {noiseSet.RandContrib.ToString()},");
                    sb.Append($"\n\t\t\t\"RandSpeed\" : {noiseSet.RandSpeed.ToString()},");
                    sb.Append($"\n\t\t\t\"RandSize\" : {noiseSet.RandSize.ToString()},");
                    sb.Append($"\n\t\t\t\"RandDensity\" : {noiseSet.RandDensity.ToString()},");
                    sb.Append($"\n\t\t\t\"HorzWaveContrib\" : {noiseSet.HorzWaveContrib.ToString()},");
                    sb.Append($"\n\t\t\t\"HorzWaveStrength\" : {noiseSet.HorzWaveStrength.ToString()},");
                    sb.Append($"\n\t\t\t\"HorzWaveVertSize\" : {noiseSet.HorzWaveVertSize.ToString()},");
                    sb.Append($"\n\t\t\t\"HorzWaveScreenSize\" : {noiseSet.HorzWaveScreenSize.ToString()},");
                    sb.Append($"\n\t\t\t\"HorzNoiseLinesContrib\" : {noiseSet.HorzNoiseLinesContrib.ToString()},");
                    sb.Append($"\n\t\t\t\"HorzNoiseLinesDensityY\" : {noiseSet.HorzNoiseLinesDensityY.ToString()},");
                    sb.Append($"\n\t\t\t\"HorzNoiseLinesDensityXY\" : {noiseSet.HorzNoiseLinesDensityXY.ToString()},");
                    sb.Append($"\n\t\t\t\"HorzDistortionContrib\" : {noiseSet.HorzDistortionContrib.ToString()},");
                    sb.Append($"\n\t\t\t\"HorzDistortionStrength\" : {noiseSet.HorzDistortionStrength.ToString()}");
                    sb.Append("\n\t\t\t},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append("\n\t\t],");

                sb.Append("\n\t\t\"Gimbal\" : {");
                sb.Append($"\n\t\t\t\"Stabilization\" : {CameraDefaults.Gimbal.Stabilization.ToString()},");
                if (!float.IsNaN(CameraDefaults.Gimbal.Rotation.Pitch))
                {
                    Debug.Log("Pitch is " + CameraDefaults.Gimbal.Rotation.Pitch);
                    sb.Append($"\n\t\t\t\"Pitch\" : {CameraDefaults.Gimbal.Rotation.Pitch.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Gimbal.Rotation.Roll))
                {
                    sb.Append($"\n\t\t\t\"Roll\" : {CameraDefaults.Gimbal.Rotation.Roll.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Gimbal.Rotation.Yaw))
                {
                    sb.Append($"\n\t\t\t\"Yaw\" : {CameraDefaults.Gimbal.Rotation.Yaw.ToString()},");
                }
                sb.Remove(sb.Length - 1, 1);
                
                sb.Append("\n\t\t},");  // End Gimabl
                if (!float.IsNaN(CameraDefaults.Position.X))
                {
                    sb.Append($"\n\t\t\"X\" : {CameraDefaults.Position.X.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Position.Y))
                {
                    sb.Append($"\n\t\t\"Y\" : {CameraDefaults.Position.Y.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Position.Z))
                {
                    sb.Append($"\n\t\t\"Z\" : {CameraDefaults.Position.Z.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Rotation.Pitch))
                {
                    sb.Append($"\n\t\t\"Pitch\" : {CameraDefaults.Rotation.Pitch.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Rotation.Roll))
                {
                    sb.Append($"\n\t\t\"Roll\" : {CameraDefaults.Rotation.Roll.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Rotation.Yaw))
                {
                    sb.Append($"\n\t\t\"Yaw\" : {CameraDefaults.Rotation.Yaw.ToString()},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append("\n\t},"); // End CameraDefaults

                sb.Append("\n\t\"OriginGeopoint\" : {");
                sb.Append($"\n\t\t\"Latitude\" : {OriginGeopoint.Latitude.ToString()},");
                sb.Append($"\n\t\t\"Longitude\" : {OriginGeopoint.Longitude.ToString()},");
                sb.Append($"\n\t\t\"Altitude\" : {OriginGeopoint.Altitude.ToString()}");
                sb.Append("\n\t},"); // End OriginGeopoint

                sb.Append("\n\t\"TimeOfDay\" : {");
                sb.Append($"\n\t\t\"Enabled\" : {TimeOfDay.Enabled.ToString().ToLower()},");
                sb.Append($"\n\t\t\"StartDateTime\" : \"{TimeOfDay.StartDateTime}\",");
                sb.Append($"\n\t\t\"CelestialClockSpeed\" : {TimeOfDay.CelestialClockSpeed.ToString()},");
                sb.Append($"\n\t\t\"StartDateTimeDst\" : {TimeOfDay.StartDateTimeDst.ToString().ToLower()},");
                sb.Append($"\n\t\t\"UpdateIntervalSecs\" : {TimeOfDay.UpdateIntervalSecs.ToString()},");
                sb.Append($"\n\t\t\"MoveSun\" : {TimeOfDay.MoveSun.ToString().ToLower()}");
                sb.Append("\n\t},"); // End TimeOfDay

                sb.Append("\n\t\"SubWindows\" : [");
                foreach (var subWindow in SubWindows)
                {
                    sb.Append("\n\t\t\t{");
                    sb.Append($"\n\t\t\t\"WindowID\" : {subWindow.WindowID.ToString()},");
                    sb.Append($"\n\t\t\t\"CameraName\" : \"{subWindow.CameraName}\",");
                    sb.Append($"\n\t\t\t\"ImageType\" : {subWindow.ImageType.ToString()},");
                    sb.Append($"\n\t\t\t\"Visible\" : {subWindow.Visible.ToString().ToLower()}");
                    sb.Append("\n\t\t\t},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append("\n\t],"); // End SubWindows

                sb.Append("\n\t\"SegmentationSettings\" : {");
                sb.Append($"\n\t\t\"InitMethod\" : \"{SegmentationSettings.InitMethod}\",");
                sb.Append($"\n\t\t\"MeshNamingMethod\" : \"{SegmentationSettings.MeshNamingMethod}\",");
                sb.Append($"\n\t\t\"OverrideExisting\" : {SegmentationSettings.OverrideExisting.ToString().ToLower()}");
                sb.Append("\n\t},");

                sb.Append("\n\t\"PawnPaths\" : {");
                sb.Append("\n\t\t\"BareboneCar\" : {");
                sb.Append($"\n\t\t\t\"PawnBP\" : \"{PawnPaths.BareboneCar.PawnBP}\"");
                sb.Append("\n\t\t},");
                sb.Append("\n\t\t\"DefaultCar\" : {");
                sb.Append($"\n\t\t\t\"PawnBP\" : \"{PawnPaths.DefaultCar.PawnBP}\"");
                sb.Append("\n\t\t},");
                sb.Append("\n\t\t\"DefaultQuadrotor\" : {");
                sb.Append($"\n\t\t\t\"PawnBP\" : \"{PawnPaths.DefaultQuadrotor.PawnBP}\"");
                sb.Append("\n\t\t},");
                sb.Append("\n\t\t\"DefaultComputerVision\" : {");
                sb.Append($"\n\t\t\t\"PawnBP\" : \"{PawnPaths.DefaultComputerVision.PawnBP}\"");
                sb.Append("\n\t\t}");
                sb.Append("\n\t},"); // End PawnPaths

                /// VEHICLES - Maybe need to add name
                sb.Append("\n\t\"Vehicles\" : {");
                foreach (var vehicle in Vehicles.Vehicles_SimpleFlight)
                {
                    if (vehicle.VehicleType == "SimpleFlight")
                    {
                        sb.Append($"\n\t\t\"{vehicle.VehicleName}\" : {{");
                        sb.Append($"\n\t\t\t\"VehicleType\" : \"{vehicle.VehicleType}\",");
                        sb.Append($"\n\t\t\t\"DefaultVehicleState\" : \"{vehicle.DefaultVehicleState}\",");
                        sb.Append($"\n\t\t\t\"AutoCreate\" : {vehicle.AutoCreate.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"EnableTrace\" : {vehicle.EnableTrace.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"PawnPath\" : \"{vehicle.PawnPath}\",");
                        sb.Append($"\n\t\t\t\"EnableCollisionPassthrogh\" : {vehicle.EnableCollisionPassthrough.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"EnableCollisions\" : {vehicle.EnableCollisions.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"IsFpvVehicle\" : {vehicle.IsFpvVehicle.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"AllowAPIAlways\" : {vehicle.AllowAPIAlways.ToString().ToLower()},");
                        sb.Append("\n\t\t\t\"RC\" : {");
                        sb.Append($"\n\t\t\t\t\"RemoteControlID\" : {vehicle.RC.RemoteControlID.ToString()},");
                        sb.Append($"\n\t\t\t\t\"AllowAPIWhenDisconnected\" : {vehicle.RC.AllowAPIWhenDisconnected.ToString().ToLower()}");
                        sb.Append("\n\t\t\t},"); // End RC
                        sb.Append("\n\t\t\t\"Cameras\" : {");
                        sb.Append($"\n\t\t\t\t\"CameraName\" : \"{vehicle.Cameras.CameraName}\",");
                        sb.Append($"\n\t\t\t\t\"ImageType\" : {vehicle.Cameras.ImageType.ToString()},");
                        sb.Append($"\n\t\t\t\t\"PixelAsFloat\" : {vehicle.Cameras.PixelAsFloat.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\t\"Compress\" : {vehicle.Cameras.Compress.ToString().ToLower()},");
                        sb.Append("\n\t\t\t\t\"CaptureSettings\" : [");

                        foreach (var capSet in vehicle.Cameras.CaptureSettings)
                        {
                            sb.Append("\n\t\t\t\t\t{");
                            sb.Append($"\n\t\t\t\t\t\"ImageType\" : {capSet.ImageType.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"Width\" : {capSet.Width.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"Height\" : {capSet.Height.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                            sb.Append($"\n\t\t\t\t\t\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                            sb.Append("\n\t\t\t\t\t},");
                        }
                        sb.Remove(sb.Length - 1, 1);

                        sb.Append("\n\t\t\t\t]");
                        sb.Append("\n\t\t\t},"); // End Cameras
                        if (!float.IsNaN(vehicle.Position.X))
                        {
                            sb.Append($"\n\t\t\t\"X\" : {vehicle.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Y))
                        {
                            sb.Append($"\n\t\t\t\"Y\" : {vehicle.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Z))
                        {
                            sb.Append($"\n\t\t\t\"Z\" : {vehicle.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Pitch))
                        {
                            sb.Append($"\n\t\t\t\"Pitch\" : {vehicle.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Roll))
                        {
                            sb.Append($"\n\t\t\t\"Roll\" : {vehicle.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Yaw))
                        {
                            sb.Append($"\n\t\t\t\"Yaw\" : {vehicle.Rotation.Yaw.ToString()},");
                        }
                        sb.Append("\n\t\t\t\"Sensors\" : {");
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.BarometerList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Barometer)
                            {
                                sb.Append($"\n\t\t\t\t\"Barometer{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 1,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.ImuList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Imu)
                            {
                                sb.Append($"\n\t\t\t\t\"Imu{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 2,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.GpsList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Gps)
                            {
                                sb.Append($"\n\t\t\t\t\"Gps{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 3,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.MagnetometerList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Magnetometer)
                            {
                                sb.Append($"\n\t\t\t\t\"Magnetometer{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 4,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.DistanceList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Distance)
                            {
                                sb.Append($"\n\t\t\t\t\"Distance{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 5,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.LidarList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Lidar)
                            {
                                sb.Append($"\n\t\t\t\t\"Lidar{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 6,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()},");
                                sb.Append($"\n\t\t\t\t\t\"NumberOfChannels\" : {sensor.NumberOfChannels.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"Range\" : {sensor.Range.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"RotationsPerSecond\" : {sensor.RotationsPerSecond.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"PointsPerSecond\" : {sensor.PointsPerSecond.ToString()},");
                                if (!float.IsNaN(sensor.Position.X))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"X\" : {sensor.Position.X.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Y))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Y\" : {sensor.Position.Y.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Z))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Z\" : {sensor.Position.Z.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Pitch))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Pitch\" : {sensor.Rotation.Pitch.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Roll))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Roll\" : {sensor.Rotation.Roll.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Yaw))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Yaw\" : {sensor.Rotation.Yaw.ToString()},");
                                }
                                sb.Append($"\n\t\t\t\t\t\"VerticalFOVUpper\" : {sensor.VerticalFOVUpper.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"VerticalFOVLower\" : {sensor.VerticalFOVLower.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"HorizontalFOVStart\" : {sensor.HorizontalFOVStart.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"HorizontalFOVEnd\" : {sensor.HorizontalFOVEnd.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"DrawDebugPoints\" : {sensor.DrawDebugPoints.ToString().ToLower()},");
                                sb.Append($"\n\t\t\t\t\t\"DataFrame\" : \"{sensor.DataFrame}\"");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append("\n\t\t\t}"); // End This SimpleFlight Sensor
                        sb.Append("\n\t\t},"); // End This SimpleFlight
                    } // End if
                } // End foreach

                foreach (var vehicle in Vehicles.Vehicles_PhysXCar)
                {
                    if (vehicle.VehicleType == "PhysXCar")
                    {
                        sb.Append($"\n\t\t\"{vehicle.VehicleName}\" : {{");
                        sb.Append($"\n\t\t\t\"VehicleType\" : \"{vehicle.VehicleType}\",");
                        sb.Append($"\n\t\t\t\"DefaultVehicleState\" : \"{vehicle.DefaultVehicleState}\",");
                        sb.Append($"\n\t\t\t\"AutoCreate\" : {vehicle.AutoCreate.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"EnableTrace\" : {vehicle.EnableTrace.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"PawnPath\" : \"{vehicle.PawnPath}\",");
                        sb.Append($"\n\t\t\t\"EnableCollisionPassthrogh\" : {vehicle.EnableCollisionPassthrogh.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"EnableCollisions\" : {vehicle.EnableCollisions.ToString().ToLower()},");
                        sb.Append("\n\t\t\t\"RC\" : {");
                        sb.Append($"\n\t\t\t\t\"RemoteControlID\" : {vehicle.RC.RemoteControlID.ToString()}");
                        sb.Append("\n\t\t\t},"); // End RC
                        sb.Append("\n\t\t\t\"Cameras\" : {");
                        sb.Append($"\n\t\t\t\t\"CameraName\" : \"{vehicle.Cameras.CameraName}\",");
                        sb.Append($"\n\t\t\t\t\"ImageType\" : {vehicle.Cameras.ImageType.ToString()},");
                        sb.Append($"\n\t\t\t\t\"PixelAsFloat\" : {vehicle.Cameras.PixelAsFloat.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\t\"Compress\" : {vehicle.Cameras.Compress.ToString().ToLower()},");
                        sb.Append("\n\t\t\t\t\"CaptureSettings\" : [");
                        foreach (var capSet in vehicle.Cameras.CaptureSettings)
                        {
                            sb.Append("\n\t\t\t\t\t{");
                            sb.Append($"\n\t\t\t\t\t\"ImageType\" : {capSet.ImageType.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"Width\" : {capSet.Width.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"Height\" : {capSet.Height.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                            sb.Append($"\n\t\t\t\t\t\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                            sb.Append("\n\t\t\t\t\t},");
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append("\n\t\t\t\t]");
                        sb.Append("\n\t\t\t},"); // End Cameras
                        if (!float.IsNaN(vehicle.Position.X))
                        {
                            sb.Append($"\n\t\t\t\"X\" : {vehicle.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Y))
                        {
                            sb.Append($"\n\t\t\t\"Y\" : {vehicle.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Z))
                        {
                            sb.Append($"\n\t\t\t\"Z\" : {vehicle.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Pitch))
                        {
                            sb.Append($"\n\t\t\t\"Pitch\" : {vehicle.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Roll))
                        {
                            sb.Append($"\n\t\t\t\"Roll\" : {vehicle.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Yaw))
                        {
                            sb.Append($"\n\t\t\t\"Yaw\" : {vehicle.Rotation.Yaw.ToString()},");
                        }
                        sb.Append("\n\t\t\t\"Sensors\" : {");
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.GpsList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Gps)
                            {
                                sb.Append($"\n\t\t\t\t\"Gps{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 3,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.LidarList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Lidar)
                            {
                                sb.Append($"\n\t\t\t\t\"Lidar{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 6,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()},");
                                sb.Append($"\n\t\t\t\t\t\"NumberOfChannels\" : {sensor.NumberOfChannels.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"Range\" : {sensor.Range.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"RotationsPerSecond\" : {sensor.RotationsPerSecond.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"PointsPerSecond\" : {sensor.PointsPerSecond.ToString()},");
                                if (!float.IsNaN(sensor.Position.X))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"X\" : {sensor.Position.X.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Y))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Y\" : {sensor.Position.Y.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Z))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Z\" : {sensor.Position.Z.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Pitch))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Pitch\" : {sensor.Rotation.Pitch.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Roll))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Roll\" : {sensor.Rotation.Roll.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Yaw))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Yaw\" : {sensor.Rotation.Yaw.ToString()},");
                                }
                                sb.Append($"\n\t\t\t\t\t\"VerticalFOVUpper\" : {sensor.VerticalFOVUpper.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"VerticalFOVLower\" : {sensor.VerticalFOVLower.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"HorizontalFOVStart\" : {sensor.HorizontalFOVStart.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"HorizontalFOVEnd\" : {sensor.HorizontalFOVEnd.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"DrawDebugPoints\" : {sensor.DrawDebugPoints.ToString().ToLower()},");
                                sb.Append($"\n\t\t\t\t\t\"DataFrame\" : \"{sensor.DataFrame}\"");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append("\n\t\t\t}"); // End This PhysXCar Sensors
                        sb.Append("\n\t\t},"); // End This PhysXCar
                    } // End if
                }

                foreach (var vehicle in Vehicles.Vehicles_PX4Multirotor)
                {
                    if (vehicle.VehicleType == "PX4Multirotor")
                    {
                        sb.Append($"\n\t\t\"{vehicle.VehicleName}\" : {{");
                        sb.Append($"\n\t\t\t\"VehicleType\" : \"{vehicle.VehicleType}\",");
                        sb.Append($"\n\t\t\t\"LogViewerPort\" : {vehicle.LogViewerPort.ToString()},");
                        //sb.Append($"\n\t\t\t\"LogViewerSendPort\" : {vehicle.LogViewerSendPort.ToString()},");
                        sb.Append($"\n\t\t\t\"OffboardCompID\" : {vehicle.OffboardCompID.ToString()},");
                        sb.Append($"\n\t\t\t\"OffboardSysID\" : {vehicle.OffboardSysID.ToString()},");
                        sb.Append($"\n\t\t\t\"QgcHostIp\" : \"{vehicle.QgcHostIp}\",");
                        sb.Append($"\n\t\t\t\"QgcPort\" : {vehicle.QgcPort.ToString()},");
                        sb.Append($"\n\t\t\t\"SerialBaudRate\" : {vehicle.SerialBaudRate.ToString()},");
                        sb.Append($"\n\t\t\t\"SerialPort\" : \"{vehicle.SerialPort}\",");
                        sb.Append($"\n\t\t\t\"SimCompID\" : {vehicle.SimCompID.ToString()},");
                        sb.Append($"\n\t\t\t\"SimSysID\" : {vehicle.SimSysID.ToString()},");
                        sb.Append($"\n\t\t\t\"SitlIp\" : \"{vehicle.SitlIp}\",");
                        sb.Append($"\n\t\t\t\"SitlPort\" : {vehicle.SitlPort.ToString()},");
                        sb.Append($"\n\t\t\t\"UdpIp\" : \"{vehicle.UdpIp}\",");
                        sb.Append($"\n\t\t\t\"UdpPort\" : {vehicle.UdpPort.ToString()},");
                        sb.Append($"\n\t\t\t\"UseSerial\" : {vehicle.UseSerial.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"VehicleCompID\" : {vehicle.VehicleCompID.ToString()},");
                        sb.Append($"\n\t\t\t\"VehicleSysID\" : {vehicle.VehicleSysID.ToString()},");
                        sb.Append($"\n\t\t\t\"Model\" : \"{vehicle.Model}\",");
                        sb.Append($"\n\t\t\t\"LocalHostIp\" : \"{vehicle.LocalHostIp}\",");
                        sb.Append($"\n\t\t\t\"DefaultVehicleState\" : \"{vehicle.DefaultVehicleState}\",");
                        sb.Append($"\n\t\t\t\"AutoCreate\" : {vehicle.AutoCreate.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"EnableTrace\" : {vehicle.EnableTrace.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"PawnPath\" : \"{vehicle.PawnPath}\",");
                        sb.Append($"\n\t\t\t\"EnableCollisionPassthrogh\" : {vehicle.EnableCollisionPassthrogh.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"EnableCollisions\" : {vehicle.EnableCollisions.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"IsFpvVehicle\" : {vehicle.IsFpvVehicle.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"AllowAPIAlways\" : {vehicle.AllowAPIAlways.ToString().ToLower()},");
                        sb.Append("\n\t\t\t\"RC\" : {");
                        sb.Append($"\n\t\t\t\t\"RemoteControlID\" : {vehicle.RC.RemoteControlID.ToString()},");
                        sb.Append($"\n\t\t\t\t\"AllowAPIWhenDisconnected\" : {vehicle.RC.AllowAPIWhenDisconnected.ToString().ToLower()}");
                        sb.Append("\n\t\t\t},"); // End RC
                        sb.Append("\n\t\t\t\"Cameras\" : {");
                        sb.Append($"\n\t\t\t\t\"CameraName\" : \"{vehicle.Cameras.CameraName}\",");
                        sb.Append($"\n\t\t\t\t\"ImageType\" : {vehicle.Cameras.ImageType.ToString()},");
                        sb.Append($"\n\t\t\t\t\"PixelAsFloat\" : {vehicle.Cameras.PixelAsFloat.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\t\"Compress\" : {vehicle.Cameras.Compress.ToString().ToLower()},");
                        sb.Append("\n\t\t\t\t\"CaptureSettings\" : [");
                        foreach (var capSet in vehicle.Cameras.CaptureSettings)
                        {
                            sb.Append("\n\t\t\t\t\t{");
                            sb.Append($"\n\t\t\t\t\t\"ImageType\" : {capSet.ImageType.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"Width\" : {capSet.Width.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"Height\" : {capSet.Height.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                            sb.Append($"\n\t\t\t\t\t\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                            sb.Append("\n\t\t\t\t\t},");
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append("\n\t\t\t\t]");
                        sb.Append("\n\t\t\t},"); // End Cameras
                        if (!float.IsNaN(vehicle.Position.X))
                        {
                            sb.Append($"\n\t\t\t\"X\" : {vehicle.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Y))
                        {
                            sb.Append($"\n\t\t\t\"Y\" : {vehicle.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Z))
                        {
                            sb.Append($"\n\t\t\t\"Z\" : {vehicle.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Pitch))
                        {
                            sb.Append($"\n\t\t\t\"Pitch\" : {vehicle.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Roll))
                        {
                            sb.Append($"\n\t\t\t\"Roll\" : {vehicle.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Yaw))
                        {
                            sb.Append($"\n\t\t\t\"Yaw\" : {vehicle.Rotation.Yaw.ToString()},");
                        }
                        sb.Append("\n\t\t\t\"Sensors\" : {");
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.BarometerList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Barometer)
                            {
                                sb.Append($"\n\t\t\t\t\"Barometer{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 1,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.ImuList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Imu)
                            {
                                sb.Append($"\n\t\t\t\t\"Imu{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 2,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.GpsList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Gps)
                            {
                                sb.Append($"\n\t\t\t\t\"Gps{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 3,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.MagnetometerList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Magnetometer)
                            {
                                sb.Append($"\n\t\t\t\t\"Magnetometer{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 4,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.DistanceList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Distance)
                            {
                                sb.Append($"\n\t\t\t\t\"Distance{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 5,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.LidarList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Lidar)
                            {
                                sb.Append($"\n\t\t\t\t\"Lidar{i}\" : {{");
                                sb.Append($"\n\t\t\t\t\t\"SensorType\" : 6,");
                                sb.Append($"\n\t\t\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()},");
                                sb.Append($"\n\t\t\t\t\t\"NumberOfChannels\" : {sensor.NumberOfChannels.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"Range\" : {sensor.Range.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"RotationsPerSecond\" : {sensor.RotationsPerSecond.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"PointsPerSecond\" : {sensor.PointsPerSecond.ToString()},");
                                if (!float.IsNaN(sensor.Position.X))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"X\" : {sensor.Position.X.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Y))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Y\" : {sensor.Position.Y.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Z))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Z\" : {sensor.Position.Z.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Pitch))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Pitch\" : {sensor.Rotation.Pitch.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Roll))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Roll\" : {sensor.Rotation.Roll.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Yaw))
                                {
                                    sb.Append($"\n\t\t\t\t\t\"Yaw\" : {sensor.Rotation.Yaw.ToString()},");
                                }
                                sb.Append($"\n\t\t\t\t\t\"VerticalFOVUpper\" : {sensor.VerticalFOVUpper.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"VerticalFOVLower\" : {sensor.VerticalFOVLower.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"HorizontalFOVStart\" : {sensor.HorizontalFOVStart.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"HorizontalFOVEnd\" : {sensor.HorizontalFOVEnd.ToString()},");
                                sb.Append($"\n\t\t\t\t\t\"DrawDebugPoints\" : {sensor.DrawDebugPoints.ToString().ToLower()},");
                                sb.Append($"\n\t\t\t\t\t\"DataFrame\" : \"{sensor.DataFrame}\"");
                                sb.Append("\n\t\t\t\t},");
                            }
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append("\n\t\t\t}"); // End This PX4 Sensor
                        sb.Append("\n\t\t},"); // End This PX4
                    } // End if
                } // End foreach

                foreach (var vehicle in Vehicles.Vehicles_ComputerVision)
                {
                    if (vehicle.VehicleType == "ComputerVision")
                    {
                        sb.Append($"\n\t\t\"{vehicle.VehicleName}\" : {{");
                        sb.Append($"\n\t\t\t\"VehicleType\" : \"{vehicle.VehicleType}\",");
                        sb.Append($"\n\t\t\t\"DefaultVehicleState\" : \"{vehicle.DefaultVehicleState}\",");
                        sb.Append($"\n\t\t\t\"AutoCreate\" : {vehicle.AutoCreate.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"EnableTrace\" : {vehicle.EnableTrace.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"PawnPath\" : \"{vehicle.PawnPath}\",");
                        sb.Append($"\n\t\t\t\"EnableCollisionPassthrogh\" : {vehicle.EnableCollisionPassthrogh.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"EnableCollisions\" : {vehicle.EnableCollisions.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"IsFpvVehicle\" : {vehicle.IsFpvVehicle.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"AllowAPIAlways\" : {vehicle.AllowAPIAlways.ToString().ToLower()},");
                        sb.Append("\n\t\t\t\"RC\" : {");
                        sb.Append($"\n\t\t\t\t\"RemoteControlID\" : {vehicle.RC.RemoteControlID.ToString()},");
                        sb.Append($"\n\t\t\t\t\"AllowAPIWhenDisconnected\" : {vehicle.RC.AllowAPIWhenDisconnected.ToString().ToLower()}");
                        sb.Append("\n\t\t\t},"); // End RC
                        sb.Append("\n\t\t\t\"Cameras\" : {");
                        sb.Append($"\n\t\t\t\t\"CameraName\" : \"{vehicle.Cameras.CameraName}\",");
                        sb.Append($"\n\t\t\t\t\"ImageType\" : {vehicle.Cameras.ImageType.ToString()},");
                        sb.Append($"\n\t\t\t\t\"PixelAsFloat\" : {vehicle.Cameras.PixelAsFloat.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\t\"Compress\" : {vehicle.Cameras.Compress.ToString().ToLower()},");
                        sb.Append("\n\t\t\t\t\"CaptureSettings\" : [");
                        foreach (var capSet in vehicle.Cameras.CaptureSettings)
                        {
                            sb.Append("\n\t\t\t\t\t{");
                            sb.Append($"\n\t\t\t\t\t\"ImageType\" : {capSet.ImageType.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"Width\" : {capSet.Width.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"Height\" : {capSet.Height.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                            sb.Append($"\n\t\t\t\t\t\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                            sb.Append($"\n\t\t\t\t\t\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                            sb.Append("\n\t\t\t\t\t},");
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append("\n\t\t\t\t]");
                        sb.Append("\n\t\t\t},"); // End Cameras
                        if (!float.IsNaN(vehicle.Position.X))
                        {
                            sb.Append($"\n\t\t\t\"X\" : {vehicle.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Y))
                        {
                            sb.Append($"\n\t\t\t\"Y\" : {vehicle.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Z))
                        {
                            sb.Append($"\n\t\t\t\"Z\" : {vehicle.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Pitch))
                        {
                            sb.Append($"\n\t\t\t\"Pitch\" : {vehicle.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Roll))
                        {
                            sb.Append($"\n\t\t\t\"Roll\" : {vehicle.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Yaw))
                        {
                            sb.Append($"\n\t\t\t\"Yaw\" : {vehicle.Rotation.Yaw.ToString()},");
                        }
                        sb.Append("\n\t\t},"); // End This ComputerVision
                    } // End if
                } // End foreach
                sb.Remove(sb.Length - 1, 1);
                sb.Append("\n\t},"); // End Vehicles

                sb.Append("\n\t\"CameraDirector\" : {");
                if (!float.IsNaN(CameraDirector.Position.X))
                {
                    sb.Append($"\n\t\t\"X\" : {CameraDirector.Position.X.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Position.Y))
                {
                    sb.Append($"\n\t\t\"Y\" : {CameraDirector.Position.Y.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Position.Z))
                {
                    sb.Append($"\n\t\t\"Z\" : {CameraDirector.Position.Z.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Rotation.Pitch))
                {
                    sb.Append($"\n\t\t\"Pitch\" : {CameraDirector.Rotation.Pitch.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Rotation.Roll))
                {
                    sb.Append($"\n\t\t\"Roll\" : {CameraDirector.Rotation.Roll.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Rotation.Yaw))
                {
                    sb.Append($"\n\t\t\"Yaw\" : {CameraDirector.Rotation.Yaw.ToString()},");
                }
                sb.Append($"\n\t\t\"FollowDistance\" : {CameraDirector.FollowDistance}");
                sb.Append("\n\t},"); // End CameraDirector

                sb.Append("\n\t\"DefaultSensors\" : {");
                i = 0;
                foreach ( var sensor in DefaultSensors.BarometerList)
                {
                    i++;
                    if ( sensor.SensorType == SensorType.Barometer)
                    {
                        sb.Append($"\n\t\t\"Barometer{i}\" : {{");
                        sb.Append($"\n\t\t\t\"SensorType\" : 1,");
                        sb.Append($"\n\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append("\n\t\t},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.ImuList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Imu)
                    {
                        sb.Append($"\n\t\t\"Imu{i}\" : {{");
                        sb.Append($"\n\t\t\t\"SensorType\" : 2,");
                        sb.Append($"\n\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append("\n\t\t},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.GpsList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Gps)
                    {
                        sb.Append($"\n\t\t\"Gps{i}\" : {{");
                        sb.Append($"\n\t\t\t\"SensorType\" : 3,");
                        sb.Append($"\n\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append("\n\t\t},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.MagnetometerList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Magnetometer)
                    {
                        sb.Append($"\n\t\t\"Magnetometer{i}\" : {{");
                        sb.Append($"\n\t\t\t\"SensorType\" : 4,");
                        sb.Append($"\n\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append("\n\t\t},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.DistanceList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Distance)
                    {
                        sb.Append($"\n\t\t\"Distance{i}\" : {{");
                        sb.Append($"\n\t\t\t\"SensorType\" : 5,");
                        sb.Append($"\n\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append("\n\t\t},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.LidarList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Lidar)
                    {
                        sb.Append($"\n\t\t\"Lidar{i}\" : {{");
                        sb.Append($"\n\t\t\t\"SensorType\" : 6,");
                        sb.Append($"\n\t\t\t\"Enabled\" : {sensor.Enabled.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"NumberOfChannels\" : {sensor.NumberOfChannels.ToString()},");
                        sb.Append($"\n\t\t\t\"Range\" : {sensor.Range.ToString()},");
                        sb.Append($"\n\t\t\t\"RotationsPerSecond\" : {sensor.RotationsPerSecond.ToString()},");
                        sb.Append($"\n\t\t\t\"PointsPerSecond\" : {sensor.PointsPerSecond.ToString()},");

                        if (!float.IsNaN(sensor.Position.X))
                        {
                            sb.Append($"\n\t\t\t\"X\" : {sensor.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Position.Y))
                        {
                            sb.Append($"\n\t\t\t\"Y\" : {sensor.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Position.Z))
                        {
                            sb.Append($"\n\t\t\t\"Z\" : {sensor.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Rotation.Pitch))
                        {
                            sb.Append($"\n\t\t\t\"Pitch\" : {sensor.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Rotation.Roll))
                        {
                            sb.Append($"\n\t\t\t\"Roll\" : {sensor.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Rotation.Yaw))
                        {
                            sb.Append($"\n\t\t\t\"Yaw\" : {sensor.Rotation.Yaw.ToString()},");
                        }
                        sb.Append($"\n\t\t\t\"VerticalFOVUpper\" : {sensor.VerticalFOVUpper.ToString()},");
                        sb.Append($"\n\t\t\t\"VerticalFOVLower\" : {sensor.VerticalFOVLower.ToString()},");
                        sb.Append($"\n\t\t\t\"HorizontalFOVStart\" : {sensor.HorizontalFOVStart.ToString()},");
                        sb.Append($"\n\t\t\t\"HorizontalFOVEnd\" : {sensor.HorizontalFOVEnd.ToString()},");
                        sb.Append($"\n\t\t\t\"DrawDebugPoints\" : {sensor.DrawDebugPoints.ToString().ToLower()},");
                        sb.Append($"\n\t\t\t\"DataFrame\" : \"{sensor.DataFrame}\"");
                        sb.Append("\n\t\t},");
                    }
                }
                sb.Remove(sb.Length -1, 1);

                sb.Append("\n\t}");
                sb.Append("\n}");

                //Debug.Log("Writing to: " + fileName);
                if (File.Exists(fileName))
                {
                    File.Delete(fileName);
                }
                StreamWriter writer = new StreamWriter(File.Open(fileName, FileMode.OpenOrCreate, FileAccess.Write));
                writer.WriteLine(sb);
                writer.Close();
                result = true;
                AirSimSettings.GetSettings().SettingsVersion = 1.2;
            }
            catch (Exception ex)
            {
                Debug.LogError("Unable to create settings.json file @ " + fileName + " Error :- " + ex.Message);
                result = false;
            }
            return result;
        }
    }

    [Serializable]
    public struct Rotation
    {
        public float Yaw;
        public float Pitch;
        public float Roll;

        public Rotation(float yaw, float pitch, float roll)
        {
            Yaw = yaw;
            Pitch = pitch;
            Roll = roll;
        }
        public bool HasNan()
        {
            return float.IsNaN(Yaw) || float.IsNaN(Pitch) || float.IsNaN(Roll);
        }

        public static Rotation NanRotation()
        {
            Rotation val = new Rotation(float.NaN, float.NaN, float.NaN);
            return val;
        }

        public static Rotation ZeroRotation()
        {
            Rotation val = new Rotation(0.0f, 0.0f, 0.0f);
            return val;
        }
    }

    [Serializable]
    public struct Position
    {
        public float X;
        public float Y;
        public float Z;

        public Position(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }
        public bool HasNan()
        {
            return float.IsNaN(X) || float.IsNaN(Y) || float.IsNaN(Z);
        }

        public static Position NanPosition()
        {
            Position val = new Position(float.NaN, float.NaN, float.NaN);
            return val;
        }

        public static Position ZeroPosition()
        {
            Position val = new Position(0.0f, 0.0f, 0.0f);
            return val;
        }
    }

} // End Namespace


