using System;
using UnityEngine;
using System.IO;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using UnityEditor;
using SimpleJSON;
using System.Runtime.InteropServices;

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
            public string Name;
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
            public List<CameraDefaultsSettings> Cameras;
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
            public List<CameraDefaultsSettings> Cameras;
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
            public List<CameraDefaultsSettings> Cameras;
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
            public List<CameraDefaultsSettings> Cameras;
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
        public int InitialViewMode;
        #endregion


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
                TimeOfDay.StartDateTime = GetFormattedDateTime();
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

            var filename = GetAirSimSettingsLogFileName();
            NoError = CreateSettingsFileWithAllMergedSettings(filename);

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
            SpeedUnitLabel = "m/s";
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
                Name = "Default",
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
                Cameras = new List<CameraDefaultsSettings>
                {
                    GetDefaultCameraDefaultSettings()
                },
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
                Cameras = new List<CameraDefaultsSettings>
                {
                    GetDefaultCameraDefaultSettings()
                },
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
                Cameras = new List<CameraDefaultsSettings>
                {
                    GetDefaultCameraDefaultSettings()
                },
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
                Cameras = new List<CameraDefaultsSettings>
                {
                    GetDefaultCameraDefaultSettings()
                },
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

        private static string GetFormattedDateTime()
        {
            var localDate = DateTime.Now;
            string formattedDate = localDate.ToString("u", System.Globalization.CultureInfo.CreateSpecificCulture("en-US"));
            return formattedDate.TrimEnd('Z');
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
                        var camerasSettings = GetDefaultCamerasSettings();
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
                    foreach (JSONNode node in camDefaultsNode["CaptureSettings"])
                    {
                        var captureSettings = GetDefaultCaptureSettingsSettings();
                        captureSettings.ImageType = node.GetValueOrDefault("ImageType", captureSettings.ImageType);
                        captureSettings.Width = node.GetValueOrDefault("Width", captureSettings.Width);
                        captureSettings.Height = node.GetValueOrDefault("Height", captureSettings.Height);
                        captureSettings.FOV_Degrees = node.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                        captureSettings.AutoExposureSpeed = node.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                        captureSettings.AutoExposureBias = node.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                        captureSettings.AutoExposureMaxBrightness = node.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                        captureSettings.AutoExposureMinBrightness = node.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                        captureSettings.MotionBlurAmount = node.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                        captureSettings.TargetGamma = node.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                        captureSettings.ProjectionMode = node.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                        captureSettings.OrthoWidth = node.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                        CameraDefaults.CaptureSettings.Add(captureSettings);
                    }
                }
                if (camDefaultsNode.HasKey("NoiseSettings"))
                {
                    CameraDefaults.NoiseSettings.Clear();
                    foreach (JSONNode node in camDefaultsNode["NoiseSettings"])
                    {
                        var noiseSettings = GetDefaultNoiseSettingsSettings();
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
                        var UserSubWindow = new SubWindowsSettings()
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
                                var simpleFlightSetting = GetDefaultSimpleFlightSettings();                           
                                if (Vehicles.Vehicles_SimpleFlight.Any(x => x.VehicleName == vehicleName)) // If we already have this vehicleName, remove it - no dups
                                {
                                    Vehicles.Vehicles_SimpleFlight.Remove(Vehicles.Vehicles_SimpleFlight.Find(x => x.VehicleName == vehicleName));
                                }
                                simpleFlightSetting.VehicleName = vehicleName; // Must be unique
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
                                    simpleFlightSetting.Cameras.Clear();
                                    var camerasNode = vNode["Cameras"];
                                    foreach (string cameraName in camerasNode.Keys)  // such as Camera1, Camera2
                                    {
                                        var cNode = camerasNode[cameraName];
                                        var newCamera = GetDefaultCameraDefaultSettings();
                                        if (simpleFlightSetting.Cameras.Any(x => x.Name == cameraName)) // If we already have a camera with this name, remove it - no dups
                                        {
                                            simpleFlightSetting.Cameras.Remove(simpleFlightSetting.Cameras.Find(x => x.Name == cameraName));
                                        }
                                        newCamera.Name = cameraName;
                                        if (cNode.HasKey("CaptureSettings"))
                                        {
                                            newCamera.CaptureSettings.Clear();
                                            foreach (JSONNode node in cNode["CaptureSettings"])
                                            {
                                                var captureSettings = GetDefaultCaptureSettingsSettings();
                                                captureSettings.ImageType = node.GetValueOrDefault("ImageType", captureSettings.ImageType);
                                                captureSettings.Width = node.GetValueOrDefault("Width", captureSettings.Width);
                                                captureSettings.Height = node.GetValueOrDefault("Height", captureSettings.Height);
                                                captureSettings.FOV_Degrees = node.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                                                captureSettings.AutoExposureSpeed = node.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                                                captureSettings.AutoExposureBias = node.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                                                captureSettings.AutoExposureMaxBrightness = node.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                                                captureSettings.AutoExposureMinBrightness = node.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                                                captureSettings.MotionBlurAmount = node.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                                                captureSettings.TargetGamma = node.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                                                captureSettings.ProjectionMode = node.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                                                captureSettings.OrthoWidth = node.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                                                newCamera.CaptureSettings.Add(captureSettings);
                                            }
                                        }
                                        if (cNode.HasKey("NoiseSettings"))
                                        {
                                            newCamera.NoiseSettings.Clear();
                                            foreach (JSONNode node in cNode["NoiseSettings"])
                                            {
                                                var noiseSettings = GetDefaultNoiseSettingsSettings();
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
                                                newCamera.NoiseSettings.Add(noiseSettings);
                                            }
                                        }
                                        if (cNode.HasKey("Gimbal"))
                                        {
                                            var gimbalNode = cNode["Gimbal"];
                                            newCamera.Gimbal.Stabilization = gimbalNode.GetValueOrDefault("Stabilization", newCamera.Gimbal.Stabilization);
                                            var gimbalRotation = Rotation.NanRotation();
                                            gimbalRotation.Pitch = gimbalNode.GetValueOrDefault("Pitch", gimbalRotation.Pitch);
                                            gimbalRotation.Roll = gimbalNode.GetValueOrDefault("Roll", gimbalRotation.Roll);
                                            gimbalRotation.Yaw = gimbalNode.GetValueOrDefault("Yaw", gimbalRotation.Yaw);
                                            newCamera.Gimbal.Rotation = gimbalRotation;
                                        }
                                        var camPosition = Position.NanPosition();
                                        camPosition.X = cNode.GetValueOrDefault("X", camPosition.X);
                                        camPosition.Y = cNode.GetValueOrDefault("Y", camPosition.Y);
                                        camPosition.Z = cNode.GetValueOrDefault("Z", camPosition.Z);
                                        newCamera.Position = camPosition;
                                        var camRotation = Rotation.NanRotation();
                                        camRotation.Pitch = cNode.GetValueOrDefault("Pitch", camRotation.Pitch);
                                        camRotation.Roll = cNode.GetValueOrDefault("Roll", camRotation.Roll);
                                        camRotation.Yaw = cNode.GetValueOrDefault("Yaw", camRotation.Yaw);
                                        newCamera.Rotation = camRotation;
                                        simpleFlightSetting.Cameras.Add(newCamera);
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
                                if (Vehicles.Vehicles_PhysXCar.Any(x => x.VehicleName == vehicleName)) // If we already have this vehicleName, remove it - no dups
                                {
                                    Vehicles.Vehicles_PhysXCar.Remove(Vehicles.Vehicles_PhysXCar.Find(x => x.VehicleName == vehicleName));
                                }
                                physXCarSetting.VehicleName = vehicleName; // Must be unique
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
                                    physXCarSetting.Cameras.Clear();
                                    var camerasNode = vNode["Cameras"];
                                    foreach (string cameraName in camerasNode.Keys)  // such as Camera1, Camera2
                                    {
                                        var cNode = camerasNode[cameraName];
                                        var newCamera = GetDefaultCameraDefaultSettings();
                                        if (physXCarSetting.Cameras.Any(x => x.Name == cameraName)) // If we already have a camera with this name, remove it - no dups
                                        {
                                            physXCarSetting.Cameras.Remove(physXCarSetting.Cameras.Find(x => x.Name == cameraName));
                                        }
                                        newCamera.Name = cameraName;
                                        if (cNode.HasKey("CaptureSettings"))
                                        {
                                            newCamera.CaptureSettings.Clear();
                                            foreach (JSONNode node in cNode["CaptureSettings"])
                                            {
                                                var captureSettings = GetDefaultCaptureSettingsSettings();
                                                captureSettings.ImageType = node.GetValueOrDefault("ImageType", captureSettings.ImageType);
                                                captureSettings.Width = node.GetValueOrDefault("Width", captureSettings.Width);
                                                captureSettings.Height = node.GetValueOrDefault("Height", captureSettings.Height);
                                                captureSettings.FOV_Degrees = node.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                                                captureSettings.AutoExposureSpeed = node.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                                                captureSettings.AutoExposureBias = node.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                                                captureSettings.AutoExposureMaxBrightness = node.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                                                captureSettings.AutoExposureMinBrightness = node.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                                                captureSettings.MotionBlurAmount = node.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                                                captureSettings.TargetGamma = node.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                                                captureSettings.ProjectionMode = node.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                                                captureSettings.OrthoWidth = node.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                                                newCamera.CaptureSettings.Add(captureSettings);
                                            }
                                        }
                                        if (cNode.HasKey("NoiseSettings"))
                                        {
                                            newCamera.NoiseSettings.Clear();
                                            foreach (JSONNode node in cNode["NoiseSettings"])
                                            {
                                                var noiseSettings = GetDefaultNoiseSettingsSettings();
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
                                                newCamera.NoiseSettings.Add(noiseSettings);
                                            }
                                        }
                                        if (cNode.HasKey("Gimbal"))
                                        {
                                            var gimbalNode = cNode["Gimbal"];
                                            newCamera.Gimbal.Stabilization = gimbalNode.GetValueOrDefault("Stabilization", newCamera.Gimbal.Stabilization);
                                            var gimbalRotation = Rotation.NanRotation();
                                            gimbalRotation.Pitch = gimbalNode.GetValueOrDefault("Pitch", gimbalRotation.Pitch);
                                            gimbalRotation.Roll = gimbalNode.GetValueOrDefault("Roll", gimbalRotation.Roll);
                                            gimbalRotation.Yaw = gimbalNode.GetValueOrDefault("Yaw", gimbalRotation.Yaw);
                                            newCamera.Gimbal.Rotation = gimbalRotation;
                                        }
                                        var camPosition = Position.NanPosition();
                                        camPosition.X = cNode.GetValueOrDefault("X", camPosition.X);
                                        camPosition.Y = cNode.GetValueOrDefault("Y", camPosition.Y);
                                        camPosition.Z = cNode.GetValueOrDefault("Z", camPosition.Z);
                                        newCamera.Position = camPosition;
                                        var camRotation = Rotation.NanRotation();
                                        camRotation.Pitch = cNode.GetValueOrDefault("Pitch", camRotation.Pitch);
                                        camRotation.Roll = cNode.GetValueOrDefault("Roll", camRotation.Roll);
                                        camRotation.Yaw = cNode.GetValueOrDefault("Yaw", camRotation.Yaw);
                                        newCamera.Rotation = camRotation;
                                        physXCarSetting.Cameras.Add(newCamera);
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
                                if (Vehicles.Vehicles_PX4Multirotor.Any(x => x.VehicleName == vehicleName)) // If we already have this vehicleName, remove it - no dups
                                {
                                    Vehicles.Vehicles_PX4Multirotor.Remove(Vehicles.Vehicles_PX4Multirotor.Find(x => x.VehicleName == vehicleName));
                                }
                                pX4Setting.VehicleName = vehicleName; // Must be unique
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
                                    pX4Setting.Cameras.Clear();
                                    var camerasNode = vNode["Cameras"];
                                    foreach (string cameraName in camerasNode.Keys)  // such as Camera1, Camera2
                                    {
                                        var cNode = camerasNode[cameraName];
                                        var newCamera = GetDefaultCameraDefaultSettings();
                                        if (pX4Setting.Cameras.Any(x => x.Name == cameraName)) // If we already have a camera with this name, remove it - no dups
                                        {
                                            pX4Setting.Cameras.Remove(pX4Setting.Cameras.Find(x => x.Name == cameraName));
                                        }
                                        newCamera.Name = cameraName;
                                        if (cNode.HasKey("CaptureSettings"))
                                        {
                                            newCamera.CaptureSettings.Clear();
                                            foreach (JSONNode node in cNode["CaptureSettings"])
                                            {
                                                var captureSettings = GetDefaultCaptureSettingsSettings();
                                                captureSettings.ImageType = node.GetValueOrDefault("ImageType", captureSettings.ImageType);
                                                captureSettings.Width = node.GetValueOrDefault("Width", captureSettings.Width);
                                                captureSettings.Height = node.GetValueOrDefault("Height", captureSettings.Height);
                                                captureSettings.FOV_Degrees = node.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                                                captureSettings.AutoExposureSpeed = node.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                                                captureSettings.AutoExposureBias = node.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                                                captureSettings.AutoExposureMaxBrightness = node.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                                                captureSettings.AutoExposureMinBrightness = node.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                                                captureSettings.MotionBlurAmount = node.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                                                captureSettings.TargetGamma = node.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                                                captureSettings.ProjectionMode = node.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                                                captureSettings.OrthoWidth = node.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                                                newCamera.CaptureSettings.Add(captureSettings);
                                            }
                                        }
                                        if (cNode.HasKey("NoiseSettings"))
                                        {
                                            newCamera.NoiseSettings.Clear();
                                            foreach (JSONNode node in cNode["NoiseSettings"])
                                            {
                                                var noiseSettings = GetDefaultNoiseSettingsSettings();
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
                                                newCamera.NoiseSettings.Add(noiseSettings);
                                            }
                                        }
                                        if (cNode.HasKey("Gimbal"))
                                        {
                                            var gimbalNode = cNode["Gimbal"];
                                            newCamera.Gimbal.Stabilization = gimbalNode.GetValueOrDefault("Stabilization", newCamera.Gimbal.Stabilization);
                                            var gimbalRotation = Rotation.NanRotation();
                                            gimbalRotation.Pitch = gimbalNode.GetValueOrDefault("Pitch", gimbalRotation.Pitch);
                                            gimbalRotation.Roll = gimbalNode.GetValueOrDefault("Roll", gimbalRotation.Roll);
                                            gimbalRotation.Yaw = gimbalNode.GetValueOrDefault("Yaw", gimbalRotation.Yaw);
                                            newCamera.Gimbal.Rotation = gimbalRotation;
                                        }
                                        var camPosition = Position.NanPosition();
                                        camPosition.X = cNode.GetValueOrDefault("X", camPosition.X);
                                        camPosition.Y = cNode.GetValueOrDefault("Y", camPosition.Y);
                                        camPosition.Z = cNode.GetValueOrDefault("Z", camPosition.Z);
                                        newCamera.Position = camPosition;
                                        var camRotation = Rotation.NanRotation();
                                        camRotation.Pitch = cNode.GetValueOrDefault("Pitch", camRotation.Pitch);
                                        camRotation.Roll = cNode.GetValueOrDefault("Roll", camRotation.Roll);
                                        camRotation.Yaw = cNode.GetValueOrDefault("Yaw", camRotation.Yaw);
                                        newCamera.Rotation = camRotation;
                                        pX4Setting.Cameras.Add(newCamera);
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
                                if (Vehicles.Vehicles_ComputerVision.Any(x => x.VehicleName == vehicleName))
                                {
                                    Vehicles.Vehicles_ComputerVision.Remove(Vehicles.Vehicles_ComputerVision.Find(x => x.VehicleName == vehicleName));
                                }
                                computerVisionSetting.VehicleName = vehicleName; // Must be unique
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
                                    computerVisionSetting.Cameras.Clear();
                                    var camerasNode = vNode["Cameras"];
                                    foreach (string cameraName in camerasNode.Keys)  // such as Camera1, Camera2
                                    {
                                        var cNode = camerasNode[cameraName];
                                        var newCamera = GetDefaultCameraDefaultSettings();
                                        if (computerVisionSetting.Cameras.Any(x => x.Name == cameraName)) // If we already have a camera with this name, remove it - no dups
                                        {
                                            computerVisionSetting.Cameras.Remove(computerVisionSetting.Cameras.Find(x => x.Name == cameraName));
                                        }
                                        newCamera.Name = cameraName;
                                        if (cNode.HasKey("CaptureSettings"))
                                        {
                                            newCamera.CaptureSettings.Clear();
                                            foreach (JSONNode node in cNode["CaptureSettings"])
                                            {
                                                var captureSettings = GetDefaultCaptureSettingsSettings();
                                                captureSettings.ImageType = node.GetValueOrDefault("ImageType", captureSettings.ImageType);
                                                captureSettings.Width = node.GetValueOrDefault("Width", captureSettings.Width);
                                                captureSettings.Height = node.GetValueOrDefault("Height", captureSettings.Height);
                                                captureSettings.FOV_Degrees = node.GetValueOrDefault("FOV_Degrees", captureSettings.FOV_Degrees);
                                                captureSettings.AutoExposureSpeed = node.GetValueOrDefault("AutoExposureSpeed", captureSettings.AutoExposureSpeed);
                                                captureSettings.AutoExposureBias = node.GetValueOrDefault("AutoExposureBias", captureSettings.AutoExposureBias);
                                                captureSettings.AutoExposureMaxBrightness = node.GetValueOrDefault("AutoExposureMaxBrightness", captureSettings.AutoExposureMaxBrightness);
                                                captureSettings.AutoExposureMinBrightness = node.GetValueOrDefault("AutoExposureMinBrightness", captureSettings.AutoExposureMinBrightness);
                                                captureSettings.MotionBlurAmount = node.GetValueOrDefault("MotionBlurAmount", captureSettings.MotionBlurAmount);
                                                captureSettings.TargetGamma = node.GetValueOrDefault("TargetGamma", captureSettings.TargetGamma);
                                                captureSettings.ProjectionMode = node.GetValueOrDefault("ProjectionMode", captureSettings.ProjectionMode);
                                                captureSettings.OrthoWidth = node.GetValueOrDefault("OrthoWidth", captureSettings.OrthoWidth);
                                                newCamera.CaptureSettings.Add(captureSettings);
                                            }
                                        }
                                        if (cNode.HasKey("NoiseSettings"))
                                        {
                                            newCamera.NoiseSettings.Clear();
                                            foreach (JSONNode node in cNode["NoiseSettings"])
                                            {
                                                var noiseSettings = GetDefaultNoiseSettingsSettings();
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
                                                newCamera.NoiseSettings.Add(noiseSettings);
                                            }
                                        }
                                        if (cNode.HasKey("Gimbal"))
                                        {
                                            var gimbalNode = cNode["Gimbal"];
                                            newCamera.Gimbal.Stabilization = gimbalNode.GetValueOrDefault("Stabilization", newCamera.Gimbal.Stabilization);
                                            var gimbalRotation = Rotation.NanRotation();
                                            gimbalRotation.Pitch = gimbalNode.GetValueOrDefault("Pitch", gimbalRotation.Pitch);
                                            gimbalRotation.Roll = gimbalNode.GetValueOrDefault("Roll", gimbalRotation.Roll);
                                            gimbalRotation.Yaw = gimbalNode.GetValueOrDefault("Yaw", gimbalRotation.Yaw);
                                            newCamera.Gimbal.Rotation = gimbalRotation;
                                        }
                                        var camPosition = Position.NanPosition();
                                        camPosition.X = cNode.GetValueOrDefault("X", camPosition.X);
                                        camPosition.Y = cNode.GetValueOrDefault("Y", camPosition.Y);
                                        camPosition.Z = cNode.GetValueOrDefault("Z", camPosition.Z);
                                        newCamera.Position = camPosition;
                                        var camRotation = Rotation.NanRotation();
                                        camRotation.Pitch = cNode.GetValueOrDefault("Pitch", camRotation.Pitch);
                                        camRotation.Roll = cNode.GetValueOrDefault("Roll", camRotation.Roll);
                                        camRotation.Yaw = cNode.GetValueOrDefault("Yaw", camRotation.Yaw);
                                        newCamera.Rotation = camRotation;
                                        computerVisionSetting.Cameras.Add(newCamera);
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

        private string GetAirSimSettingsLogFileName()
        {
            string settingsName = "settings_log_" + DateTime.Now.ToString("yyyy-dd-M--HH-mm-ss") + ".json";

            if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
            {
                string linuxFileName = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), Path.Combine("Documents/AirSim", settingsName));
                return linuxFileName;
            }
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                string fileName = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), Path.Combine("AirSim", settingsName));
                return fileName;
            }
            else
            {
                Debug.Log("Notice: Could not determing OSPlatform.");
                return "";
            }
        }

        private bool CreateSettingsFileWithMinRequiredValues(string fileName)
        {
            Debug.Log("Create filename: " + fileName);
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
            Debug.Log("Merge filename: " + fileName);
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
                string lf = Environment.NewLine;
                string t1 = lf + "\t";
                string t2 = t1 + "\t";
                string t3 = t2 + "\t";
                string t4 = t3 + "\t";
                string t5 = t4 + "\t";
                string t6 = t5 + "\t";

                StringBuilder sb = new StringBuilder(10000);
                sb.Append("{");
                var airSimUri = new Uri("https://github.com/Microsoft/AirSim/blob/master/docs/settings.md");
                sb.Append($"{t1}\"SeeDocsAt\" : \"{airSimUri.ToString()}\",");
                sb.Append($"{t1}\"SettingsVersion\" : {SettingsVersion.ToString()},");
                sb.Append($"{t1}\"SimMode\" : \"{SimMode}\",");
                sb.Append($"{t1}\"ClockType\" : \"{ClockType}\",");
                sb.Append($"{t1}\"ClockSpeed\" : {ClockSpeed.ToString()},");
                sb.Append($"{t1}\"LocalHostIp\" : \"{LocalHostIp}\",");
                sb.Append($"{t1}\"ApiServerAddress\" : \"{ApiServerAddress}\",");
                sb.Append($"{t1}\"RecordUIVisible\" : {RecordUIVisible.ToString().ToLower()},");
                sb.Append($"{t1}\"LogMessagesVisible\" : {LogMessagesVisible.ToString().ToLower()},");
                sb.Append($"{t1}\"ViewMode\" : \"{ViewMode}\",");
                sb.Append($"{t1}\"RpcEnabled\" : {RpcEnabled.ToString().ToLower()},");
                sb.Append($"{t1}\"EngineSound\" : {EngineSound.ToString().ToLower()},");
                sb.Append($"{t1}\"PhysicsEngineName\" : \"{PhysicsEngineName}\",");
                sb.Append($"{t1}\"SpeedUnitFactor\" : {SpeedUnitFactor.ToString()},");
                sb.Append($"{t1}\"SpeedUnitLabel\" : \"{SpeedUnitLabel}\",");

                sb.Append($"{t1}\"Recording\" : {{");
                sb.Append($"{t2}\"RecordOnMove\" : {Recording.RecordOnMove.ToString().ToLower()},");
                sb.Append($"{t2}\"RecordInterval\" : {Recording.RecordInterval.ToString()},");
                sb.Append($"{t2}\"Cameras\" : [");
                foreach (var cam in Recording.Cameras)
                {
                    sb.Append($"{t3}{{");
                    sb.Append($"{t3}\"CameraName\" : \"{cam.CameraName}\",");
                    sb.Append($"{t3}\"ImageType\" : {cam.ImageType.ToString()},");
                    sb.Append($"{t3}\"PixelAsFloat\" : {cam.PixelAsFloat.ToString().ToLower()},");
                    sb.Append($"{t3}\"Compress\" : {cam.Compress.ToString().ToLower()}");
                    sb.Append($"{t3}}},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append($"{t2}]");
                sb.Append($"{t1}}},");

                sb.Append($"{t1}\"CameraDefaults\" : {{");
                sb.Append($"{t2}\"CaptureSettings\" : [");
                foreach (var capSet in CameraDefaults.CaptureSettings)
                {
                    sb.Append($"{t3}{{");
                    sb.Append($"{t3}\"ImageType\" : {capSet.ImageType.ToString()},");
                    sb.Append($"{t3}\"Width\" : {capSet.Width.ToString()},");
                    sb.Append($"{t3}\"Height\" : {capSet.Height.ToString()},");
                    sb.Append($"{t3}\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                    sb.Append($"{t3}\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                    sb.Append($"{t3}\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                    sb.Append($"{t3}\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                    sb.Append($"{t3}\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                    sb.Append($"{t3}\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                    sb.Append($"{t3}\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                    sb.Append($"{t3}\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                    sb.Append($"{t3}\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                    sb.Append($"{t3}}},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append($"{t2}],");

                sb.Append($"{t2}\"NoiseSettings\" : [");
                foreach (var noiseSet in CameraDefaults.NoiseSettings)
                {
                    sb.Append($"{t3}{{");
                    sb.Append($"{t3}\"Enabled\" : {noiseSet.Enabled.ToString().ToLower()},");
                    sb.Append($"{t3}\"ImageType\" : {noiseSet.ImageType.ToString()},");
                    sb.Append($"{t3}\"RandContrib\" : {noiseSet.RandContrib.ToString()},");
                    sb.Append($"{t3}\"RandSpeed\" : {noiseSet.RandSpeed.ToString()},");
                    sb.Append($"{t3}\"RandSize\" : {noiseSet.RandSize.ToString()},");
                    sb.Append($"{t3}\"RandDensity\" : {noiseSet.RandDensity.ToString()},");
                    sb.Append($"{t3}\"HorzWaveContrib\" : {noiseSet.HorzWaveContrib.ToString()},");
                    sb.Append($"{t3}\"HorzWaveStrength\" : {noiseSet.HorzWaveStrength.ToString()},");
                    sb.Append($"{t3}\"HorzWaveVertSize\" : {noiseSet.HorzWaveVertSize.ToString()},");
                    sb.Append($"{t3}\"HorzWaveScreenSize\" : {noiseSet.HorzWaveScreenSize.ToString()},");
                    sb.Append($"{t3}\"HorzNoiseLinesContrib\" : {noiseSet.HorzNoiseLinesContrib.ToString()},");
                    sb.Append($"{t3}\"HorzNoiseLinesDensityY\" : {noiseSet.HorzNoiseLinesDensityY.ToString()},");
                    sb.Append($"{t3}\"HorzNoiseLinesDensityXY\" : {noiseSet.HorzNoiseLinesDensityXY.ToString()},");
                    sb.Append($"{t3}\"HorzDistortionContrib\" : {noiseSet.HorzDistortionContrib.ToString()},");
                    sb.Append($"{t3}\"HorzDistortionStrength\" : {noiseSet.HorzDistortionStrength.ToString()}");
                    sb.Append($"{t3}}},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append($"{t2}],");

                sb.Append($"{t2}\"Gimbal\" : {{");
                sb.Append($"{t3}\"Stabilization\" : {CameraDefaults.Gimbal.Stabilization.ToString()},");
                if (!float.IsNaN(CameraDefaults.Gimbal.Rotation.Pitch))
                {
                    sb.Append($"{t3}\"Pitch\" : {CameraDefaults.Gimbal.Rotation.Pitch.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Gimbal.Rotation.Roll))
                {
                    sb.Append($"{t3}\"Roll\" : {CameraDefaults.Gimbal.Rotation.Roll.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Gimbal.Rotation.Yaw))
                {
                    sb.Append($"{t3}\"Yaw\" : {CameraDefaults.Gimbal.Rotation.Yaw.ToString()},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append($"{t2}}},");  // End Gimabl
                if (!float.IsNaN(CameraDefaults.Position.X))
                {
                    sb.Append($"{t2}\"X\" : {CameraDefaults.Position.X.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Position.Y))
                {
                    sb.Append($"{t2}\"Y\" : {CameraDefaults.Position.Y.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Position.Z))
                {
                    sb.Append($"{t2}\"Z\" : {CameraDefaults.Position.Z.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Rotation.Pitch))
                {
                    sb.Append($"{t2}\"Pitch\" : {CameraDefaults.Rotation.Pitch.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Rotation.Roll))
                {
                    sb.Append($"{t2}\"Roll\" : {CameraDefaults.Rotation.Roll.ToString()},");
                }
                if (!float.IsNaN(CameraDefaults.Rotation.Yaw))
                {
                    sb.Append($"{t2}\"Yaw\" : {CameraDefaults.Rotation.Yaw.ToString()},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append($"{t1}}},"); // End CameraDefaults

                sb.Append($"{t1}\"OriginGeopoint\" : {{");
                sb.Append($"{t2}\"Latitude\" : {OriginGeopoint.Latitude.ToString()},");
                sb.Append($"{t2}\"Longitude\" : {OriginGeopoint.Longitude.ToString()},");
                sb.Append($"{t2}\"Altitude\" : {OriginGeopoint.Altitude.ToString()}");
                sb.Append($"{t1}}},"); // End OriginGeopoint

                sb.Append($"{t1}\"TimeOfDay\" : {{");
                sb.Append($"{t2}\"Enabled\" : {TimeOfDay.Enabled.ToString().ToLower()},");
                sb.Append($"{t2}\"StartDateTime\" : \"{TimeOfDay.StartDateTime}\",");
                sb.Append($"{t2}\"CelestialClockSpeed\" : {TimeOfDay.CelestialClockSpeed.ToString()},");
                sb.Append($"{t2}\"StartDateTimeDst\" : {TimeOfDay.StartDateTimeDst.ToString().ToLower()},");
                sb.Append($"{t2}\"UpdateIntervalSecs\" : {TimeOfDay.UpdateIntervalSecs.ToString()},");
                sb.Append($"{t2}\"MoveSun\" : {TimeOfDay.MoveSun.ToString().ToLower()}");
                sb.Append($"{t1}}},"); // End TimeOfDay

                sb.Append($"{t1}\"SubWindows\" : [");
                foreach (var subWindow in SubWindows)
                {
                    sb.Append($"{t3}{{");
                    sb.Append($"{t3}\"WindowID\" : {subWindow.WindowID.ToString()},");
                    sb.Append($"{t3}\"CameraName\" : \"{subWindow.CameraName}\",");
                    sb.Append($"{t3}\"ImageType\" : {subWindow.ImageType.ToString()},");
                    sb.Append($"{t3}\"Visible\" : {subWindow.Visible.ToString().ToLower()}");
                    sb.Append($"{t3}}},");
                }
                sb.Remove(sb.Length - 1, 1);
                sb.Append($"{t1}],"); // End SubWindows

                sb.Append($"{t1}\"SegmentationSettings\" : {{");
                sb.Append($"{t2}\"InitMethod\" : \"{SegmentationSettings.InitMethod}\",");
                sb.Append($"{t2}\"MeshNamingMethod\" : \"{SegmentationSettings.MeshNamingMethod}\",");
                sb.Append($"{t2}\"OverrideExisting\" : {SegmentationSettings.OverrideExisting.ToString().ToLower()}");
                sb.Append($"{t1}}},");

                sb.Append($"{t1}\"PawnPaths\" : {{");
                sb.Append($"{t2}\"BareboneCar\" : {{");
                sb.Append($"{t3}\"PawnBP\" : \"{PawnPaths.BareboneCar.PawnBP}\"");
                sb.Append($"{t2}}},");
                sb.Append($"{t2}\"DefaultCar\" : {{");
                sb.Append($"{t3}\"PawnBP\" : \"{PawnPaths.DefaultCar.PawnBP}\"");
                sb.Append($"{t2}}},");
                sb.Append($"{t2}\"DefaultQuadrotor\" : {{");
                sb.Append($"{t3}\"PawnBP\" : \"{PawnPaths.DefaultQuadrotor.PawnBP}\"");
                sb.Append($"{t2}}},");
                sb.Append($"{t2}\"DefaultComputerVision\" : {{");
                sb.Append($"{t3}\"PawnBP\" : \"{PawnPaths.DefaultComputerVision.PawnBP}\"");
                sb.Append($"{t2}}}");
                sb.Append($"{t1}}},"); // End PawnPaths

                sb.Append($"{t1}\"Vehicles\" : {{");
                foreach (var vehicle in Vehicles.Vehicles_SimpleFlight)
                {
                    if (vehicle.VehicleType == "SimpleFlight")
                    {
                        sb.Append($"{t2}\"{vehicle.VehicleName}\" : {{");
                        sb.Append($"{t3}\"VehicleType\" : \"{vehicle.VehicleType}\",");
                        sb.Append($"{t3}\"DefaultVehicleState\" : \"{vehicle.DefaultVehicleState}\",");
                        sb.Append($"{t3}\"AutoCreate\" : {vehicle.AutoCreate.ToString().ToLower()},");
                        sb.Append($"{t3}\"EnableTrace\" : {vehicle.EnableTrace.ToString().ToLower()},");
                        sb.Append($"{t3}\"PawnPath\" : \"{vehicle.PawnPath}\",");
                        sb.Append($"{t3}\"EnableCollisionPassthrogh\" : {vehicle.EnableCollisionPassthrough.ToString().ToLower()},");
                        sb.Append($"{t3}\"EnableCollisions\" : {vehicle.EnableCollisions.ToString().ToLower()},");
                        sb.Append($"{t3}\"IsFpvVehicle\" : {vehicle.IsFpvVehicle.ToString().ToLower()},");
                        sb.Append($"{t3}\"AllowAPIAlways\" : {vehicle.AllowAPIAlways.ToString().ToLower()},");
                        sb.Append($"{t3}\"RC\" : {{");
                        sb.Append($"{t4}\"RemoteControlID\" : {vehicle.RC.RemoteControlID.ToString()},");
                        sb.Append($"{t4}\"AllowAPIWhenDisconnected\" : {vehicle.RC.AllowAPIWhenDisconnected.ToString().ToLower()}");
                        sb.Append($"{t3}}},"); // End RC
                        sb.Append($"{t3}\"Cameras\" : {{");
                        
                        foreach (var cam in vehicle.Cameras)
                        {
                            sb.Append($"{t4}\"{cam.Name}\" : {{");
                            sb.Append($"{t5}\"CaptureSettings\" : [");
                            foreach (var capSet in cam.CaptureSettings)
                            {
                                sb.Append($"{t6}{{");
                                sb.Append($"{t6}\"ImageType\" : {capSet.ImageType.ToString()},");
                                sb.Append($"{t6}\"Width\" : {capSet.Width.ToString()},");
                                sb.Append($"{t6}\"Height\" : {capSet.Height.ToString()},");
                                sb.Append($"{t6}\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                                sb.Append($"{t6}\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                                sb.Append($"{t6}\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                                sb.Append($"{t6}\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                                sb.Append($"{t6}\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                                sb.Append($"{t6}\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                                sb.Append($"{t6}\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                                sb.Append($"{t6}\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                                sb.Append($"{t6}\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                                sb.Append($"{t6}}},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}],");
                            sb.Append($"{t5}\"NoiseSettings\" : [");
                            foreach (var noiseSet in cam.NoiseSettings)
                            {
                                sb.Append($"{t6}{{");
                                sb.Append($"{t6}\"Enabled\" : {noiseSet.Enabled.ToString().ToLower()},");
                                sb.Append($"{t6}\"ImageType\" : {noiseSet.ImageType.ToString()},");
                                sb.Append($"{t6}\"RandContrib\" : {noiseSet.RandContrib.ToString()},");
                                sb.Append($"{t6}\"RandSpeed\" : {noiseSet.RandSpeed.ToString()},");
                                sb.Append($"{t6}\"RandSize\" : {noiseSet.RandSize.ToString()},");
                                sb.Append($"{t6}\"RandDensity\" : {noiseSet.RandDensity.ToString()},");
                                sb.Append($"{t6}\"HorzWaveContrib\" : {noiseSet.HorzWaveContrib.ToString()},");
                                sb.Append($"{t6}\"HorzWaveStrength\" : {noiseSet.HorzWaveStrength.ToString()},");
                                sb.Append($"{t6}\"HorzWaveVertSize\" : {noiseSet.HorzWaveVertSize.ToString()},");
                                sb.Append($"{t6}\"HorzWaveScreenSize\" : {noiseSet.HorzWaveScreenSize.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesContrib\" : {noiseSet.HorzNoiseLinesContrib.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesDensityY\" : {noiseSet.HorzNoiseLinesDensityY.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesDensityXY\" : {noiseSet.HorzNoiseLinesDensityXY.ToString()},");
                                sb.Append($"{t6}\"HorzDistortionContrib\" : {noiseSet.HorzDistortionContrib.ToString()},");
                                sb.Append($"{t6}\"HorzDistortionStrength\" : {noiseSet.HorzDistortionStrength.ToString()}");
                                sb.Append($"{t6}}},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}],");
                            sb.Append($"{t5}\"Gimbal\" : {{");
                            sb.Append($"{t6}\"Stabilization\" : {cam.Gimbal.Stabilization.ToString()},");
                            if (!float.IsNaN(cam.Gimbal.Rotation.Pitch))
                            {
                                sb.Append($"{t6}\"Pitch\" : {cam.Gimbal.Rotation.Pitch.ToString()},");
                            }
                            if (!float.IsNaN(cam.Gimbal.Rotation.Roll))
                            {
                                sb.Append($"{t6}\"Roll\" : {cam.Gimbal.Rotation.Roll.ToString()},");
                            }
                            if (!float.IsNaN(cam.Gimbal.Rotation.Yaw))
                            {
                                sb.Append($"{t6}\"Yaw\" : {cam.Gimbal.Rotation.Yaw.ToString()},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}}},");  // End Gimabl
                            // Note: Camera position and rotation settings currently if omitted will cause Unreal to crash: Issue #1836
                            // Therefore we will at least write 0.0 for any unspecified (NaN) values
                            if (!float.IsNaN(cam.Position.X))
                            {
                                sb.Append($"{t5}\"X\" : {cam.Position.X.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"X\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Position.Y))
                            {
                                sb.Append($"{t5}\"Y\" : {cam.Position.Y.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Y\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Position.Z))
                            {
                                sb.Append($"{t5}\"Z\" : {cam.Position.Z.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Z\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Pitch))
                            {
                                sb.Append($"{t5}\"Pitch\" : {cam.Rotation.Pitch.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Pitch\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Roll))
                            {
                                sb.Append($"{t5}\"Roll\" : {cam.Rotation.Roll.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Roll\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Yaw))
                            {
                                sb.Append($"{t5}\"Yaw\" : {cam.Rotation.Yaw.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Yaw\" : 0.0,");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t4}}},"); // End this cam
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append($"{t3}}},"); // End Cameras
                        if (!float.IsNaN(vehicle.Position.X))
                        {
                            sb.Append($"{t3}\"X\" : {vehicle.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Y))
                        {
                            sb.Append($"{t3}\"Y\" : {vehicle.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Z))
                        {
                            sb.Append($"{t3}\"Z\" : {vehicle.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Pitch))
                        {
                            sb.Append($"{t3}\"Pitch\" : {vehicle.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Roll))
                        {
                            sb.Append($"{t3}\"Roll\" : {vehicle.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Yaw))
                        {
                            sb.Append($"{t3}\"Yaw\" : {vehicle.Rotation.Yaw.ToString()},");
                        }
                        sb.Append($"{t3}\"Sensors\" : {{");
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.BarometerList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Barometer)
                            {
                                sb.Append($"{t4}\"Barometer{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 1,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.ImuList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Imu)
                            {
                                sb.Append($"{t4}\"Imu{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 2,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.GpsList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Gps)
                            {
                                sb.Append($"{t4}\"Gps{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 3,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.MagnetometerList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Magnetometer)
                            {
                                sb.Append($"{t4}\"Magnetometer{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 4,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.DistanceList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Distance)
                            {
                                sb.Append($"{t4}\"Distance{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 5,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.LidarList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Lidar)
                            {
                                sb.Append($"{t4}\"Lidar{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 6,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()},");
                                sb.Append($"{t5}\"NumberOfChannels\" : {sensor.NumberOfChannels.ToString()},");
                                sb.Append($"{t5}\"Range\" : {sensor.Range.ToString()},");
                                sb.Append($"{t5}\"RotationsPerSecond\" : {sensor.RotationsPerSecond.ToString()},");
                                sb.Append($"{t5}\"PointsPerSecond\" : {sensor.PointsPerSecond.ToString()},");
                                if (!float.IsNaN(sensor.Position.X))
                                {
                                    sb.Append($"{t5}\"X\" : {sensor.Position.X.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Y))
                                {
                                    sb.Append($"{t5}\"Y\" : {sensor.Position.Y.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Z))
                                {
                                    sb.Append($"{t5}\"Z\" : {sensor.Position.Z.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Pitch))
                                {
                                    sb.Append($"{t5}\"Pitch\" : {sensor.Rotation.Pitch.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Roll))
                                {
                                    sb.Append($"{t5}\"Roll\" : {sensor.Rotation.Roll.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Yaw))
                                {
                                    sb.Append($"{t5}\"Yaw\" : {sensor.Rotation.Yaw.ToString()},");
                                }
                                sb.Append($"{t5}\"VerticalFOVUpper\" : {sensor.VerticalFOVUpper.ToString()},");
                                sb.Append($"{t5}\"VerticalFOVLower\" : {sensor.VerticalFOVLower.ToString()},");
                                sb.Append($"{t5}\"HorizontalFOVStart\" : {sensor.HorizontalFOVStart.ToString()},");
                                sb.Append($"{t5}\"HorizontalFOVEnd\" : {sensor.HorizontalFOVEnd.ToString()},");
                                sb.Append($"{t5}\"DrawDebugPoints\" : {sensor.DrawDebugPoints.ToString().ToLower()},");
                                sb.Append($"{t5}\"DataFrame\" : \"{sensor.DataFrame}\"");
                                sb.Append($"{t4}}},");
                            }
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append($"{t3}}}"); // End This SimpleFlight Sensor
                        sb.Append($"{t2}}},"); // End This SimpleFlight
                    } // End if
                } // End foreach

                foreach (var vehicle in Vehicles.Vehicles_PhysXCar)
                {
                    if (vehicle.VehicleType == "PhysXCar")
                    {
                        sb.Append($"{t2}\"{vehicle.VehicleName}\" : {{");
                        sb.Append($"{t3}\"VehicleType\" : \"{vehicle.VehicleType}\",");
                        sb.Append($"{t3}\"DefaultVehicleState\" : \"{vehicle.DefaultVehicleState}\",");
                        sb.Append($"{t3}\"AutoCreate\" : {vehicle.AutoCreate.ToString().ToLower()},");
                        sb.Append($"{t3}\"EnableTrace\" : {vehicle.EnableTrace.ToString().ToLower()},");
                        sb.Append($"{t3}\"PawnPath\" : \"{vehicle.PawnPath}\",");
                        sb.Append($"{t3}\"EnableCollisionPassthrogh\" : {vehicle.EnableCollisionPassthrogh.ToString().ToLower()},");
                        sb.Append($"{t3}\"EnableCollisions\" : {vehicle.EnableCollisions.ToString().ToLower()},");
                        sb.Append($"{t3}\"RC\" : {{");
                        sb.Append($"{t4}\"RemoteControlID\" : {vehicle.RC.RemoteControlID.ToString()}");
                        sb.Append($"{t3}}},"); // End RC
                        sb.Append($"{t3}\"Cameras\" : {{");
                        foreach (var cam in vehicle.Cameras)
                        {
                            sb.Append($"{t4}\"{cam.Name}\" : {{");
                            sb.Append($"{t5}\"CaptureSettings\" : [");
                            foreach (var capSet in cam.CaptureSettings)
                            {
                                sb.Append($"{t6}{{");
                                sb.Append($"{t6}\"ImageType\" : {capSet.ImageType.ToString()},");
                                sb.Append($"{t6}\"Width\" : {capSet.Width.ToString()},");
                                sb.Append($"{t6}\"Height\" : {capSet.Height.ToString()},");
                                sb.Append($"{t6}\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                                sb.Append($"{t6}\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                                sb.Append($"{t6}\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                                sb.Append($"{t6}\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                                sb.Append($"{t6}\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                                sb.Append($"{t6}\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                                sb.Append($"{t6}\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                                sb.Append($"{t6}\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                                sb.Append($"{t6}\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                                sb.Append($"{t6}}},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}],");
                            sb.Append($"{t5}\"NoiseSettings\" : [");
                            foreach (var noiseSet in cam.NoiseSettings)
                            {
                                sb.Append($"{t6}{{");
                                sb.Append($"{t6}\"Enabled\" : {noiseSet.Enabled.ToString().ToLower()},");
                                sb.Append($"{t6}\"ImageType\" : {noiseSet.ImageType.ToString()},");
                                sb.Append($"{t6}\"RandContrib\" : {noiseSet.RandContrib.ToString()},");
                                sb.Append($"{t6}\"RandSpeed\" : {noiseSet.RandSpeed.ToString()},");
                                sb.Append($"{t6}\"RandSize\" : {noiseSet.RandSize.ToString()},");
                                sb.Append($"{t6}\"RandDensity\" : {noiseSet.RandDensity.ToString()},");
                                sb.Append($"{t6}\"HorzWaveContrib\" : {noiseSet.HorzWaveContrib.ToString()},");
                                sb.Append($"{t6}\"HorzWaveStrength\" : {noiseSet.HorzWaveStrength.ToString()},");
                                sb.Append($"{t6}\"HorzWaveVertSize\" : {noiseSet.HorzWaveVertSize.ToString()},");
                                sb.Append($"{t6}\"HorzWaveScreenSize\" : {noiseSet.HorzWaveScreenSize.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesContrib\" : {noiseSet.HorzNoiseLinesContrib.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesDensityY\" : {noiseSet.HorzNoiseLinesDensityY.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesDensityXY\" : {noiseSet.HorzNoiseLinesDensityXY.ToString()},");
                                sb.Append($"{t6}\"HorzDistortionContrib\" : {noiseSet.HorzDistortionContrib.ToString()},");
                                sb.Append($"{t6}\"HorzDistortionStrength\" : {noiseSet.HorzDistortionStrength.ToString()}");
                                sb.Append($"{t6}}},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}],");
                            sb.Append($"{t5}\"Gimbal\" : {{");
                            sb.Append($"{t6}\"Stabilization\" : {cam.Gimbal.Stabilization.ToString()},");
                            if (!float.IsNaN(cam.Gimbal.Rotation.Pitch))
                            {
                                sb.Append($"{t6}\"Pitch\" : {cam.Gimbal.Rotation.Pitch.ToString()},");
                            }
                            if (!float.IsNaN(cam.Gimbal.Rotation.Roll))
                            {
                                sb.Append($"{t6}\"Roll\" : {cam.Gimbal.Rotation.Roll.ToString()},");
                            }
                            if (!float.IsNaN(cam.Gimbal.Rotation.Yaw))
                            {
                                sb.Append($"{t6}\"Yaw\" : {cam.Gimbal.Rotation.Yaw.ToString()},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}}},");  // End Gimabl
                            // Note: Camera position and rotation settings currently if omitted will cause Unreal to crash: Issue #1836
                            // Therefore we will at least write 0.0 for any unspecified (NaN) values
                            if (!float.IsNaN(cam.Position.X))
                            {
                                sb.Append($"{t5}\"X\" : {cam.Position.X.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"X\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Position.Y))
                            {
                                sb.Append($"{t5}\"Y\" : {cam.Position.Y.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Y\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Position.Z))
                            {
                                sb.Append($"{t5}\"Z\" : {cam.Position.Z.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Z\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Pitch))
                            {
                                sb.Append($"{t5}\"Pitch\" : {cam.Rotation.Pitch.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Pitch\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Roll))
                            {
                                sb.Append($"{t5}\"Roll\" : {cam.Rotation.Roll.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Roll\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Yaw))
                            {
                                sb.Append($"{t5}\"Yaw\" : {cam.Rotation.Yaw.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Yaw\" : 0.0,");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t4}}},"); // End this cam
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append($"{t3}}},"); // End Cameras
                        if (!float.IsNaN(vehicle.Position.X))
                        {
                            sb.Append($"{t3}\"X\" : {vehicle.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Y))
                        {
                            sb.Append($"{t3}\"Y\" : {vehicle.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Z))
                        {
                            sb.Append($"{t3}\"Z\" : {vehicle.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Pitch))
                        {
                            sb.Append($"{t3}\"Pitch\" : {vehicle.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Roll))
                        {
                            sb.Append($"{t3}\"Roll\" : {vehicle.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Yaw))
                        {
                            sb.Append($"{t3}\"Yaw\" : {vehicle.Rotation.Yaw.ToString()},");
                        }
                        sb.Append($"{t3}\"Sensors\" : {{");
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.GpsList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Gps)
                            {
                                sb.Append($"{t4}\"Gps{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 3,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.LidarList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Lidar)
                            {
                                sb.Append($"{t4}\"Lidar{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 6,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()},");
                                sb.Append($"{t5}\"NumberOfChannels\" : {sensor.NumberOfChannels.ToString()},");
                                sb.Append($"{t5}\"Range\" : {sensor.Range.ToString()},");
                                sb.Append($"{t5}\"RotationsPerSecond\" : {sensor.RotationsPerSecond.ToString()},");
                                sb.Append($"{t5}\"PointsPerSecond\" : {sensor.PointsPerSecond.ToString()},");
                                if (!float.IsNaN(sensor.Position.X))
                                {
                                    sb.Append($"{t5}\"X\" : {sensor.Position.X.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Y))
                                {
                                    sb.Append($"{t5}\"Y\" : {sensor.Position.Y.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Z))
                                {
                                    sb.Append($"{t5}\"Z\" : {sensor.Position.Z.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Pitch))
                                {
                                    sb.Append($"{t5}\"Pitch\" : {sensor.Rotation.Pitch.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Roll))
                                {
                                    sb.Append($"{t5}\"Roll\" : {sensor.Rotation.Roll.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Yaw))
                                {
                                    sb.Append($"{t5}\"Yaw\" : {sensor.Rotation.Yaw.ToString()},");
                                }
                                sb.Append($"{t5}\"VerticalFOVUpper\" : {sensor.VerticalFOVUpper.ToString()},");
                                sb.Append($"{t5}\"VerticalFOVLower\" : {sensor.VerticalFOVLower.ToString()},");
                                sb.Append($"{t5}\"HorizontalFOVStart\" : {sensor.HorizontalFOVStart.ToString()},");
                                sb.Append($"{t5}\"HorizontalFOVEnd\" : {sensor.HorizontalFOVEnd.ToString()},");
                                sb.Append($"{t5}\"DrawDebugPoints\" : {sensor.DrawDebugPoints.ToString().ToLower()},");
                                sb.Append($"{t5}\"DataFrame\" : \"{sensor.DataFrame}\"");
                                sb.Append($"{t4}}},");
                            }
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append($"{t3}}}"); // End This PhysXCar Sensors
                        sb.Append($"{t2}}},"); // End This PhysXCar
                    } // End if
                }

                foreach (var vehicle in Vehicles.Vehicles_PX4Multirotor)
                {
                    if (vehicle.VehicleType == "PX4Multirotor")
                    {
                        sb.Append($"{t2}\"{vehicle.VehicleName}\" : {{");
                        sb.Append($"{t3}\"VehicleType\" : \"{vehicle.VehicleType}\",");
                        sb.Append($"{t3}\"LogViewerPort\" : {vehicle.LogViewerPort.ToString()},");
                        //sb.Append($"{t3}\"LogViewerSendPort\" : {vehicle.LogViewerSendPort.ToString()},");
                        sb.Append($"{t3}\"OffboardCompID\" : {vehicle.OffboardCompID.ToString()},");
                        sb.Append($"{t3}\"OffboardSysID\" : {vehicle.OffboardSysID.ToString()},");
                        sb.Append($"{t3}\"QgcHostIp\" : \"{vehicle.QgcHostIp}\",");
                        sb.Append($"{t3}\"QgcPort\" : {vehicle.QgcPort.ToString()},");
                        sb.Append($"{t3}\"SerialBaudRate\" : {vehicle.SerialBaudRate.ToString()},");
                        sb.Append($"{t3}\"SerialPort\" : \"{vehicle.SerialPort}\",");
                        sb.Append($"{t3}\"SimCompID\" : {vehicle.SimCompID.ToString()},");
                        sb.Append($"{t3}\"SimSysID\" : {vehicle.SimSysID.ToString()},");
                        sb.Append($"{t3}\"SitlIp\" : \"{vehicle.SitlIp}\",");
                        sb.Append($"{t3}\"SitlPort\" : {vehicle.SitlPort.ToString()},");
                        sb.Append($"{t3}\"UdpIp\" : \"{vehicle.UdpIp}\",");
                        sb.Append($"{t3}\"UdpPort\" : {vehicle.UdpPort.ToString()},");
                        sb.Append($"{t3}\"UseSerial\" : {vehicle.UseSerial.ToString().ToLower()},");
                        sb.Append($"{t3}\"VehicleCompID\" : {vehicle.VehicleCompID.ToString()},");
                        sb.Append($"{t3}\"VehicleSysID\" : {vehicle.VehicleSysID.ToString()},");
                        sb.Append($"{t3}\"Model\" : \"{vehicle.Model}\",");
                        sb.Append($"{t3}\"LocalHostIp\" : \"{vehicle.LocalHostIp}\",");
                        sb.Append($"{t3}\"DefaultVehicleState\" : \"{vehicle.DefaultVehicleState}\",");
                        sb.Append($"{t3}\"AutoCreate\" : {vehicle.AutoCreate.ToString().ToLower()},");
                        sb.Append($"{t3}\"EnableTrace\" : {vehicle.EnableTrace.ToString().ToLower()},");
                        sb.Append($"{t3}\"PawnPath\" : \"{vehicle.PawnPath}\",");
                        sb.Append($"{t3}\"EnableCollisionPassthrogh\" : {vehicle.EnableCollisionPassthrogh.ToString().ToLower()},");
                        sb.Append($"{t3}\"EnableCollisions\" : {vehicle.EnableCollisions.ToString().ToLower()},");
                        sb.Append($"{t3}\"IsFpvVehicle\" : {vehicle.IsFpvVehicle.ToString().ToLower()},");
                        sb.Append($"{t3}\"AllowAPIAlways\" : {vehicle.AllowAPIAlways.ToString().ToLower()},");
                        sb.Append($"{t3}\"RC\" : {{");
                        sb.Append($"{t4}\"RemoteControlID\" : {vehicle.RC.RemoteControlID.ToString()},");
                        sb.Append($"{t4}\"AllowAPIWhenDisconnected\" : {vehicle.RC.AllowAPIWhenDisconnected.ToString().ToLower()}");
                        sb.Append($"{t3}}},"); // End RC
                        sb.Append($"{t3}\"Cameras\" : {{");
                        foreach (var cam in vehicle.Cameras)
                        {
                            sb.Append($"{t4}\"{cam.Name}\" : {{");
                            sb.Append($"{t5}\"CaptureSettings\" : [");
                            foreach (var capSet in cam.CaptureSettings)
                            {
                                sb.Append($"{t6}{{");
                                sb.Append($"{t6}\"ImageType\" : {capSet.ImageType.ToString()},");
                                sb.Append($"{t6}\"Width\" : {capSet.Width.ToString()},");
                                sb.Append($"{t6}\"Height\" : {capSet.Height.ToString()},");
                                sb.Append($"{t6}\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                                sb.Append($"{t6}\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                                sb.Append($"{t6}\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                                sb.Append($"{t6}\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                                sb.Append($"{t6}\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                                sb.Append($"{t6}\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                                sb.Append($"{t6}\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                                sb.Append($"{t6}\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                                sb.Append($"{t6}\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                                sb.Append($"{t6}}},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}],");
                            sb.Append($"{t5}\"NoiseSettings\" : [");
                            foreach (var noiseSet in cam.NoiseSettings)
                            {
                                sb.Append($"{t6}{{");
                                sb.Append($"{t6}\"Enabled\" : {noiseSet.Enabled.ToString().ToLower()},");
                                sb.Append($"{t6}\"ImageType\" : {noiseSet.ImageType.ToString()},");
                                sb.Append($"{t6}\"RandContrib\" : {noiseSet.RandContrib.ToString()},");
                                sb.Append($"{t6}\"RandSpeed\" : {noiseSet.RandSpeed.ToString()},");
                                sb.Append($"{t6}\"RandSize\" : {noiseSet.RandSize.ToString()},");
                                sb.Append($"{t6}\"RandDensity\" : {noiseSet.RandDensity.ToString()},");
                                sb.Append($"{t6}\"HorzWaveContrib\" : {noiseSet.HorzWaveContrib.ToString()},");
                                sb.Append($"{t6}\"HorzWaveStrength\" : {noiseSet.HorzWaveStrength.ToString()},");
                                sb.Append($"{t6}\"HorzWaveVertSize\" : {noiseSet.HorzWaveVertSize.ToString()},");
                                sb.Append($"{t6}\"HorzWaveScreenSize\" : {noiseSet.HorzWaveScreenSize.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesContrib\" : {noiseSet.HorzNoiseLinesContrib.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesDensityY\" : {noiseSet.HorzNoiseLinesDensityY.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesDensityXY\" : {noiseSet.HorzNoiseLinesDensityXY.ToString()},");
                                sb.Append($"{t6}\"HorzDistortionContrib\" : {noiseSet.HorzDistortionContrib.ToString()},");
                                sb.Append($"{t6}\"HorzDistortionStrength\" : {noiseSet.HorzDistortionStrength.ToString()}");
                                sb.Append($"{t6}}},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}],");
                            sb.Append($"{t5}\"Gimbal\" : {{");
                            sb.Append($"{t6}\"Stabilization\" : {cam.Gimbal.Stabilization.ToString()},");
                            if (!float.IsNaN(cam.Gimbal.Rotation.Pitch))
                            {
                                sb.Append($"{t6}\"Pitch\" : {cam.Gimbal.Rotation.Pitch.ToString()},");
                            }
                            if (!float.IsNaN(cam.Gimbal.Rotation.Roll))
                            {
                                sb.Append($"{t6}\"Roll\" : {cam.Gimbal.Rotation.Roll.ToString()},");
                            }
                            if (!float.IsNaN(cam.Gimbal.Rotation.Yaw))
                            {
                                sb.Append($"{t6}\"Yaw\" : {cam.Gimbal.Rotation.Yaw.ToString()},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}}},");  // End Gimabl
                            // Note: Camera position and rotation settings currently if omitted will cause Unreal to crash: Issue #1836
                            // Therefore we will at least write 0.0 for any unspecified (NaN) values
                            if (!float.IsNaN(cam.Position.X))
                            {
                                sb.Append($"{t5}\"X\" : {cam.Position.X.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"X\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Position.Y))
                            {
                                sb.Append($"{t5}\"Y\" : {cam.Position.Y.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Y\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Position.Z))
                            {
                                sb.Append($"{t5}\"Z\" : {cam.Position.Z.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Z\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Pitch))
                            {
                                sb.Append($"{t5}\"Pitch\" : {cam.Rotation.Pitch.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Pitch\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Roll))
                            {
                                sb.Append($"{t5}\"Roll\" : {cam.Rotation.Roll.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Roll\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Yaw))
                            {
                                sb.Append($"{t5}\"Yaw\" : {cam.Rotation.Yaw.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Yaw\" : 0.0,");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t4}}},"); // End this cam
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append($"{t3}}},"); // End Cameras
                        if (!float.IsNaN(vehicle.Position.X))
                        {
                            sb.Append($"{t3}\"X\" : {vehicle.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Y))
                        {
                            sb.Append($"{t3}\"Y\" : {vehicle.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Z))
                        {
                            sb.Append($"{t3}\"Z\" : {vehicle.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Pitch))
                        {
                            sb.Append($"{t3}\"Pitch\" : {vehicle.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Roll))
                        {
                            sb.Append($"{t3}\"Roll\" : {vehicle.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Yaw))
                        {
                            sb.Append($"{t3}\"Yaw\" : {vehicle.Rotation.Yaw.ToString()},");
                        }
                        sb.Append($"{t3}\"Sensors\" : {{");
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.BarometerList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Barometer)
                            {
                                sb.Append($"{t4}\"Barometer{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 1,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.ImuList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Imu)
                            {
                                sb.Append($"{t4}\"Imu{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 2,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.GpsList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Gps)
                            {
                                sb.Append($"{t4}\"Gps{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 3,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.MagnetometerList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Magnetometer)
                            {
                                sb.Append($"{t4}\"Magnetometer{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 4,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.DistanceList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Distance)
                            {
                                sb.Append($"{t4}\"Distance{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 5,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                                sb.Append($"{t4}}},");
                            }
                        }
                        i = 0;
                        foreach (var sensor in vehicle.Sensors.LidarList)
                        {
                            i++;
                            if (sensor.SensorType == SensorType.Lidar)
                            {
                                sb.Append($"{t4}\"Lidar{i}\" : {{");
                                sb.Append($"{t5}\"SensorType\" : 6,");
                                sb.Append($"{t5}\"Enabled\" : {sensor.Enabled.ToString().ToLower()},");
                                sb.Append($"{t5}\"NumberOfChannels\" : {sensor.NumberOfChannels.ToString()},");
                                sb.Append($"{t5}\"Range\" : {sensor.Range.ToString()},");
                                sb.Append($"{t5}\"RotationsPerSecond\" : {sensor.RotationsPerSecond.ToString()},");
                                sb.Append($"{t5}\"PointsPerSecond\" : {sensor.PointsPerSecond.ToString()},");
                                if (!float.IsNaN(sensor.Position.X))
                                {
                                    sb.Append($"{t5}\"X\" : {sensor.Position.X.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Y))
                                {
                                    sb.Append($"{t5}\"Y\" : {sensor.Position.Y.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Position.Z))
                                {
                                    sb.Append($"{t5}\"Z\" : {sensor.Position.Z.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Pitch))
                                {
                                    sb.Append($"{t5}\"Pitch\" : {sensor.Rotation.Pitch.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Roll))
                                {
                                    sb.Append($"{t5}\"Roll\" : {sensor.Rotation.Roll.ToString()},");
                                }
                                if (!float.IsNaN(sensor.Rotation.Yaw))
                                {
                                    sb.Append($"{t5}\"Yaw\" : {sensor.Rotation.Yaw.ToString()},");
                                }
                                sb.Append($"{t5}\"VerticalFOVUpper\" : {sensor.VerticalFOVUpper.ToString()},");
                                sb.Append($"{t5}\"VerticalFOVLower\" : {sensor.VerticalFOVLower.ToString()},");
                                sb.Append($"{t5}\"HorizontalFOVStart\" : {sensor.HorizontalFOVStart.ToString()},");
                                sb.Append($"{t5}\"HorizontalFOVEnd\" : {sensor.HorizontalFOVEnd.ToString()},");
                                sb.Append($"{t5}\"DrawDebugPoints\" : {sensor.DrawDebugPoints.ToString().ToLower()},");
                                sb.Append($"{t5}\"DataFrame\" : \"{sensor.DataFrame}\"");
                                sb.Append($"{t4}}},");
                            }
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append($"{t3}}}"); // End This PX4 Sensor
                        sb.Append($"{t2}}},"); // End This PX4
                    } // End if
                } // End foreach

                foreach (var vehicle in Vehicles.Vehicles_ComputerVision)
                {
                    if (vehicle.VehicleType == "ComputerVision")
                    {
                        sb.Append($"{t2}\"{vehicle.VehicleName}\" : {{");
                        sb.Append($"{t3}\"VehicleType\" : \"{vehicle.VehicleType}\",");
                        sb.Append($"{t3}\"DefaultVehicleState\" : \"{vehicle.DefaultVehicleState}\",");
                        sb.Append($"{t3}\"AutoCreate\" : {vehicle.AutoCreate.ToString().ToLower()},");
                        sb.Append($"{t3}\"EnableTrace\" : {vehicle.EnableTrace.ToString().ToLower()},");
                        sb.Append($"{t3}\"PawnPath\" : \"{vehicle.PawnPath}\",");
                        sb.Append($"{t3}\"EnableCollisionPassthrogh\" : {vehicle.EnableCollisionPassthrogh.ToString().ToLower()},");
                        sb.Append($"{t3}\"EnableCollisions\" : {vehicle.EnableCollisions.ToString().ToLower()},");
                        sb.Append($"{t3}\"IsFpvVehicle\" : {vehicle.IsFpvVehicle.ToString().ToLower()},");
                        sb.Append($"{t3}\"AllowAPIAlways\" : {vehicle.AllowAPIAlways.ToString().ToLower()},");
                        sb.Append($"{t3}\"RC\" : {{");
                        sb.Append($"{t4}\"RemoteControlID\" : {vehicle.RC.RemoteControlID.ToString()},");
                        sb.Append($"{t4}\"AllowAPIWhenDisconnected\" : {vehicle.RC.AllowAPIWhenDisconnected.ToString().ToLower()}");
                        sb.Append($"{t3}}},"); // End RC
                        sb.Append($"{t3}\"Cameras\" : {{");
                        foreach (var cam in vehicle.Cameras)
                        {
                            sb.Append($"{t4}\"{cam.Name}\" : {{");
                            sb.Append($"{t5}\"CaptureSettings\" : [");
                            foreach (var capSet in cam.CaptureSettings)
                            {
                                sb.Append($"{t6}{{");
                                sb.Append($"{t6}\"ImageType\" : {capSet.ImageType.ToString()},");
                                sb.Append($"{t6}\"Width\" : {capSet.Width.ToString()},");
                                sb.Append($"{t6}\"Height\" : {capSet.Height.ToString()},");
                                sb.Append($"{t6}\"FOV_Degrees\" : {capSet.FOV_Degrees.ToString()},");
                                sb.Append($"{t6}\"AutoExposureSpeed\" : {capSet.AutoExposureSpeed.ToString()},");
                                sb.Append($"{t6}\"AutoExposureBias\" : {capSet.AutoExposureBias.ToString()},");
                                sb.Append($"{t6}\"AutoExposureMaxBrightness\" : {capSet.AutoExposureMaxBrightness.ToString()},");
                                sb.Append($"{t6}\"AutoExposureMinBrightness\" : {capSet.AutoExposureMinBrightness.ToString()},");
                                sb.Append($"{t6}\"MotionBlurAmount\" : {capSet.MotionBlurAmount.ToString()},");
                                sb.Append($"{t6}\"TargetGamma\" : {capSet.TargetGamma.ToString()},");
                                sb.Append($"{t6}\"ProjectionMode\" : \"{capSet.ProjectionMode}\",");
                                sb.Append($"{t6}\"OrthoWidth\" : {capSet.OrthoWidth.ToString()}");
                                sb.Append($"{t6}}},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}],");
                            sb.Append($"{t5}\"NoiseSettings\" : [");
                            foreach (var noiseSet in cam.NoiseSettings)
                            {
                                sb.Append($"{t6}{{");
                                sb.Append($"{t6}\"Enabled\" : {noiseSet.Enabled.ToString().ToLower()},");
                                sb.Append($"{t6}\"ImageType\" : {noiseSet.ImageType.ToString()},");
                                sb.Append($"{t6}\"RandContrib\" : {noiseSet.RandContrib.ToString()},");
                                sb.Append($"{t6}\"RandSpeed\" : {noiseSet.RandSpeed.ToString()},");
                                sb.Append($"{t6}\"RandSize\" : {noiseSet.RandSize.ToString()},");
                                sb.Append($"{t6}\"RandDensity\" : {noiseSet.RandDensity.ToString()},");
                                sb.Append($"{t6}\"HorzWaveContrib\" : {noiseSet.HorzWaveContrib.ToString()},");
                                sb.Append($"{t6}\"HorzWaveStrength\" : {noiseSet.HorzWaveStrength.ToString()},");
                                sb.Append($"{t6}\"HorzWaveVertSize\" : {noiseSet.HorzWaveVertSize.ToString()},");
                                sb.Append($"{t6}\"HorzWaveScreenSize\" : {noiseSet.HorzWaveScreenSize.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesContrib\" : {noiseSet.HorzNoiseLinesContrib.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesDensityY\" : {noiseSet.HorzNoiseLinesDensityY.ToString()},");
                                sb.Append($"{t6}\"HorzNoiseLinesDensityXY\" : {noiseSet.HorzNoiseLinesDensityXY.ToString()},");
                                sb.Append($"{t6}\"HorzDistortionContrib\" : {noiseSet.HorzDistortionContrib.ToString()},");
                                sb.Append($"{t6}\"HorzDistortionStrength\" : {noiseSet.HorzDistortionStrength.ToString()}");
                                sb.Append($"{t6}}},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}],");
                            sb.Append($"{t5}\"Gimbal\" : {{");
                            sb.Append($"{t6}\"Stabilization\" : {cam.Gimbal.Stabilization.ToString()},");
                            if (!float.IsNaN(cam.Gimbal.Rotation.Pitch))
                            {
                                sb.Append($"{t6}\"Pitch\" : {cam.Gimbal.Rotation.Pitch.ToString()},");
                            }
                            if (!float.IsNaN(cam.Gimbal.Rotation.Roll))
                            {
                                sb.Append($"{t6}\"Roll\" : {cam.Gimbal.Rotation.Roll.ToString()},");
                            }
                            if (!float.IsNaN(cam.Gimbal.Rotation.Yaw))
                            {
                                sb.Append($"{t6}\"Yaw\" : {cam.Gimbal.Rotation.Yaw.ToString()},");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t5}}},");  // End Gimabl
                            // Note: Camera position and rotation settings currently if omitted will cause Unreal to crash: Issue #1836
                            // Therefore we will at least write 0.0 for any unspecified (NaN) values
                            if (!float.IsNaN(cam.Position.X))
                            {
                                sb.Append($"{t5}\"X\" : {cam.Position.X.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"X\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Position.Y))
                            {
                                sb.Append($"{t5}\"Y\" : {cam.Position.Y.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Y\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Position.Z))
                            {
                                sb.Append($"{t5}\"Z\" : {cam.Position.Z.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Z\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Pitch))
                            {
                                sb.Append($"{t5}\"Pitch\" : {cam.Rotation.Pitch.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Pitch\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Roll))
                            {
                                sb.Append($"{t5}\"Roll\" : {cam.Rotation.Roll.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Roll\" : 0.0,");
                            }
                            if (!float.IsNaN(cam.Rotation.Yaw))
                            {
                                sb.Append($"{t5}\"Yaw\" : {cam.Rotation.Yaw.ToString()},");
                            }
                            else
                            {
                                sb.Append($"{t5}\"Yaw\" : 0.0,");
                            }
                            sb.Remove(sb.Length - 1, 1);
                            sb.Append($"{t4}}},"); // End this cam
                        }
                        sb.Remove(sb.Length - 1, 1);
                        sb.Append($"{t3}}},"); // End Cameras
                        if (!float.IsNaN(vehicle.Position.X))
                        {
                            sb.Append($"{t3}\"X\" : {vehicle.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Y))
                        {
                            sb.Append($"{t3}\"Y\" : {vehicle.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Position.Z))
                        {
                            sb.Append($"{t3}\"Z\" : {vehicle.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Pitch))
                        {
                            sb.Append($"{t3}\"Pitch\" : {vehicle.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Roll))
                        {
                            sb.Append($"{t3}\"Roll\" : {vehicle.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(vehicle.Rotation.Yaw))
                        {
                            sb.Append($"{t3}\"Yaw\" : {vehicle.Rotation.Yaw.ToString()},");
                        }
                        sb.Append($"{t2}}},"); // End This ComputerVision
                    } // End if
                } // End foreach
                sb.Remove(sb.Length - 1, 1);
                sb.Append($"{t1}}},"); // End Vehicles

                sb.Append($"{t1}\"CameraDirector\" : {{");
                if (!float.IsNaN(CameraDirector.Position.X))
                {
                    sb.Append($"{t2}\"X\" : {CameraDirector.Position.X.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Position.Y))
                {
                    sb.Append($"{t2}\"Y\" : {CameraDirector.Position.Y.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Position.Z))
                {
                    sb.Append($"{t2}\"Z\" : {CameraDirector.Position.Z.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Rotation.Pitch))
                {
                    sb.Append($"{t2}\"Pitch\" : {CameraDirector.Rotation.Pitch.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Rotation.Roll))
                {
                    sb.Append($"{t2}\"Roll\" : {CameraDirector.Rotation.Roll.ToString()},");
                }
                if (!float.IsNaN(CameraDirector.Rotation.Yaw))
                {
                    sb.Append($"{t2}\"Yaw\" : {CameraDirector.Rotation.Yaw.ToString()},");
                }
                sb.Append($"{t2}\"FollowDistance\" : {CameraDirector.FollowDistance}");
                sb.Append($"{t1}}},"); // End CameraDirector

                sb.Append($"{t1}\"DefaultSensors\" : {{");
                i = 0;
                foreach ( var sensor in DefaultSensors.BarometerList)
                {
                    i++;
                    if ( sensor.SensorType == SensorType.Barometer)
                    {
                        sb.Append($"{t2}\"Barometer{i}\" : {{");
                        sb.Append($"{t3}\"SensorType\" : 1,");
                        sb.Append($"{t3}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append($"{t2}}},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.ImuList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Imu)
                    {
                        sb.Append($"{t2}\"Imu{i}\" : {{");
                        sb.Append($"{t3}\"SensorType\" : 2,");
                        sb.Append($"{t3}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append($"{t2}}},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.GpsList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Gps)
                    {
                        sb.Append($"{t2}\"Gps{i}\" : {{");
                        sb.Append($"{t3}\"SensorType\" : 3,");
                        sb.Append($"{t3}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append($"{t2}}},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.MagnetometerList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Magnetometer)
                    {
                        sb.Append($"{t2}\"Magnetometer{i}\" : {{");
                        sb.Append($"{t3}\"SensorType\" : 4,");
                        sb.Append($"{t3}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append($"{t2}}},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.DistanceList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Distance)
                    {
                        sb.Append($"{t2}\"Distance{i}\" : {{");
                        sb.Append($"{t3}\"SensorType\" : 5,");
                        sb.Append($"{t3}\"Enabled\" : {sensor.Enabled.ToString().ToLower()}");
                        sb.Append($"{t2}}},");
                    }
                }
                i = 0;
                foreach (var sensor in DefaultSensors.LidarList)
                {
                    i++;
                    if (sensor.SensorType == SensorType.Lidar)
                    {
                        sb.Append($"{t2}\"Lidar{i}\" : {{");
                        sb.Append($"{t3}\"SensorType\" : 6,");
                        sb.Append($"{t3}\"Enabled\" : {sensor.Enabled.ToString().ToLower()},");
                        sb.Append($"{t3}\"NumberOfChannels\" : {sensor.NumberOfChannels.ToString()},");
                        sb.Append($"{t3}\"Range\" : {sensor.Range.ToString()},");
                        sb.Append($"{t3}\"RotationsPerSecond\" : {sensor.RotationsPerSecond.ToString()},");
                        sb.Append($"{t3}\"PointsPerSecond\" : {sensor.PointsPerSecond.ToString()},");

                        if (!float.IsNaN(sensor.Position.X))
                        {
                            sb.Append($"{t3}\"X\" : {sensor.Position.X.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Position.Y))
                        {
                            sb.Append($"{t3}\"Y\" : {sensor.Position.Y.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Position.Z))
                        {
                            sb.Append($"{t3}\"Z\" : {sensor.Position.Z.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Rotation.Pitch))
                        {
                            sb.Append($"{t3}\"Pitch\" : {sensor.Rotation.Pitch.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Rotation.Roll))
                        {
                            sb.Append($"{t3}\"Roll\" : {sensor.Rotation.Roll.ToString()},");
                        }
                        if (!float.IsNaN(sensor.Rotation.Yaw))
                        {
                            sb.Append($"{t3}\"Yaw\" : {sensor.Rotation.Yaw.ToString()},");
                        }
                        sb.Append($"{t3}\"VerticalFOVUpper\" : {sensor.VerticalFOVUpper.ToString()},");
                        sb.Append($"{t3}\"VerticalFOVLower\" : {sensor.VerticalFOVLower.ToString()},");
                        sb.Append($"{t3}\"HorizontalFOVStart\" : {sensor.HorizontalFOVStart.ToString()},");
                        sb.Append($"{t3}\"HorizontalFOVEnd\" : {sensor.HorizontalFOVEnd.ToString()},");
                        sb.Append($"{t3}\"DrawDebugPoints\" : {sensor.DrawDebugPoints.ToString().ToLower()},");
                        sb.Append($"{t3}\"DataFrame\" : \"{sensor.DataFrame}\"");
                        sb.Append($"{t2}}},");
                    }
                }
                sb.Remove(sb.Length -1, 1);

                sb.Append($"{t1}}}");
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
                GetSettings().SettingsVersion = 1.2;
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


