// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_AirSimSettings_hpp
#define airsim_core_AirSimSettings_hpp

#include <string>
#include <vector>
#include <exception>
#include <functional>
#include "Settings.hpp"
#include "CommonStructs.hpp"
#include "common_utils/Utils.hpp"
#include "ImageCaptureBase.hpp"

namespace msr { namespace airlib {

struct AirSimSettings {
private:
    typedef common_utils::Utils Utils;
    typedef ImageCaptureBase::ImageType ImageType;

public: //types
    static constexpr int kSubwindowCount = 3; //must be >= 3 for now

    struct SubwindowSetting {
        int window_index;
        ImageType image_type;
        bool visible;
        int camera_id;

        SubwindowSetting(int window_index_val = 0, ImageType image_type_val = ImageType::Scene, bool visible_val = false, int camera_id_val = 0)
            : window_index(window_index_val), image_type(image_type_val), visible(visible_val), camera_id(camera_id_val)
        {
        }
    };

    struct RecordingSettings {
        bool record_on_move;
        float record_interval;
        std::vector<std::string> header_columns;

        std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests;

        RecordingSettings(bool record_on_move_val = false, float record_interval_val = 0.05f)
            : record_on_move(record_on_move_val), record_interval(record_interval_val)
        {
        }
    };

    struct CarMeshPaths {
        std::string skeletal = "/AirSim/VehicleAdv/Vehicle/Vehicle_SkelMesh.Vehicle_SkelMesh";
        std::string bp = "/AirSim/VehicleAdv/Vehicle/VehicleAnimationBlueprint";
        std::string slippery_mat = "/AirSim/VehicleAdv/PhysicsMaterials/Slippery.Slippery";
        std::string non_slippery_mat = "/AirSim/VehicleAdv/PhysicsMaterials/NonSlippery.NonSlippery";

    };

    struct VehicleSettings {
        std::string vehicle_name, firmware_name;
        int server_port;

        VehicleSettings(const std::string& vehicle_name_val = "", const std::string& firmware_name_val = "", int server_port_val = 41451)
            : vehicle_name(vehicle_name_val), firmware_name(firmware_name_val), server_port(server_port_val)
        {
        }

        void getRawSettings(Settings& settings) const
        {
            Settings::singleton().getChild(vehicle_name, settings);
        }
    };

    struct AdditionalCameraSetting {
        // Additional camera positions
        float x = 0.5f;
        float y = 0.0f;
        float z = 0.0f;
        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
    };

    struct GimbleSetting {
        float stabilization = 0;
        //bool is_world_frame = false;
        float pitch = Utils::nan<float>();
        float roll = Utils::nan<float>();
        float yaw = Utils::nan<float>();
    };

    struct CaptureSetting {
        //below settinsg are obtained by using Unreal console command (press ~):
        // ShowFlag.VisualizeHDR 1.
        //to replicate camera settings to SceneCapture2D
        //TODO: should we use UAirBlueprintLib::GetDisplayGamma()?
        typedef msr::airlib::Utils Utils;
        static constexpr float kSceneTargetGamma = 1.4f;

        int image_type = 0;

        unsigned int width = 256, height = 144; //960 X 540
        float fov_degrees = Utils::nan<float>(); //90.0f
        int auto_exposure_method = -1;   //histogram
        float auto_exposure_speed = Utils::nan<float>(); // 100.0f;
        float auto_exposure_bias = Utils::nan<float>(); // 0;
        float auto_exposure_max_brightness = Utils::nan<float>(); // 0.64f;
        float auto_exposure_min_brightness = Utils::nan<float>(); // 0.03f;
        float auto_exposure_low_percent = Utils::nan<float>(); // 80.0f;
        float auto_exposure_high_percent = Utils::nan<float>(); // 98.3f;
        float auto_exposure_histogram_log_min = Utils::nan<float>(); // -8;
        float auto_exposure_histogram_log_max = Utils::nan<float>(); // 4;
        float motion_blur_amount = Utils::nan<float>();
        float target_gamma = Utils::nan<float>(); //1.0f; //This would be reset to kSceneTargetGamma for scene as default
        int projection_mode = 0; // ECameraProjectionMode::Perspective
        float ortho_width = Utils::nan<float>();

        GimbleSetting gimble;
    };

    struct NoiseSetting {
        int ImageType = 0;

        bool Enabled = false;

        float RandContrib = 0.2f;
        float RandSpeed = 100000.0f;
        float RandSize = 500.0f;
        float RandDensity = 2.0f;

        float HorzWaveContrib = 0.03f;
        float HorzWaveStrength = 0.08f;
        float HorzWaveVertSize = 1.0f;
        float HorzWaveScreenSize = 1.0f;

        float HorzNoiseLinesContrib = 1.0f;
        float HorzNoiseLinesDensityY = 0.01f;
        float HorzNoiseLinesDensityXY = 0.5f;

        float HorzDistortionContrib = 1.0f; 
        float HorzDistortionStrength = 0.002f;

    };

    struct SegmentationSettings {
        enum class InitMethodType {
            None, CommonObjectsRandomIDs
        };

        enum class MeshNamingMethodType {
            OwnerName, StaticMeshName
        };

        InitMethodType init_method = InitMethodType::CommonObjectsRandomIDs;
        bool override_existing = false;
        MeshNamingMethodType mesh_naming_method = MeshNamingMethodType::OwnerName;
    };

    struct TimeOfDaySettings {
        bool enabled = false;
        std::string start_datetime = "";    //format: %Y-%m-%d %H:%M:%S
        bool is_start_datetime_dst = false;
        float celestial_clock_speed = 1;
        float update_interval_secs = 60;
    };

private: //fields
    float settings_version_actual;
    float settings_version_minimum = 1;

public: //fields
    std::string simmode_name;

    std::vector<SubwindowSetting> subwindow_settings;

    std::vector<AdditionalCameraSetting> additional_camera_settings;
    std::map<int, CaptureSetting> capture_settings;
    std::map<int, NoiseSetting>  noise_settings;

    RecordingSettings recording_settings;
    SegmentationSettings segmentation_settings;
    TimeOfDaySettings tod_settings;

    std::vector<std::string> warning_messages;

    bool is_record_ui_visible;
    int initial_view_mode;
    bool enable_rpc;
    std::string api_server_address;
    std::string default_vehicle_config;
    std::string physics_engine_name;
    std::string usage_scenario;
    bool enable_collision_passthrough;
    std::string clock_type;
    float clock_speed;
    bool engine_sound;
    bool log_messages_visible;
    HomeGeoPoint origin_geopoint;
    CarMeshPaths car_mesh_paths;

public: //methods
    static AirSimSettings& singleton() 
    {
        static AirSimSettings instance;
        return instance;
    }

    AirSimSettings()
    {
        clear();
    }

    //returns number of warnings
    unsigned int load(std::function<std::string(void)> simmode_getter)
    {
        //wipe out previous values
        clear();

        const Settings& settings = Settings::singleton();
        checkSettingsVersion(settings);
        loadCoreSimModeSettings(settings, simmode_getter);
        loadClockSettings(settings);
        loadSubWindowsSettings(settings);
        loadViewModeSettings(settings);
        loadRecordingSettings(settings);
        loadAdditionalCameraSettings(settings);
        loadCaptureSettings(settings);
        loadCameraNoiseSettings(settings);
        loadSegmentationSettings(settings);
        loadOtherSettings(settings);

        return static_cast<unsigned int>(warning_messages.size());
    }

    void clear()
    {
        warning_messages.clear();

        initializeSubwindowSettings();
        initializeImageTypeSettings();
        segmentation_settings = SegmentationSettings();
        noise_settings.clear();
        capture_settings.clear();

        simmode_name = "";
        recording_settings = RecordingSettings();
        is_record_ui_visible = false;
        initial_view_mode = 3; //ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME
        enable_rpc = false;
        api_server_address = "";
        default_vehicle_config = "";
        physics_engine_name = "";
        usage_scenario = "";
        enable_collision_passthrough = false;
        clock_type = "";
        clock_speed = 1.0f;
        engine_sound = true;     
        log_messages_visible = true;
        //0,0,0 in Unreal is mapped to this GPS coordinates
        origin_geopoint = HomeGeoPoint(GeoPoint(47.641468, -122.140165, 122)); 
    }

    VehicleSettings getVehicleSettings(const std::string& vehicle_name)
    {
        Settings& settings = Settings::singleton();
        Settings vehicle_config_settings;
        settings.getChild(vehicle_name, vehicle_config_settings);

        std::string firmware_name = vehicle_config_settings.getString("FirmwareName", vehicle_name);
        int server_port = vehicle_config_settings.getInt("ApiServerPort", 41451);

        return VehicleSettings(vehicle_name, firmware_name, server_port);
    }

    static void initializeSettings(const std::string& json_settings_text)
    {
        Settings& settings = Settings::loadJSonString(json_settings_text);
        if (! settings.isLoadSuccess())
            throw std::invalid_argument("Cannot parse JSON settings string.");
    }

    static void createDefaultSettingsFile()
    {
        std::string settings_filename = Settings::getUserDirectoryFullPath("settings.json");
        Settings& settings = Settings::loadJSonString("{}");
        //write some settings in new file otherwise the string "null" is written if all settigs are empty
        settings.setString("SeeDocsAt", "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md");
        settings.setDouble("SettingsVersion", 1.0);

        //TODO: there is a crash in Linux due to settings.saveJSonString(). Remove this workaround after we only support Unreal 4.17
        //https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html
        settings.saveJSonFile(settings_filename);
    }

private:
    void loadCoreSimModeSettings(const Settings& settings, std::function<std::string(void)> simmode_getter)
    {
        simmode_name = settings.getString("SimMode", "");
        if (simmode_name == "") {
            if (simmode_getter)
                simmode_name = simmode_getter();
            else
                throw std::invalid_argument("simmode_name is not expected empty in SimModeBase");
        }

        usage_scenario = settings.getString("UsageScenario", "");
        default_vehicle_config = settings.getString("DefaultVehicleConfig", "");
        if (default_vehicle_config == "") {
            if (simmode_name == "Multirotor")
                default_vehicle_config = "SimpleFlight";
            else if (simmode_name == "Car")
                default_vehicle_config = "PhysXCar";
            else       
                warning_messages.push_back("SimMode is not valid: " + simmode_name);
        }

        physics_engine_name = settings.getString("PhysicsEngineName", "");
        if (physics_engine_name == "") {
            if (simmode_name == "Multirotor")
                physics_engine_name = "FastPhysicsEngine";
            else
                physics_engine_name = "PhysX";
        }
    }

    void checkSettingsVersion(const Settings& settings)
    {
        //we had spelling mistake so we are currently supporting SettingsVersion or SettingdVersion :(
        settings_version_actual = settings.getFloat("SettingsVersion", settings.getFloat("SettingdVersion", 0));

        if (settings_version_actual < settings_version_minimum) {
            if ((settings.size() == 1 && 
                ((settings.getString("SeeDocsAt", "") != "") || settings.getString("see_docs_at", "") != ""))
                || (settings.size() == 0)) {
                //no warnings because we have default settings
            }
            else {
                warning_messages.push_back("Your settings file does not have SettingsVersion element. This probably means you have old format settings file.");
                warning_messages.push_back("Please look at new settings and update your settings.json: https://git.io/v9mYY");
            }
        }
    }

    void loadViewModeSettings(const Settings& settings)
    {
        std::string view_mode_string = settings.getString("ViewMode", "");

        if (view_mode_string == "") {
            if (usage_scenario == "") {
                if (simmode_name == "Multirotor")
                    view_mode_string = "FlyWithMe";
                else
                    view_mode_string = "SpringArmChase";
            }
            else
                view_mode_string = "SpringArmChase";
        }

        if (view_mode_string == "Fpv")
            initial_view_mode = 1; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV;
        else if (view_mode_string == "GroundObserver")
            initial_view_mode = 2; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER;
        else if (view_mode_string == "FlyWithMe")
            initial_view_mode = 3; //ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME;
        else if (view_mode_string == "Manual")
            initial_view_mode = 4; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL;
        else if (view_mode_string == "SpringArmChase")
            initial_view_mode = 5; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE;
        else if (view_mode_string == "Backup")
            initial_view_mode = 6; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_BACKUP;
        else if (view_mode_string == "NoDisplay")
            initial_view_mode = 7; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_NODISPLAY;
        else
            warning_messages.push_back("ViewMode setting is not recognized: " + view_mode_string);
    }

    void loadRecordingSettings(const Settings& settings)
    {
        Settings recording_json;
        if (settings.getChild("Recording", recording_json)) {
            recording_settings.record_on_move = recording_json.getBool("RecordOnMove", recording_settings.record_on_move);
            recording_settings.record_interval = recording_json.getFloat("RecordInterval", recording_settings.record_interval);

            Settings req_cameras_settings;
            if (recording_json.getChild("Cameras", req_cameras_settings)) {
                for (size_t child_index = 0; child_index < req_cameras_settings.size(); ++child_index) {
                    Settings req_camera_settings;
                    if (req_cameras_settings.getChild(child_index, req_camera_settings)) {
                        uint8_t camera_id = static_cast<uint8_t>(req_camera_settings.getInt("CameraID", 0));
                        ImageType image_type =
                            Utils::toEnum<ImageType>(
                                req_camera_settings.getInt("ImageType", 0));
                        bool compress = req_camera_settings.getBool("Compress", true);
                        bool pixels_as_float = req_camera_settings.getBool("PixelsAsFloat", false);

                        recording_settings.requests.push_back(msr::airlib::ImageCaptureBase::ImageRequest(
                            camera_id, image_type, pixels_as_float, compress));
                    }
                }
            }
        }
        if (recording_settings.requests.size() == 0)
            recording_settings.requests.push_back(msr::airlib::ImageCaptureBase::ImageRequest(
                0, ImageType::Scene, false, true));

        if (simmode_name == "Multirotor") {
            recording_settings.header_columns = std::vector<std::string> {
                "Timestamp", "Position(x)", "Position(y)", "Position(z)", "Orientation(w)",
                "Orientation(x)", "Orientation(y)", "Orientation(z)", "ImageName"
            };
        }
        else if (simmode_name == "Car") {
            recording_settings.header_columns = std::vector<std::string> {
                "Timestamp", "Speed (kmph)", "Throttle" , "Steering", "Brake", "Gear", "ImageName"
            };
        }
        else 
            warning_messages.push_back("SimMode is not valid: " + simmode_name);
    }

    void loadCaptureSettings(const Settings& settings)
    {
        Settings json_parent;
        if (settings.getChild("CaptureSettings", json_parent)) {
            for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                Settings json_settings_child;     
                if (json_parent.getChild(child_index, json_settings_child)) {
                    CaptureSetting capture_setting;
                    createCaptureSettings(json_settings_child, capture_setting);
                    capture_settings[capture_setting.image_type] = capture_setting;
                }
            }
        }
    }

    void loadSegmentationSettings(const Settings& settings)
    {
        Settings json_parent;
        if (settings.getChild("SegmentationSettings", json_parent)) {
            std::string init_method = Utils::toLower(json_parent.getString("InitMethod", ""));
            if (init_method == "" || init_method == "commonobjectsrandomids")
                segmentation_settings.init_method = SegmentationSettings::InitMethodType::CommonObjectsRandomIDs;
            else if (init_method == "none")
                segmentation_settings.init_method = SegmentationSettings::InitMethodType::None;
            else
                //TODO: below exception doesn't actually get raised right now because of issue in Unreal Engine?
                throw std::invalid_argument(std::string("SegmentationSettings init_method has invalid value in settings ") + init_method);

            segmentation_settings.override_existing = json_parent.getBool("OverrideExisting", false);

            std::string mesh_naming_method = Utils::toLower(json_parent.getString("MeshNamingMethod", ""));
            if (mesh_naming_method == "" || mesh_naming_method == "ownername")
                segmentation_settings.mesh_naming_method = SegmentationSettings::MeshNamingMethodType::OwnerName;
            else if (mesh_naming_method == "staticmeshname")
                segmentation_settings.mesh_naming_method = SegmentationSettings::MeshNamingMethodType::StaticMeshName;
            else
                throw std::invalid_argument(std::string("SegmentationSettings MeshNamingMethod has invalid value in settings ") + mesh_naming_method);
        }
    }

    void loadCameraNoiseSettings(const Settings& settings)
    {
        Settings json_parent;
        if (settings.getChild("NoiseSettings", json_parent)) {
            for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                Settings json_settings_child;     
                if (json_parent.getChild(child_index, json_settings_child)) {
                    NoiseSetting noise_setting;
                    createNoiseSettings(json_settings_child, noise_setting);
                    noise_settings[noise_setting.ImageType] = noise_setting;
                }
            }
        }
    }

    void createNoiseSettings(const msr::airlib::Settings& settings, NoiseSetting& noise_setting)
    {
        noise_setting.Enabled = settings.getBool("Enabled", noise_setting.Enabled);
        noise_setting.ImageType = settings.getInt("ImageType", noise_setting.ImageType);

        noise_setting.HorzWaveStrength = settings.getFloat("HorzWaveStrength", noise_setting.HorzWaveStrength);
        noise_setting.RandSpeed = settings.getFloat("RandSpeed", noise_setting.RandSpeed);
        noise_setting.RandSize = settings.getFloat("RandSize", noise_setting.RandSize);
        noise_setting.RandDensity = settings.getFloat("RandDensity", noise_setting.RandDensity);
        noise_setting.RandContrib = settings.getFloat("RandContrib", noise_setting.RandContrib);
        noise_setting.HorzWaveContrib = settings.getFloat("HorzWaveContrib", noise_setting.HorzWaveContrib);
        noise_setting.HorzWaveVertSize = settings.getFloat("HorzWaveVertSize", noise_setting.HorzWaveVertSize);
        noise_setting.HorzWaveScreenSize = settings.getFloat("HorzWaveScreenSize", noise_setting.HorzWaveScreenSize);
        noise_setting.HorzNoiseLinesContrib = settings.getFloat("HorzNoiseLinesContrib", noise_setting.HorzNoiseLinesContrib);
        noise_setting.HorzNoiseLinesDensityY = settings.getFloat("HorzNoiseLinesDensityY", noise_setting.HorzNoiseLinesDensityY);
        noise_setting.HorzNoiseLinesDensityXY = settings.getFloat("HorzNoiseLinesDensityXY", noise_setting.HorzNoiseLinesDensityXY);
        noise_setting.HorzDistortionStrength = settings.getFloat("HorzDistortionStrength", noise_setting.HorzDistortionStrength);
        noise_setting.HorzDistortionContrib = settings.getFloat("HorzDistortionContrib", noise_setting.HorzDistortionContrib);
    }

    void loadAdditionalCameraSettings(const Settings& settings)
    {
        Settings json_parent;
        if (settings.getChild("AdditionalCameras", json_parent)) {
            for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                Settings additional_camera_setting;
                if (json_parent.getChild(child_index, additional_camera_setting)) {
                    AdditionalCameraSetting setting;
                    setting.x = additional_camera_setting.getFloat("X", setting.x);
                    setting.y = additional_camera_setting.getFloat("Y", setting.y);
                    setting.z = additional_camera_setting.getFloat("Z", setting.z);
                    setting.yaw = additional_camera_setting.getFloat("Yaw", setting.yaw);
                    setting.pitch = additional_camera_setting.getFloat("Pitch", setting.pitch);
                    setting.roll = additional_camera_setting.getFloat("Roll", setting.roll);
                    additional_camera_settings.push_back(setting);
                }
            }
        }
    }

    void createCaptureSettings(const msr::airlib::Settings& settings, CaptureSetting& capture_setting)
    {
        capture_setting.width = settings.getInt("Width", capture_setting.width);
        capture_setting.height = settings.getInt("Height", capture_setting.height);
        capture_setting.fov_degrees = settings.getFloat("FOV_Degrees", capture_setting.fov_degrees);
        capture_setting.auto_exposure_speed = settings.getFloat("AutoExposureSpeed", capture_setting.auto_exposure_speed);
        capture_setting.auto_exposure_bias = settings.getFloat("AutoExposureBias", capture_setting.auto_exposure_bias);
        capture_setting.auto_exposure_max_brightness = settings.getFloat("AutoExposureMaxBrightness", capture_setting.auto_exposure_max_brightness);
        capture_setting.auto_exposure_min_brightness = settings.getFloat("AutoExposureMinBrightness", capture_setting.auto_exposure_min_brightness);
        capture_setting.motion_blur_amount = settings.getFloat("MotionBlurAmount", capture_setting.motion_blur_amount);
        capture_setting.image_type = settings.getInt("ImageType", 0);
        capture_setting.target_gamma = settings.getFloat("TargetGamma", 
            capture_setting.image_type == 0 ? CaptureSetting::kSceneTargetGamma : Utils::nan<float>());

        std::string projection_mode = Utils::toLower(settings.getString("ProjectionMode", ""));
        if (projection_mode == "" || projection_mode == "perspective")
            capture_setting.projection_mode = 0; // Perspective
        else if (projection_mode == "orthographic")
            capture_setting.projection_mode = 1; // Orthographic
        else
            throw std::invalid_argument(std::string("CaptureSettings projection_mode has invalid value in settings ") + projection_mode);

        capture_setting.ortho_width = settings.getFloat("OrthoWidth", capture_setting.ortho_width);

        Settings json_parent;
        if (settings.getChild("Gimble", json_parent)) {
            //capture_setting.gimble.is_world_frame = json_parent.getBool("IsWorldFrame", false);
            capture_setting.gimble.stabilization = json_parent.getFloat("Stabilization", false);
            capture_setting.gimble.pitch = json_parent.getFloat("Pitch", Utils::nan<float>());
            capture_setting.gimble.roll = json_parent.getFloat("Roll", Utils::nan<float>());
            capture_setting.gimble.yaw = json_parent.getFloat("Yaw", Utils::nan<float>());
        }
    }

    void loadSubWindowsSettings(const Settings& settings)
    {
        //load default subwindows
        initializeSubwindowSettings();

        Settings json_parent;
        if (settings.getChild("SubWindows", json_parent)) {
            for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                Settings json_settings_child;
                if (json_parent.getChild(child_index, json_settings_child)) {
                    int window_index = json_settings_child.getInt("WindowID", 0);
                    SubwindowSetting& subwindow_setting = subwindow_settings.at(window_index);
                    subwindow_setting.window_index = window_index;
                    subwindow_setting.image_type = Utils::toEnum<ImageType>(
                        json_settings_child.getInt("ImageType", 0));
                    subwindow_setting.visible = json_settings_child.getBool("Visible", false);
                    subwindow_setting.camera_id = json_settings_child.getInt("CameraID", 0);
                }
            }
        }
    }

    void initializeImageTypeSettings()
    {
        capture_settings.clear();

        int image_count = Utils::toNumeric(ImageType::Count);

        for (int i = -1; i < image_count; ++i) {
            capture_settings[i] = CaptureSetting();
            noise_settings[i] = NoiseSetting();
        }

        capture_settings.at(Utils::toNumeric(ImageType::Scene)).target_gamma = CaptureSetting::kSceneTargetGamma;
    }

    void initializeSubwindowSettings()
    {
        subwindow_settings.clear();
        subwindow_settings.push_back(SubwindowSetting(0, ImageType::DepthVis, false, 0)); //depth
        subwindow_settings.push_back(SubwindowSetting(0, ImageType::Segmentation, false, 0)); //seg
        subwindow_settings.push_back(SubwindowSetting(0, ImageType::Scene, false, 0)); //vis
    }

    void loadOtherSettings(const Settings& settings)
    {
        enable_rpc = settings.getBool("RpcEnabled", true);
        //by default we spawn server at local endpoint. Do not use 127.0.0.1 as default below
        //because for docker container default is 0.0.0.0 and people get really confused why things
        //don't work
        api_server_address = settings.getString("LocalHostIp", "");
        is_record_ui_visible = settings.getBool("RecordUIVisible", true);
        engine_sound = settings.getBool("EngineSound", false);

        enable_collision_passthrough = settings.getBool("EnableCollisionPassthrogh", false);
        log_messages_visible = settings.getBool("LogMessagesVisible", true);

        {   //load origin geopoint
            Settings origin_geopoint_json;
            if (settings.getChild("OriginGeopoint", origin_geopoint_json)) {
                GeoPoint origin = origin_geopoint.home_point;
                origin.latitude = origin_geopoint_json.getDouble("Latitude", origin.latitude);
                origin.longitude = origin_geopoint_json.getDouble("Longitude", origin.longitude);
                origin.altitude = origin_geopoint_json.getFloat("Altitude", origin.altitude);
                origin_geopoint.initialize(origin);
            }
        }

        {   //time of day settings
            Settings tod_settings_json;
            if (settings.getChild("TimeOfDay", tod_settings_json)) {
                tod_settings.enabled = tod_settings_json.getBool("Enabled", tod_settings.enabled);
                tod_settings.start_datetime = tod_settings_json.getString("StartDateTime", tod_settings.start_datetime);
                tod_settings.celestial_clock_speed = tod_settings_json.getFloat("CelestialClockSpeed", tod_settings.celestial_clock_speed);
                tod_settings.is_start_datetime_dst = tod_settings_json.getBool("StartDateTimeDst", tod_settings.is_start_datetime_dst);
                tod_settings.update_interval_secs = tod_settings_json.getFloat("UpdateIntervalSecs", tod_settings.update_interval_secs);
            }
        }
    }

    void loadClockSettings(const Settings& settings)
    {
        clock_type = settings.getString("ClockType", "");

        if (clock_type == "") {
            if (default_vehicle_config == "SimpleFlight")
                clock_type = "SteppableClock";
            else
                clock_type = "ScalableClock";
        }

        clock_speed = settings.getFloat("ClockSpeed", 1.0f);
    }
};

}} //namespace
#endif
