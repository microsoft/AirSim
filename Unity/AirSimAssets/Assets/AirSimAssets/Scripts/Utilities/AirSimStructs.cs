using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Text;

/*
 * Structs used in the simulation are defined here.
 * Majority of them are the mapping from AirLib
 */

namespace AirSimUnity {

    namespace CarStructs {

        [StructLayout(LayoutKind.Sequential)]
        public struct CarControls {
            public float throttle; /* 1 to -1 */
            public float steering; /* 1 to -1 */
            public float brake;    /* 1 to -1 */

            [MarshalAs(UnmanagedType.U1)]
            public bool handbrake;

            [MarshalAs(UnmanagedType.U1)]
            public bool is_manual_gear;

            public int manual_gear;

            [MarshalAs(UnmanagedType.U1)]
            public bool gear_immediate;

            public CarControls(float throttle, float steering, float brake, bool handbrake, bool isManualGear, int gear, bool isGearImmediate) {
                this.throttle = throttle;
                this.steering = steering;
                this.brake = brake;
                this.handbrake = handbrake;
                this.is_manual_gear = isManualGear;
                this.manual_gear = gear;
                this.gear_immediate = isGearImmediate;
            }

            public void Reset()
            {
                throttle = 0;
                steering = 0;
                brake = 0;
                handbrake = false;
                is_manual_gear = false;
                manual_gear = 0;
                gear_immediate = false;
            }
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct CarState {
            public int gear;
            public float speed;
            public long timeStamp;
            public float engineRotationSpeed;
            public float engineMaxRotationSpeed;
            public AirSimPose pose;

            public void Reset()
            {
                gear = 0;
                speed = 0;
                timeStamp = 0;
                engineRotationSpeed = 0;
                engineMaxRotationSpeed = 0;
                pose.Reset();
            }
        }

        public struct CarData {
            public int speed;
            public int gear;
            public float throttle;
            public float brake;
            public float steering;
            public float engineRotationSpeed;
            public float engineMaxRotationSpeed;

            public void Reset()
            {
                speed = 0;
                gear = 0;
                throttle = 0f;
                brake = 0f;
                steering = 0f;
                engineRotationSpeed = 0;
                engineMaxRotationSpeed = 0;
            }
        }
    }

    namespace DroneStructs {

        [StructLayout(LayoutKind.Sequential)]
        internal struct MultirotorState {
            private CollisionInfo collision;
            private KinemticState kinematics_estimated;
            private KinemticState kinematics_true;
            private GeoPoint gps_location;
            private long timestamp;

            public MultirotorState(CollisionInfo collision, KinemticState kinemtic_estimate, KinemticState kinematic_true, GeoPoint point, long time) {
                this.collision = collision;
                this.kinematics_estimated = kinemtic_estimate;
                this.kinematics_true = kinematic_true;
                this.gps_location = point;
                this.timestamp = time;
            }
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct AirSimRCData {
            public long timestamp;
            public float pitch, roll, throttle, yaw;
            public float left_z, right_z;
            public uint switch1, switch2, switch3, switch4,
                switch5, switch6, switch7, switch8;

            [MarshalAs(UnmanagedType.U1)]
            public bool is_initialized;

            [MarshalAs(UnmanagedType.U1)]
            public bool is_valid;

            public void Reset()
            {
                timestamp = 0;
                pitch =0;
                roll=0;
                throttle=0;
                yaw = 0;
                left_z =0;
                right_z =0;
                switch1 =0;
                switch2 =0;
                switch3 = 0;
                switch4=0;
                switch5 = 0;
                switch6 = 0;
                switch7 = 0;
                switch8 = 0;
            }
        }

        public struct RotorInfo {
            public float rotorSpeed;
            public int rotorDirection;
            public float rotorThrust;
            public float rotorControlFiltered;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct KinemticState {
        public AirSimPose pose;
        public Twist twist;
        public Accelerations accelerations;

        public KinemticState(AirSimPose pose, Twist twist, Accelerations accelerations) {
            this.pose = pose;
            this.twist = twist;
            this.accelerations = accelerations;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct AirSimPose {
        public AirSimVector position;
        public AirSimQuaternion orientation;

        public AirSimPose(AirSimVector position, AirSimQuaternion orientation) {
            this.position = position;
            this.orientation = orientation;
        }

        public void Reset()
        {
            position.Reset();
            orientation.Reset();
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Twist {
        public AirSimVector linear;
        public AirSimVector angular;

        public Twist(AirSimVector linear, AirSimVector angular) {
            this.linear = linear;
            this.angular = angular;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct AirSimVector
    {
        public float x;
        public float y;
        public float z;

        public AirSimVector(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public void Set(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public void Reset()
        {
            x = y = z = 0;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct AirSimQuaternion {
        public float x;
        public float y;
        public float z;
        public float w;

        public AirSimQuaternion(float x, float y, float z, float w) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public void Set(float x, float y, float z, float w) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }


        public void Reset()
        {
            x = y = z = w = 0;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GeoPoint {
        private double lattitude;
        private double longitude;
        private float altitude;

        public GeoPoint(float lattitude, float longitude, float altitude) {
            this.lattitude = lattitude;
            this.longitude = longitude;
            this.altitude = altitude;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HomeGeoPoint {
        private GeoPoint home_point;
        private double lat_rad, lon_rad;
        private double cos_lat, sin_lat;

        public HomeGeoPoint(GeoPoint home_point, double lat_rad, double lon_rad, double cos_lat, double sin_lat) {
            this.home_point = home_point;
            this.lat_rad = lat_rad;
            this.lon_rad = lon_rad;
            this.cos_lat = cos_lat;
            this.sin_lat = sin_lat;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GeoPose {
        private AirSimQuaternion orientation;
        private GeoPoint position;

        public GeoPose(AirSimQuaternion orientation, GeoPoint position) {
            this.orientation = orientation;
            this.position = position;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Accelerations {
        public AirSimVector linear;
        public AirSimVector angular;

        public Accelerations(AirSimVector linear, AirSimVector angular) {
            this.linear = linear;
            this.angular = angular;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct CollisionInfo {

        [MarshalAs(UnmanagedType.U1)]
        public bool has_collided;

        public AirSimVector normal;
        public AirSimVector impact_point;
        public AirSimVector position;
        public float penetration_depth;
        public long time_stamp;
        public int collision_count;
        public string object_name;
        public int object_id;

        public void SetDefaultValues() {
            has_collided = false;
            normal = new AirSimVector(0, 0, 0);
            impact_point = new AirSimVector(0, 0, 0);
            position = new AirSimVector(0, 0, 0);
            penetration_depth = 0;
            time_stamp = 0;
            collision_count = 0;
            object_name = "none";
            object_id = -1;
        }
    };

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    public struct WrapperConfig {

        [MarshalAs(UnmanagedType.U1)]
        public bool is_fpv_vehicle;

        public string vehicle_config_name;

        [MarshalAs(UnmanagedType.U1)]
        public bool enable_collision;

        [MarshalAs(UnmanagedType.U1)]
        public bool enable_passthrough_on_collisions;

        [MarshalAs(UnmanagedType.U1)]
        public bool enable_trace;

        public WrapperConfig(bool is_fpv_vehicle, string vehicle_config_name, bool enable_collision, bool enable_passthrough_on_collisions, bool enable_trace) {
            this.is_fpv_vehicle = is_fpv_vehicle;
            this.vehicle_config_name = vehicle_config_name;
            this.enable_collision = enable_collision;
            this.enable_passthrough_on_collisions = enable_passthrough_on_collisions;
            this.enable_trace = enable_trace;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct CameraInfo {
        public AirSimPose pose;
        public float fov;

        public CameraInfo(AirSimPose pose, float fov) {
            this.pose = pose;
            this.fov = fov;
        }
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct ImageRequest {
        public string camera_name;
        public ImageType image_type;

        [MarshalAs(UnmanagedType.U1)]
        public bool pixels_as_float;

        [MarshalAs(UnmanagedType.U1)]
        public bool compress;

        public ImageRequest(string camera_name, ImageType image_type, bool pixels_as_float, bool compress) {
            this.camera_name = camera_name;
            this.image_type = image_type;
            this.pixels_as_float = pixels_as_float;
            this.compress = compress;
        }
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct ImageResponse
    {
        public int image_uint_len;
        public byte[] image_data_uint;
        public int image_float_len;
        public float[] image_data_float;
        public AirSimVector camera_position;
        public AirSimQuaternion camera_orientation;

        [MarshalAs(UnmanagedType.U1)]
        public bool pixels_as_float;

        [MarshalAs(UnmanagedType.U1)]
        public bool compress;

        public int width;
        public int height;
        public ImageType image_type;

        public ImageResponse(List<byte> imageDataInt, List<float> imageDataFloat, string cameraName,
            AirSimVector camera_position, AirSimQuaternion camera_orientation,
            bool pixels_as_float, bool compress, int width, int height, ImageType image_type) {
        
            if (imageDataInt == null) {
                image_uint_len = 0;
            } else {
                image_uint_len = imageDataInt.Count;
            }

            image_data_uint = new byte[image_uint_len];
            if (image_uint_len > 0) {
                imageDataInt.CopyTo(image_data_uint);
            }

            if (imageDataFloat == null) {
                image_float_len = 0;
            } else {
                image_float_len = imageDataFloat.Count;
            }

            image_data_float = new float[image_float_len];

            if (image_float_len > 0) {
                imageDataFloat.CopyTo(image_data_float);
            }

            this.camera_position = camera_position;
            this.camera_orientation = camera_orientation;
            this.pixels_as_float = pixels_as_float;
            this.compress = compress;
            this.width = width;
            this.height = height;
            this.image_type = image_type;
        }

        public void reset() {
            image_uint_len = 0;
            image_float_len = 0;
            image_data_uint = new byte[] { };
            image_data_float = new float[] { };
            this.camera_position = new AirSimVector();
            this.camera_orientation = new AirSimQuaternion();
            this.pixels_as_float = false;
            this.compress = false;
            this.width = 0;
            this.height = 0;
            this.image_type = ImageType.Scene;
        }
    }

    public enum ImageType {
        Scene = 0,
        DepthPlanner,
        DepthPerspective,
        DepthVis,
        DisparityNormalized,
        Segmentation,
        SurfaceNormals,
        Infrared,
        Count
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct UnityTransform
    {
        /** Rotation of this transformation, as a quaternion */
        public AirSimQuaternion Rotation;
        /** Translation of this transformation, as a vector */
        public AirSimVector Position;
        /** 3D scale (always applied in local space) as a vector */
        public AirSimVector Scale3D;

        public UnityTransform(AirSimQuaternion rotation, AirSimVector position, AirSimVector scale3d)
        {
            Rotation = rotation;
            Position = position;
            Scale3D = scale3d;
        }

        public void DefaultInitialization()
        {
            Rotation = new AirSimQuaternion(0, 0, 0, 1);
            Position = new AirSimVector(0, 0, 0);
            Scale3D = new AirSimVector(1, 1, 1);
        }

    };

    [StructLayout(LayoutKind.Sequential)]
    public struct RayCastHitResult
    {
        public bool isHit;
        public float distance;

        public RayCastHitResult(bool isHit, float distance)
        {
            this.isHit = isHit;
            this.distance = distance;
        }
    };
}