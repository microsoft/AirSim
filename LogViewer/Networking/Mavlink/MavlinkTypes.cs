using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace Microsoft.Networking.Mavlink
{
    public partial class MAVLink
    {
        public const string MAVLINK_BUILD_DATE = "Fri Jun 17 15:03:03 2016";
        public const string MAVLINK_WIRE_PROTOCOL_VERSION = "1.0";
        public const int MAVLINK_MAX_DIALECT_PAYLOAD_SIZE = 255;

        public const int MAVLINK_LITTLE_ENDIAN = 1;
        public const int MAVLINK_BIG_ENDIAN = 0;

        public const byte MAVLINK_STX = 254;

        public const byte MAVLINK_ENDIAN = MAVLINK_LITTLE_ENDIAN;

        public const bool MAVLINK_ALIGNED_FIELDS = (1 == 1);

        public const byte MAVLINK_CRC_EXTRA = 1;

        public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);

        public static readonly byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] { 9, 31, 12, 0, 14, 28, 3, 32, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 30, 101, 22, 26, 16, 14, 28, 32, 28, 28, 22, 22, 21, 6, 6, 37, 4, 4, 2, 2, 4, 2, 2, 3, 13, 12, 37, 4, 0, 0, 27, 25, 0, 0, 0, 0, 0, 68, 26, 185, 229, 42, 6, 4, 0, 11, 18, 0, 0, 37, 20, 35, 33, 3, 0, 0, 0, 22, 39, 37, 53, 51, 53, 51, 0, 28, 56, 42, 33, 81, 0, 0, 0, 0, 0, 0, 26, 32, 32, 20, 32, 62, 44, 64, 84, 9, 254, 16, 12, 36, 44, 64, 22, 6, 14, 12, 97, 2, 2, 113, 35, 6, 79, 35, 35, 22, 13, 255, 14, 18, 43, 8, 22, 14, 36, 43, 41, 32, 243, 14, 93, 0, 100, 36, 60, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 40, 63, 182, 54, 0, 0, 0, 0, 0, 0, 32, 52, 53, 6, 2, 38, 19, 254, 36, 30, 18, 18, 51, 9, 0 };

        public static readonly byte[] MAVLINK_MESSAGE_CRCS = new byte[] { 50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 153, 183, 51, 59, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 163, 105, 151, 35, 179, 0, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0 };

        public static readonly Type[] MAVLINK_MESSAGE_INFO = new Type[] { typeof(mavlink_heartbeat_t), // 0
            typeof(mavlink_sys_status_t), // 1
            typeof(mavlink_system_time_t), // 2
            null, // 3
            typeof(mavlink_ping_t), // 4 
            typeof(mavlink_change_operator_control_t), // 5
            typeof(mavlink_change_operator_control_ack_t), // 6
            typeof(mavlink_auth_key_t), // 7
            null, // 8
            null, // 9
            null, // 10
            typeof(mavlink_set_mode_t), // 11
            null,                                                                // 12
            null,                                                                // 13
            null,                                                                // 14
            null,                                                                // 15
            null,                                                                // 16
            null,                                                                // 17
            null,                                                                // 18
            null,                                                                // 19
            typeof(mavlink_param_request_read_t),                                // 20
            typeof(mavlink_param_request_list_t),                                // 21
            typeof(mavlink_param_value_t),                                       // 22
            typeof(mavlink_param_set_t),                                         // 23
            typeof(mavlink_gps_raw_int_t),                                       // 24
            typeof(mavlink_gps_status_t),                                        // 25
            typeof(mavlink_scaled_imu_t),                                        // 26
            typeof(mavlink_raw_imu_t),                                           // 27
            typeof(mavlink_raw_pressure_t),                                      // 28
            typeof(mavlink_scaled_pressure_t),                                   // 29 
            typeof(mavlink_attitude_t),                                          // 30
            typeof(mavlink_attitude_quaternion_t),                               // 31
            typeof(mavlink_local_position_ned_t),                                // 32
            typeof(mavlink_global_position_int_t),                               // 33
            typeof(mavlink_rc_channels_scaled_t),                                // 34
            typeof(mavlink_rc_channels_raw_t),                                   // 35
            typeof(mavlink_servo_output_raw_t),                                  // 36
            typeof(mavlink_mission_request_partial_list_t),                      // 37
            typeof(mavlink_mission_write_partial_list_t),                        // 38
            typeof(mavlink_mission_item_t),                                      // 39 
            typeof(mavlink_mission_request_t),                                   // 40
            typeof(mavlink_mission_set_current_t),                               // 41
            typeof(mavlink_mission_current_t),                                   // 42
            typeof(mavlink_mission_request_list_t),                              // 43
            typeof(mavlink_mission_count_t),                                     // 44
            typeof(mavlink_mission_clear_all_t),                                 // 45
            typeof(mavlink_mission_item_reached_t),                              // 46
            typeof(mavlink_mission_ack_t),                                       // 47
            typeof(mavlink_set_gps_global_origin_t),                             // 48
            typeof(mavlink_gps_global_origin_t),                                 // 49
            typeof(mavlink_param_map_rc_t),                                      // 50
            typeof(mavlink_mission_request_int_t),                               // 51
            null,                                                                // 52
            null,                                                                // 53
            typeof(mavlink_safety_set_allowed_area_t),                           // 54
            typeof(mavlink_safety_allowed_area_t),                               // 55
            null,                                                                // 56
            null,                                                                // 57
            null,                                                                // 58
            null,                                                                // 59
            null,                                                                // 60
            typeof(mavlink_attitude_quaternion_cov_t),                           // 61
            typeof(mavlink_nav_controller_output_t),                             // 62
            typeof(mavlink_global_position_int_cov_t),                           // 63
            typeof(mavlink_local_position_ned_cov_t),                            // 64
            typeof(mavlink_rc_channels_t),                                       // 65
            typeof(mavlink_request_data_stream_t),                               // 66
            typeof(mavlink_data_stream_t),                                       // 67
            null,                                                                // 68
            typeof(mavlink_manual_control_t),                                    // 69
            typeof(mavlink_rc_channels_override_t),                              // 70
            null,                                                                // 71
            null,                                                                // 72
            typeof(mavlink_mission_item_int_t),                                  // 73
            typeof(mavlink_vfr_hud_t),                                           // 74
            typeof(mavlink_command_int_t),                                       // 75
            typeof(mavlink_command_long_t),                                      // 76
            typeof(mavlink_command_ack_t),                                       // 77
            null,                                                                // 78
            null,                                                                // 79
            null,                                                                // 80
            typeof(mavlink_manual_setpoint_t),                                   // 81
            typeof(mavlink_set_attitude_target_t),                               // 82
            typeof(mavlink_attitude_target_t),                                   // 83
            typeof(mavlink_set_position_target_local_ned_t),                     // 84
            typeof(mavlink_position_target_local_ned_t),                         // 85
            typeof(mavlink_set_position_target_global_int_t),                    // 86
            typeof(mavlink_position_target_global_int_t),                        // 87
            null,                                                                // 88
            typeof(mavlink_local_position_ned_system_global_offset_t),           // 89
            typeof(mavlink_hil_state_t),                                         // 90
            typeof(mavlink_hil_controls_t),                                      // 91
            typeof(mavlink_hil_rc_inputs_raw_t),                                 // 92
            typeof(mavlink_hil_actuator_controls_t),                             // 93
            null,                                                                // 94
            null,                                                                // 95
            null,                                                                // 96
            null,                                                                // 97
            null,                                                                // 98
            null,                                                                // 99
            typeof(mavlink_optical_flow_t),                                      // 100
            typeof(mavlink_global_vision_position_estimate_t),                   // 101
            typeof(mavlink_vision_position_estimate_t),                          // 102
            typeof(mavlink_vision_speed_estimate_t),                             // 103
            typeof(mavlink_vicon_position_estimate_t),                           // 104
            typeof(mavlink_highres_imu_t),                                       // 105
            typeof(mavlink_optical_flow_rad_t),                                  // 106
            typeof(mavlink_hil_sensor_t),                                        // 107
            typeof(mavlink_sim_state_t),                                         // 108
            typeof(mavlink_radio_status_t),                                      // 109
            typeof(mavlink_file_transfer_protocol_t),                            // 110
            typeof(mavlink_timesync_t),                                          // 111
            typeof(mavlink_camera_trigger_t),                                    // 112
            typeof(mavlink_hil_gps_t),                                           // 113
            typeof(mavlink_hil_optical_flow_t),                                  // 114
            typeof(mavlink_hil_state_quaternion_t),                              // 115
            typeof(mavlink_scaled_imu2_t),                                       // 116
            typeof(mavlink_log_request_list_t),                                  // 117
            typeof(mavlink_log_entry_t),                                         // 118
            typeof(mavlink_log_request_data_t),                                  // 119
            typeof(mavlink_log_data_t),                                          // 120
            typeof(mavlink_log_erase_t),                                         // 121
            typeof(mavlink_log_request_end_t),                                   // 122
            typeof(mavlink_gps_inject_data_t),                                   // 123
            typeof(mavlink_gps2_raw_t),                                          // 124
            typeof(mavlink_power_status_t),                                      // 125
            typeof(mavlink_serial_control_t),                                    // 126
            typeof(mavlink_gps_rtk_t),                                           // 127
            typeof(mavlink_gps2_rtk_t),                                          // 128
            typeof(mavlink_scaled_imu3_t),                                       // 129
            typeof(mavlink_data_transmission_handshake_t),                       // 130
            typeof(mavlink_encapsulated_data_t),                                 // 131
            typeof(mavlink_distance_sensor_t),                                   // 132
            typeof(mavlink_terrain_request_t),                                   // 133
            typeof(mavlink_terrain_data_t),                                      // 134
            typeof(mavlink_terrain_check_t),                                     // 135
            typeof(mavlink_terrain_report_t),                                    // 136
            typeof(mavlink_scaled_pressure2_t),                                  // 137
            typeof(mavlink_att_pos_mocap_t),                                     // 138
            typeof(mavlink_set_actuator_control_target_t),                       // 139
            typeof(mavlink_actuator_control_target_t),                           // 140
            typeof(mavlink_altitude_t),                                          // 141
            typeof(mavlink_resource_request_t),                                  // 142
            typeof(mavlink_scaled_pressure3_t),                                  // 143
            null,                                                                // 144
            null,                                                                // 145
            typeof(mavlink_control_system_state_t),                              // 146
            typeof(mavlink_battery_status_t),                                    // 147
            typeof(mavlink_autopilot_version_t),                                 // 148
            typeof(mavlink_landing_target_t),                                    // 149
            typeof(mavlink_sensor_offsets_t),                                    // 150
            typeof(mavlink_set_mag_offsets_t),                                   // 151
            typeof(mavlink_meminfo_t),                                           // 152
            typeof(mavlink_ap_adc_t),                                            // 153
            typeof(mavlink_digicam_configure_t),                                 // 154
            typeof(mavlink_digicam_control_t),                                   // 155
            typeof(mavlink_mount_configure_t),                                   // 156
            typeof(mavlink_mount_control_t),                                     // 157
            typeof(mavlink_mount_status_t),                                      // 158
            null,                                                                // 159
            typeof(mavlink_fence_point_t),                                       // 160
            typeof(mavlink_fence_fetch_point_t),                                 // 161
            typeof(mavlink_fence_status_t),                                      // 162
            typeof(mavlink_ahrs_t),                                              // 163
            typeof(mavlink_simstate_t),                                          // 164
            typeof(mavlink_hwstatus_t),                                          // 165
            typeof(mavlink_radio_t),                                             // 166
            typeof(mavlink_limits_status_t),                                     // 167
            typeof(mavlink_wind_t),                                              // 168
            typeof(mavlink_data16_t),                                            // 169
            typeof(mavlink_data32_t),                                            // 170
            typeof(mavlink_data64_t),                                            // 171
            typeof(mavlink_data96_t),                                            // 172
            typeof(mavlink_rangefinder_t),                                       // 173
            typeof(mavlink_airspeed_autocal_t),                                  // 174
            typeof(mavlink_rally_point_t),                                       // 175
            typeof(mavlink_rally_fetch_point_t),                                 // 176
            typeof(mavlink_compassmot_status_t),                                 // 177
            typeof(mavlink_ahrs2_t),                                             // 178
            typeof(mavlink_camera_status_t),                                     // 179
            typeof(mavlink_camera_feedback_t),                                   // 180
            typeof(mavlink_battery2_t),                                          // 181
            typeof(mavlink_ahrs3_t),                                             // 182
            typeof(mavlink_autopilot_version_request_t),                         // 183
            typeof(mavlink_remote_log_data_block_t),                             // 184
            typeof(mavlink_remote_log_block_status_t),                           // 1858
            typeof(mavlink_led_control_t),                                       // 186
            null,                                                                // 187
            null,                                                                // 188
            null,                                                                // 189
            null,                                                                // 190
            typeof(mavlink_mag_cal_progress_t),                                  // 191
            typeof(mavlink_mag_cal_report_t),                                    // 192
            typeof(mavlink_ekf_status_report_t),                                 // 193
            typeof(mavlink_pid_tuning_t),                                        // 194
            null,                                                                // 195
            null,                                                                // 196
            null,                                                                // 197
            null,                                                                // 198
            null,                                                                // 199
            typeof(mavlink_gimbal_report_t),                                     // 200
            typeof(mavlink_gimbal_control_t),                                    // 201
            null,                                                                // 202
            null,                                                                // 203
            typeof(mavlink_telemetry),                                           // 204
            null,                                                                // 205
            null,                                                                // 206
            null,                                                                // 207
            null,                                                                // 208
            null,                                                                // 209
            null,                                                                // 210
            null,                                                                // 211
            null,                                                                // 212
            null,                                                                // 213
            typeof(mavlink_gimbal_torque_cmd_report_t),                          // 214
            typeof(mavlink_gopro_heartbeat_t),                                   // 215
            typeof(mavlink_gopro_get_request_t),                                 // 216
            typeof(mavlink_gopro_get_response_t),                                // 217
            typeof(mavlink_gopro_set_request_t),                                 // 218
            typeof(mavlink_gopro_set_response_t),                                // 219
            null,                                                                // 220
            null,                                                                // 221
            null,                                                                // 222
            null,                                                                // 223
            null,                                                                // 224
            null,                                                                // 225
            typeof(mavlink_rpm_t),                                               // 226
            null,                                                                // 227
            null,                                                                // 228
            null,                                                                // 229
            null,                                                                // 230
            null,                                                                // 231
            null,                                                                // 232
            null,                                                                // 233
            null,                                                                // 234
            null,                                                                // 235
            null,                                                                // 236
            null,                                                                // 237
            null,                                                                // 238
            null,                                                                // 239
            null,                                                                // 240
            typeof(mavlink_vibration_t),                                         // 241
            typeof(mavlink_home_position_t),                                     // 242
            typeof(mavlink_set_home_position_t),                                 // 243
            typeof(mavlink_message_interval_t),                                  // 244
            typeof(mavlink_extended_sys_state_t),                                // 245
            typeof(mavlink_adsb_vehicle_t),                                      // 246
            typeof(mavlink_collision_t),                                         // 247
            typeof(mavlink_v2_extension_t),                                      // 248
            typeof(mavlink_memory_vect_t),                                       // 249
            typeof(mavlink_debug_vect_t),                                        // 250
            typeof(mavlink_named_value_float_t),                                 // 251
            typeof(mavlink_named_value_int_t),                                   // 252
            typeof(mavlink_statustext_t),                                        // 253
            typeof(mavlink_debug_t),                                             // 254
            null                                                                 // 255
            };

        public static readonly string[] MAVLINK_NAMES = new string[] { "HEARTBEAT", "SYS_STATUS", "SYSTEM_TIME", null, "PING", "CHANGE_OPERATOR_CONTROL", "CHANGE_OPERATOR_CONTROL_ACK", "AUTH_KEY", null, null, null, "SET_MODE", null, null, null, null, null, null, null, null, "PARAM_REQUEST_READ", "PARAM_REQUEST_LIST", "PARAM_VALUE", "PARAM_SET", "GPS_RAW_INT", "GPS_STATUS", "SCALED_IMU", "RAW_IMU", "RAW_PRESSURE", "SCALED_PRESSURE", "ATTITUDE", "ATTITUDE_QUATERNION", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "RC_CHANNELS_SCALED", "RC_CHANNELS_RAW", "SERVO_OUTPUT_RAW", "MISSION_REQUEST_PARTIAL_LIST", "MISSION_WRITE_PARTIAL_LIST", "MISSION_ITEM", "MISSION_REQUEST", "MISSION_SET_CURRENT", "MISSION_CURRENT", "MISSION_REQUEST_LIST", "MISSION_COUNT", "MISSION_CLEAR_ALL", "MISSION_ITEM_REACHED", "MISSION_ACK", "SET_GPS_GLOBAL_ORIGIN", "GPS_GLOBAL_ORIGIN", "PARAM_MAP_RC", "MISSION_REQUEST_INT", null, null, "SAFETY_SET_ALLOWED_AREA", "SAFETY_ALLOWED_AREA", null, null, null, null, null, "ATTITUDE_QUATERNION_COV", "NAV_CONTROLLER_OUTPUT", "GLOBAL_POSITION_INT_COV", "LOCAL_POSITION_NED_COV", "RC_CHANNELS", "REQUEST_DATA_STREAM", "DATA_STREAM", null, "MANUAL_CONTROL", "RC_CHANNELS_OVERRIDE", null, null, "MISSION_ITEM_INT", "VFR_HUD", "COMMAND_INT", "COMMAND_LONG", "COMMAND_ACK", null, null, null, "MANUAL_SETPOINT", "SET_ATTITUDE_TARGET", "ATTITUDE_TARGET", "SET_POSITION_TARGET_LOCAL_NED", "POSITION_TARGET_LOCAL_NED", "SET_POSITION_TARGET_GLOBAL_INT", "POSITION_TARGET_GLOBAL_INT", null, "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET", "HIL_STATE", "HIL_CONTROLS", "HIL_RC_INPUTS_RAW", null, null, null, null, null, null, null, "OPTICAL_FLOW", "GLOBAL_VISION_POSITION_ESTIMATE", "VISION_POSITION_ESTIMATE", "VISION_SPEED_ESTIMATE", "VICON_POSITION_ESTIMATE", "HIGHRES_IMU", "OPTICAL_FLOW_RAD", "HIL_SENSOR", "SIM_STATE", "RADIO_STATUS", "FILE_TRANSFER_PROTOCOL", "TIMESYNC", "CAMERA_TRIGGER", "HIL_GPS", "HIL_OPTICAL_FLOW", "HIL_STATE_QUATERNION", "SCALED_IMU2", "LOG_REQUEST_LIST", "LOG_ENTRY", "LOG_REQUEST_DATA", "LOG_DATA", "LOG_ERASE", "LOG_REQUEST_END", "GPS_INJECT_DATA", "GPS2_RAW", "POWER_STATUS", "SERIAL_CONTROL", "GPS_RTK", "GPS2_RTK", "SCALED_IMU3", "DATA_TRANSMISSION_HANDSHAKE", "ENCAPSULATED_DATA", "DISTANCE_SENSOR", "TERRAIN_REQUEST", "TERRAIN_DATA", "TERRAIN_CHECK", "TERRAIN_REPORT", "SCALED_PRESSURE2", "ATT_POS_MOCAP", "SET_ACTUATOR_CONTROL_TARGET", "ACTUATOR_CONTROL_TARGET", "ALTITUDE", "RESOURCE_REQUEST", "SCALED_PRESSURE3", null, null, "CONTROL_SYSTEM_STATE", "BATTERY_STATUS", "AUTOPILOT_VERSION", "LANDING_TARGET", "SENSOR_OFFSETS", "SET_MAG_OFFSETS", "MEMINFO", "AP_ADC", "DIGICAM_CONFIGURE", "DIGICAM_CONTROL", "MOUNT_CONFIGURE", "MOUNT_CONTROL", "MOUNT_STATUS", null, "FENCE_POINT", "FENCE_FETCH_POINT", "FENCE_STATUS", "AHRS", "SIMSTATE", "HWSTATUS", "RADIO", "LIMITS_STATUS", "WIND", "DATA16", "DATA32", "DATA64", "DATA96", "RANGEFINDER", "AIRSPEED_AUTOCAL", "RALLY_POINT", "RALLY_FETCH_POINT", "COMPASSMOT_STATUS", "AHRS2", "CAMERA_STATUS", "CAMERA_FEEDBACK", "BATTERY2", "AHRS3", "AUTOPILOT_VERSION_REQUEST", "REMOTE_LOG_DATA_BLOCK", "REMOTE_LOG_BLOCK_STATUS", "LED_CONTROL", null, null, null, null, "MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "EKF_STATUS_REPORT", "PID_TUNING", null, null, null, null, null, "GIMBAL_REPORT", "GIMBAL_CONTROL", null, null, null, null, null, null, null, null, null, null, null, null, "GIMBAL_TORQUE_CMD_REPORT", "GOPRO_HEARTBEAT", "GOPRO_GET_REQUEST", "GOPRO_GET_RESPONSE", "GOPRO_SET_REQUEST", "GOPRO_SET_RESPONSE", null, null, null, null, null, null, "RPM", null, null, null, null, null, null, null, null, null, null, null, null, null, null, "VIBRATION", "HOME_POSITION", "SET_HOME_POSITION", "MESSAGE_INTERVAL", "EXTENDED_SYS_STATE", "ADSB_VEHICLE", null, "V2_EXTENSION", "MEMORY_VECT", "DEBUG_VECT", "NAMED_VALUE_FLOAT", "NAMED_VALUE_INT", "STATUSTEXT", "DEBUG", null };

        public const byte MAVLINK_VERSION = 2;

        public enum MAVLINK_MSG_ID
        {
            HEARTBEAT = 0,
            SYS_STATUS = 1,
            SYSTEM_TIME = 2,
            PING = 4,
            CHANGE_OPERATOR_CONTROL = 5,
            CHANGE_OPERATOR_CONTROL_ACK = 6,
            AUTH_KEY = 7,
            SET_MODE = 11,
            PARAM_REQUEST_READ = 20,
            PARAM_REQUEST_LIST = 21,
            PARAM_VALUE = 22,
            PARAM_SET = 23,
            GPS_RAW_INT = 24,
            GPS_STATUS = 25,
            SCALED_IMU = 26,
            RAW_IMU = 27,
            RAW_PRESSURE = 28,
            SCALED_PRESSURE = 29,
            ATTITUDE = 30,
            ATTITUDE_QUATERNION = 31,
            LOCAL_POSITION_NED = 32,
            GLOBAL_POSITION_INT = 33,
            RC_CHANNELS_SCALED = 34,
            RC_CHANNELS_RAW = 35,
            SERVO_OUTPUT_RAW = 36,
            MISSION_REQUEST_PARTIAL_LIST = 37,
            MISSION_WRITE_PARTIAL_LIST = 38,
            MISSION_ITEM = 39,
            MISSION_REQUEST = 40,
            MISSION_SET_CURRENT = 41,
            MISSION_CURRENT = 42,
            MISSION_REQUEST_LIST = 43,
            MISSION_COUNT = 44,
            MISSION_CLEAR_ALL = 45,
            MISSION_ITEM_REACHED = 46,
            MISSION_ACK = 47,
            SET_GPS_GLOBAL_ORIGIN = 48,
            GPS_GLOBAL_ORIGIN = 49,
            PARAM_MAP_RC = 50,
            MISSION_REQUEST_INT = 51,
            SAFETY_SET_ALLOWED_AREA = 54,
            SAFETY_ALLOWED_AREA = 55,
            ATTITUDE_QUATERNION_COV = 61,
            NAV_CONTROLLER_OUTPUT = 62,
            GLOBAL_POSITION_INT_COV = 63,
            LOCAL_POSITION_NED_COV = 64,
            RC_CHANNELS = 65,
            REQUEST_DATA_STREAM = 66,
            DATA_STREAM = 67,
            MANUAL_CONTROL = 69,
            RC_CHANNELS_OVERRIDE = 70,
            MISSION_ITEM_INT = 73,
            VFR_HUD = 74,
            COMMAND_INT = 75,
            COMMAND_LONG = 76,
            COMMAND_ACK = 77,
            MANUAL_SETPOINT = 81,
            SET_ATTITUDE_TARGET = 82,
            ATTITUDE_TARGET = 83,
            SET_POSITION_TARGET_LOCAL_NED = 84,
            POSITION_TARGET_LOCAL_NED = 85,
            SET_POSITION_TARGET_GLOBAL_INT = 86,
            POSITION_TARGET_GLOBAL_INT = 87,
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = 89,
            HIL_STATE = 90,
            HIL_CONTROLS = 91,
            HIL_RC_INPUTS_RAW = 92,
            HIL_ACTUATOR_CONTROLS = 93,
            OPTICAL_FLOW = 100,
            GLOBAL_VISION_POSITION_ESTIMATE = 101,
            VISION_POSITION_ESTIMATE = 102,
            VISION_SPEED_ESTIMATE = 103,
            VICON_POSITION_ESTIMATE = 104,
            HIGHRES_IMU = 105,
            OPTICAL_FLOW_RAD = 106,
            HIL_SENSOR = 107,
            SIM_STATE = 108,
            RADIO_STATUS = 109,
            FILE_TRANSFER_PROTOCOL = 110,
            TIMESYNC = 111,
            CAMERA_TRIGGER = 112,
            HIL_GPS = 113,
            HIL_OPTICAL_FLOW = 114,
            HIL_STATE_QUATERNION = 115,
            SCALED_IMU2 = 116,
            LOG_REQUEST_LIST = 117,
            LOG_ENTRY = 118,
            LOG_REQUEST_DATA = 119,
            LOG_DATA = 120,
            LOG_ERASE = 121,
            LOG_REQUEST_END = 122,
            GPS_INJECT_DATA = 123,
            GPS2_RAW = 124,
            POWER_STATUS = 125,
            SERIAL_CONTROL = 126,
            GPS_RTK = 127,
            GPS2_RTK = 128,
            SCALED_IMU3 = 129,
            DATA_TRANSMISSION_HANDSHAKE = 130,
            ENCAPSULATED_DATA = 131,
            DISTANCE_SENSOR = 132,
            TERRAIN_REQUEST = 133,
            TERRAIN_DATA = 134,
            TERRAIN_CHECK = 135,
            TERRAIN_REPORT = 136,
            SCALED_PRESSURE2 = 137,
            ATT_POS_MOCAP = 138,
            SET_ACTUATOR_CONTROL_TARGET = 139,
            ACTUATOR_CONTROL_TARGET = 140,
            ALTITUDE = 141,
            RESOURCE_REQUEST = 142,
            SCALED_PRESSURE3 = 143,
            CONTROL_SYSTEM_STATE = 146,
            BATTERY_STATUS = 147,
            AUTOPILOT_VERSION = 148,
            LANDING_TARGET = 149,
            SENSOR_OFFSETS = 150,
            SET_MAG_OFFSETS = 151,
            MEMINFO = 152,
            AP_ADC = 153,
            DIGICAM_CONFIGURE = 154,
            DIGICAM_CONTROL = 155,
            MOUNT_CONFIGURE = 156,
            MOUNT_CONTROL = 157,
            MOUNT_STATUS = 158,
            FENCE_POINT = 160,
            FENCE_FETCH_POINT = 161,
            FENCE_STATUS = 162,
            AHRS = 163,
            SIMSTATE = 164,
            HWSTATUS = 165,
            RADIO = 166,
            LIMITS_STATUS = 167,
            WIND = 168,
            DATA16 = 169,
            DATA32 = 170,
            DATA64 = 171,
            DATA96 = 172,
            RANGEFINDER = 173,
            AIRSPEED_AUTOCAL = 174,
            RALLY_POINT = 175,
            RALLY_FETCH_POINT = 176,
            COMPASSMOT_STATUS = 177,
            AHRS2 = 178,
            CAMERA_STATUS = 179,
            CAMERA_FEEDBACK = 180,
            BATTERY2 = 181,
            AHRS3 = 182,
            AUTOPILOT_VERSION_REQUEST = 183,
            REMOTE_LOG_DATA_BLOCK = 184,
            REMOTE_LOG_BLOCK_STATUS = 185,
            LED_CONTROL = 186,
            MAG_CAL_PROGRESS = 191,
            MAG_CAL_REPORT = 192,
            EKF_STATUS_REPORT = 193,
            PID_TUNING = 194,
            GIMBAL_REPORT = 200,
            GIMBAL_CONTROL = 201,
            GIMBAL_TORQUE_CMD_REPORT = 214,
            GOPRO_HEARTBEAT = 215,
            GOPRO_GET_REQUEST = 216,
            GOPRO_GET_RESPONSE = 217,
            GOPRO_SET_REQUEST = 218,
            GOPRO_SET_RESPONSE = 219,
            RPM = 226,
            VIBRATION = 241,
            HOME_POSITION = 242,
            SET_HOME_POSITION = 243,
            MESSAGE_INTERVAL = 244,
            EXTENDED_SYS_STATE = 245,
            ADSB_VEHICLE = 246,
            V2_EXTENSION = 248,
            MEMORY_VECT = 249,
            DEBUG_VECT = 250,
            NAMED_VALUE_FLOAT = 251,
            NAMED_VALUE_INT = 252,
            STATUSTEXT = 253,
            DEBUG = 254,

        }



        ///<summary>  </summary>
        public enum MAV_CMD
        {
            ///<summary> Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  </summary>
            WAYPOINT = 16,
            ///<summary> Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  </summary>
            LOITER_UNLIM = 17,
            ///<summary> Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  </summary>
            LOITER_TURNS = 18,
            ///<summary> Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  </summary>
            LOITER_TIME = 19,
            ///<summary> Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            RETURN_TO_LAUNCH = 20,
            ///<summary> Land at location |Abort Alt| Empty| Empty| Desired yaw angle| Latitude| Longitude| Altitude|  </summary>
            LAND = 21,
            ///<summary> Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  </summary>
            TAKEOFF = 22,
            ///<summary> Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate [ms^-1]| Desired yaw angle [rad]| Y-axis position [m]| X-axis position [m]| Z-axis / ground level position [m]|  </summary>
            LAND_LOCAL = 23,
            ///<summary> Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]| Empty| Takeoff ascend rate [ms^-1]| Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position [m]| X-axis position [m]| Z-axis position [m]|  </summary>
            TAKEOFF_LOCAL = 24,
            ///<summary> Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  </summary>
            FOLLOW = 25,
            ///<summary> Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. | Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  </summary>
            CONTINUE_AND_CHANGE_ALT = 30,
            ///<summary> Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.  |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Empty| Latitude| Longitude| Altitude|  </summary>
            LOITER_TO_ALT = 31,
            ///<summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  </summary>
            ROI = 80,
            ///<summary> Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  </summary>
            PATHPLANNING = 81,
            ///<summary> Navigate to MISSION using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  </summary>
            SPLINE_WAYPOINT = 82,
            ///<summary> Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |altitude (m)| descent speed (m/s)| Wiggle Time (s)| Empty| Empty| Empty| Empty|  </summary>
            ALTITUDE_WAIT = 83,
            ///<summary> Takeoff from ground using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees| Latitude| Longitude| Altitude|  </summary>
            VTOL_TAKEOFF = 84,
            ///<summary> Land using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees| Latitude| Longitude| Altitude|  </summary>
            VTOL_LAND = 85,
            ///<summary> hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            GUIDED_ENABLE = 92,
            ///<summary> NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            LAST = 95,
            ///<summary> Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            CONDITION_DELAY = 112,
            ///<summary> Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  </summary>
            CONDITION_CHANGE_ALT = 113,
            ///<summary> Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            CONDITION_DISTANCE = 114,
            ///<summary> Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  </summary>
            CONDITION_YAW = 115,
            ///<summary> NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            CONDITION_LAST = 159,
            ///<summary> Set system mode. |Mode, as defined by ENUM MAV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  </summary>
            DO_SET_MODE = 176,
            ///<summary> Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_JUMP = 177,
            ///<summary> Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  </summary>
            DO_CHANGE_SPEED = 178,
            ///<summary> Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  </summary>
            DO_SET_HOME = 179,
            ///<summary> Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_SET_PARAMETER = 180,
            ///<summary> Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_SET_RELAY = 181,
            ///<summary> Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  </summary>
            DO_REPEAT_RELAY = 182,
            ///<summary> Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_SET_SERVO = 183,
            ///<summary> Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  </summary>
            DO_REPEAT_SERVO = 184,
            ///<summary> Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_FLIGHTTERMINATION = 185,
            ///<summary> Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  </summary>
            DO_LAND_START = 189,
            ///<summary> Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_RALLY_LAND = 190,
            ///<summary> Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_GO_AROUND = 191,
            ///<summary> Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  </summary>
            DO_CONTROL_VIDEO = 200,
            ///<summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  </summary>
            DO_SET_ROI = 201,
            ///<summary> Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  </summary>
            DO_DIGICAM_CONFIGURE = 202,
            ///<summary> Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  </summary>
            DO_DIGICAM_CONTROL = 203,
            ///<summary> Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  </summary>
            DO_MOUNT_CONFIGURE = 204,
            ///<summary> Mission command to control a camera or antenna mount |pitch or lat in degrees, depending on mount mode.| roll or lon in degrees depending on mount mode| yaw or alt (in meters) depending on mount mode| reserved| reserved| reserved| MAV_MOUNT_MODE enum value|  </summary>
            DO_MOUNT_CONTROL = 205,
            ///<summary> Mission command to set CAM_TRIGG_DIST for this flight |Camera trigger distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_SET_CAM_TRIGG_DIST = 206,
            ///<summary> Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_FENCE_ENABLE = 207,
            ///<summary> Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_PARACHUTE = 208,
            ///<summary> Mission command to perform motor test |motor sequence number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| Empty| Empty| Empty|  </summary>
            DO_MOTOR_TEST = 209,
            ///<summary> Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_INVERTED_FLIGHT = 210,
            ///<summary> Mission command to operate EPM gripper |gripper number (a number from 1 to max number of grippers on the vehicle)| gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum)| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_GRIPPER = 211,
            ///<summary> Enable/disable autotune |enable (1: enable, 0:disable)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_AUTOTUNE_ENABLE = 212,
            ///<summary> Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  </summary>
            DO_MOUNT_CONTROL_QUAT = 220,
            ///<summary> set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_GUIDED_MASTER = 221,
            ///<summary> set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  </summary>
            DO_GUIDED_LIMITS = 222,
            ///<summary> NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_LAST = 240,
            ///<summary> Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Accelerometer calibration: 0: no, 1: yes| Compass/Motor interference calibration: 0: no, 1: yes| Empty|  </summary>
            PREFLIGHT_CALIBRATION = 241,
            ///<summary> Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  </summary>
            PREFLIGHT_SET_SENSOR_OFFSETS = 242,
            ///<summary> Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
            PREFLIGHT_UAVCAN = 243,
            ///<summary> Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  </summary>
            PREFLIGHT_STORAGE = 245,
            ///<summary> Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| Reserved, send 0| Reserved, send 0| Reserved, send 0| Reserved, send 0| Reserved, send 0|  </summary>
            PREFLIGHT_REBOOT_SHUTDOWN = 246,
            ///<summary> Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  </summary>
            OVERRIDE_GOTO = 252,
            ///<summary> start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  </summary>
            MISSION_START = 300,
            ///<summary> Arms / Disarms a component |1 to arm, 0 to disarm|  </summary>
            COMPONENT_ARM_DISARM = 400,
            ///<summary> Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
            GET_HOME_POSITION = 410,
            ///<summary> Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  </summary>
            START_RX_PAIR = 500,
            ///<summary> Request the interval between messages for a particular MAVLink message ID |The MAVLink message ID|  </summary>
            GET_MESSAGE_INTERVAL = 510,
            ///<summary> Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM |The MAVLink message ID| The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.|  </summary>
            SET_MESSAGE_INTERVAL = 511,
            ///<summary> Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  </summary>
            REQUEST_AUTOPILOT_CAPABILITIES = 520,
            ///<summary> Start image capture sequence |Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  </summary>
            IMAGE_START_CAPTURE = 2000,
            ///<summary> Stop image capture sequence |Reserved| Reserved|  </summary>
            IMAGE_STOP_CAPTURE = 2001,
            ///<summary> Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start)| Shutter integration time (in ms)| Reserved|  </summary>
            DO_TRIGGER_CONTROL = 2003,
            ///<summary> Starts video capture |Camera ID (0 for all cameras), 1 for first, 2 for second, etc.| Frames per second| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  </summary>
            VIDEO_START_CAPTURE = 2500,
            ///<summary> Stop the current video capture |Reserved| Reserved|  </summary>
            VIDEO_STOP_CAPTURE = 2501,
            ///<summary> Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  </summary>
            PANORAMA_CREATE = 2800,
            ///<summary> Request VTOL transition |The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.|  </summary>
            DO_VTOL_TRANSITION = 3000,
            ///<summary> Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  </summary>
            PAYLOAD_PREPARE_DEPLOY = 30001,
            ///<summary> Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
            PAYLOAD_CONTROL_DEPLOY = 30002,
            ///<summary>  | </summary>
            ENUM_END = 30003,
            ///<summary> A system wide power-off event has been initiated. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            POWER_OFF_INITIATED = 42000,
            ///<summary> FLY button has been clicked. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            SOLO_BTN_FLY_CLICK = 42001,
            ///<summary> FLY button has been held for 1.5 seconds. |Takeoff altitude| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            SOLO_BTN_FLY_HOLD = 42002,
            ///<summary> PAUSE button has been clicked. |1 if Solo is in a shot mode, 0 otherwise| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            SOLO_BTN_PAUSE_CLICK = 42003,
            ///<summary> Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Automatically retry on failure (0=no retry, 1=retry).| Save without user input (0=require input, 1=autosave).| Delay (seconds)| Autoreboot (0=user reboot, 1=autoreboot)| Empty| Empty|  </summary>
            DO_START_MAG_CAL = 42424,
            ///<summary> Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_ACCEPT_MAG_CAL = 42425,
            ///<summary> Cancel a running magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_CANCEL_MAG_CAL = 42426,
            ///<summary> Command autopilot to get into factory test/diagnostic mode |0 means get out of test mode, 1 means get into test mode| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            SET_FACTORY_TEST_MODE = 42427,
            ///<summary> Reply with the version banner |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_SEND_BANNER = 42428,
            ///<summary> Causes the gimbal to reset and boot as if it was just powered on |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            GIMBAL_RESET = 42501,
            ///<summary> Reports progress and success or failure of gimbal axis calibration procedure |Gimbal axis we're reporting calibration progress for| Current calibration progress for this axis, 0x64=100%| Status of the calibration| Empty| Empty| Empty| Empty|  </summary>
            GIMBAL_AXIS_CALIBRATION_STATUS = 42502,
            ///<summary> Starts commutation calibration on the gimbal |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            GIMBAL_REQUEST_AXIS_CALIBRATION = 42503,
            ///<summary> Erases gimbal application and parameters |Magic number| Magic number| Magic number| Magic number| Magic number| Magic number| Magic number|  </summary>
            GIMBAL_FULL_RESET = 42505,

        };

        ///<summary>  </summary>
        public enum LIMITS_STATE
        {
            ///<summary> pre-initialization | </summary>
            LIMITS_INIT = 0,
            ///<summary> disabled | </summary>
            LIMITS_DISABLED = 1,
            ///<summary> checking limits | </summary>
            LIMITS_ENABLED = 2,
            ///<summary> a limit has been breached | </summary>
            LIMITS_TRIGGERED = 3,
            ///<summary> taking action eg. RTL | </summary>
            LIMITS_RECOVERING = 4,
            ///<summary> we're no longer in breach of a limit | </summary>
            LIMITS_RECOVERED = 5,
            ///<summary>  | </summary>
            ENUM_END = 6,

        };

        ///<summary>  </summary>
        public enum LIMIT_MODULE
        {
            ///<summary> pre-initialization | </summary>
            LIMIT_GPSLOCK = 1,
            ///<summary> disabled | </summary>
            LIMIT_GEOFENCE = 2,
            ///<summary> checking limits | </summary>
            LIMIT_ALTITUDE = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary> Flags in RALLY_POINT message </summary>
        public enum RALLY_FLAGS
        {
            ///<summary> Flag set when requiring favorable winds for landing. | </summary>
            FAVORABLE_WIND = 1,
            ///<summary> Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag not set when plane is to loiter at Rally point until commanded to land. | </summary>
            LAND_IMMEDIATELY = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary>  </summary>
        public enum PARACHUTE_ACTION
        {
            ///<summary> Disable parachute release | </summary>
            PARACHUTE_DISABLE = 0,
            ///<summary> Enable parachute release | </summary>
            PARACHUTE_ENABLE = 1,
            ///<summary> Release parachute | </summary>
            PARACHUTE_RELEASE = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary>  </summary>
        public enum MOTOR_TEST_THROTTLE_TYPE
        {
            ///<summary> throttle as a percentage from 0 ~ 100 | </summary>
            MOTOR_TEST_THROTTLE_PERCENT = 0,
            ///<summary> throttle as an absolute PWM value (normally in range of 1000~2000) | </summary>
            MOTOR_TEST_THROTTLE_PWM = 1,
            ///<summary> throttle pass-through from pilot's transmitter | </summary>
            MOTOR_TEST_THROTTLE_PILOT = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary> Gripper actions. </summary>
        public enum GRIPPER_ACTIONS
        {
            ///<summary> gripper release of cargo | </summary>
            GRIPPER_ACTION_RELEASE = 0,
            ///<summary> gripper grabs onto cargo | </summary>
            GRIPPER_ACTION_GRAB = 1,
            ///<summary>  | </summary>
            ENUM_END = 2,

        };

        ///<summary>  </summary>
        public enum CAMERA_STATUS_TYPES
        {
            ///<summary> Camera heartbeat, announce camera component ID at 1hz | </summary>
            CAMERA_STATUS_TYPE_HEARTBEAT = 0,
            ///<summary> Camera image triggered | </summary>
            CAMERA_STATUS_TYPE_TRIGGER = 1,
            ///<summary> Camera connection lost | </summary>
            CAMERA_STATUS_TYPE_DISCONNECT = 2,
            ///<summary> Camera unknown error | </summary>
            CAMERA_STATUS_TYPE_ERROR = 3,
            ///<summary> Camera battery low. Parameter p1 shows reported voltage | </summary>
            CAMERA_STATUS_TYPE_LOWBATT = 4,
            ///<summary> Camera storage low. Parameter p1 shows reported shots remaining | </summary>
            CAMERA_STATUS_TYPE_LOWSTORE = 5,
            ///<summary> Camera storage low. Parameter p1 shows reported video minutes remaining | </summary>
            CAMERA_STATUS_TYPE_LOWSTOREV = 6,
            ///<summary>  | </summary>
            ENUM_END = 7,

        };

        ///<summary>  </summary>
        public enum CAMERA_FEEDBACK_FLAGS
        {
            ///<summary> Shooting photos, not video | </summary>
            CAMERA_FEEDBACK_PHOTO = 0,
            ///<summary> Shooting video, not stills | </summary>
            CAMERA_FEEDBACK_VIDEO = 1,
            ///<summary> Unable to achieve requested exposure (e.g. shutter speed too low) | </summary>
            CAMERA_FEEDBACK_BADEXPOSURE = 2,
            ///<summary> Closed loop feedback from camera, we know for sure it has successfully taken a picture | </summary>
            CAMERA_FEEDBACK_CLOSEDLOOP = 3,
            ///<summary> Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture | </summary>
            CAMERA_FEEDBACK_OPENLOOP = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary>  </summary>
        public enum MAV_MODE_GIMBAL
        {
            ///<summary> Gimbal is powered on but has not started initializing yet | </summary>
            UNINITIALIZED = 0,
            ///<summary> Gimbal is currently running calibration on the pitch axis | </summary>
            CALIBRATING_PITCH = 1,
            ///<summary> Gimbal is currently running calibration on the roll axis | </summary>
            CALIBRATING_ROLL = 2,
            ///<summary> Gimbal is currently running calibration on the yaw axis | </summary>
            CALIBRATING_YAW = 3,
            ///<summary> Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate command from copter | </summary>
            INITIALIZED = 4,
            ///<summary> Gimbal is actively stabilizing | </summary>
            ACTIVE = 5,
            ///<summary> Gimbal is relaxed because it missed more than 10 expected rate command messages in a row. Gimbal will move back to active mode when it receives a new rate command | </summary>
            RATE_CMD_TIMEOUT = 6,
            ///<summary>  | </summary>
            ENUM_END = 7,

        };

        ///<summary>  </summary>
        public enum GIMBAL_AXIS
        {
            ///<summary> Gimbal yaw axis | </summary>
            YAW = 0,
            ///<summary> Gimbal pitch axis | </summary>
            PITCH = 1,
            ///<summary> Gimbal roll axis | </summary>
            ROLL = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary>  </summary>
        public enum GIMBAL_AXIS_CALIBRATION_STATUS
        {
            ///<summary> Axis calibration is in progress | </summary>
            IN_PROGRESS = 0,
            ///<summary> Axis calibration succeeded | </summary>
            SUCCEEDED = 1,
            ///<summary> Axis calibration failed | </summary>
            FAILED = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary>  </summary>
        public enum GIMBAL_AXIS_CALIBRATION_REQUIRED
        {
            ///<summary> Whether or not this axis requires calibration is unknown at this time | </summary>
            UNKNOWN = 0,
            ///<summary> This axis requires calibration | </summary>
            TRUE = 1,
            ///<summary> This axis does not require calibration | </summary>
            FALSE = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary>  </summary>
        public enum GOPRO_HEARTBEAT_STATUS
        {
            ///<summary> No GoPro connected | </summary>
            DISCONNECTED = 0,
            ///<summary> The detected GoPro is not HeroBus compatible | </summary>
            INCOMPATIBLE = 1,
            ///<summary> A HeroBus compatible GoPro is connected | </summary>
            CONNECTED = 2,
            ///<summary> An unrecoverable error was encountered with the connected GoPro, it may require a power cycle | </summary>
            ERROR = 3,
            ///<summary>  | </summary>
            ENUM_END = 4,

        };

        ///<summary>  </summary>
        public enum GOPRO_HEARTBEAT_FLAGS
        {
            ///<summary> GoPro is currently recording | </summary>
            GOPRO_FLAG_RECORDING = 1,
            ///<summary>  | </summary>
            ENUM_END = 2,

        };

        ///<summary>  </summary>
        public enum GOPRO_REQUEST_STATUS
        {
            ///<summary> The write message with ID indicated succeeded | </summary>
            GOPRO_REQUEST_SUCCESS = 0,
            ///<summary> The write message with ID indicated failed | </summary>
            GOPRO_REQUEST_FAILED = 1,
            ///<summary>  | </summary>
            ENUM_END = 2,

        };

        ///<summary>  </summary>
        public enum GOPRO_COMMAND
        {
            ///<summary> (Get/Set) | </summary>
            POWER = 0,
            ///<summary> (Get/Set) | </summary>
            CAPTURE_MODE = 1,
            ///<summary> (___/Set) | </summary>
            SHUTTER = 2,
            ///<summary> (Get/___) | </summary>
            BATTERY = 3,
            ///<summary> (Get/___) | </summary>
            MODEL = 4,
            ///<summary> (Get/Set) | </summary>
            VIDEO_SETTINGS = 5,
            ///<summary> (Get/Set) | </summary>
            LOW_LIGHT = 6,
            ///<summary> (Get/Set) | </summary>
            PHOTO_RESOLUTION = 7,
            ///<summary> (Get/Set) | </summary>
            PHOTO_BURST_RATE = 8,
            ///<summary> (Get/Set) | </summary>
            PROTUNE = 9,
            ///<summary> (Get/Set) Hero 3+ Only | </summary>
            PROTUNE_WHITE_BALANCE = 10,
            ///<summary> (Get/Set) Hero 3+ Only | </summary>
            PROTUNE_COLOUR = 11,
            ///<summary> (Get/Set) Hero 3+ Only | </summary>
            PROTUNE_GAIN = 12,
            ///<summary> (Get/Set) Hero 3+ Only | </summary>
            PROTUNE_SHARPNESS = 13,
            ///<summary> (Get/Set) Hero 3+ Only | </summary>
            PROTUNE_EXPOSURE = 14,
            ///<summary> (Get/Set) | </summary>
            TIME = 15,
            ///<summary> (Get/Set) | </summary>
            CHARGING = 16,
            ///<summary>  | </summary>
            ENUM_END = 17,

        };

        ///<summary>  </summary>
        public enum GOPRO_CAPTURE_MODE
        {
            ///<summary> Video mode | </summary>
            VIDEO = 0,
            ///<summary> Photo mode | </summary>
            PHOTO = 1,
            ///<summary> Burst mode, hero 3+ only | </summary>
            BURST = 2,
            ///<summary> Time lapse mode, hero 3+ only | </summary>
            TIME_LAPSE = 3,
            ///<summary> Multi shot mode, hero 4 only | </summary>
            MULTI_SHOT = 4,
            ///<summary> Playback mode, hero 4 only, silver only except when LCD or HDMI is connected to black | </summary>
            PLAYBACK = 5,
            ///<summary> Playback mode, hero 4 only | </summary>
            SETUP = 6,
            ///<summary> Mode not yet known | </summary>
            UNKNOWN = 255,
            ///<summary>  | </summary>
            ENUM_END = 256,

        };

        ///<summary>  </summary>
        public enum GOPRO_RESOLUTION
        {
            ///<summary> 848 x 480 (480p) | </summary>
            _480p = 0,
            ///<summary> 1280 x 720 (720p) | </summary>
            _720p = 1,
            ///<summary> 1280 x 960 (960p) | </summary>
            _960p = 2,
            ///<summary> 1920 x 1080 (1080p) | </summary>
            _1080p = 3,
            ///<summary> 1920 x 1440 (1440p) | </summary>
            _1440p = 4,
            ///<summary> 2704 x 1440 (2.7k-17:9) | </summary>
            _2_7k_17_9 = 5,
            ///<summary> 2704 x 1524 (2.7k-16:9) | </summary>
            _2_7k_16_9 = 6,
            ///<summary> 2704 x 2028 (2.7k-4:3) | </summary>
            _2_7k_4_3 = 7,
            ///<summary> 3840 x 2160 (4k-16:9) | </summary>
            _4k_16_9 = 8,
            ///<summary> 4096 x 2160 (4k-17:9) | </summary>
            _4k_17_9 = 9,
            ///<summary> 1280 x 720 (720p-SuperView) | </summary>
            _720p_SUPERVIEW = 10,
            ///<summary> 1920 x 1080 (1080p-SuperView) | </summary>
            _1080p_SUPERVIEW = 11,
            ///<summary> 2704 x 1520 (2.7k-SuperView) | </summary>
            _2_7k_SUPERVIEW = 12,
            ///<summary> 3840 x 2160 (4k-SuperView) | </summary>
            _4k_SUPERVIEW = 13,
            ///<summary>  | </summary>
            ENUM_END = 14,

        };

        ///<summary>  </summary>
        public enum GOPRO_FRAME_RATE
        {
            ///<summary> 12 FPS | </summary>
            _12 = 0,
            ///<summary> 15 FPS | </summary>
            _15 = 1,
            ///<summary> 24 FPS | </summary>
            _24 = 2,
            ///<summary> 25 FPS | </summary>
            _25 = 3,
            ///<summary> 30 FPS | </summary>
            _30 = 4,
            ///<summary> 48 FPS | </summary>
            _48 = 5,
            ///<summary> 50 FPS | </summary>
            _50 = 6,
            ///<summary> 60 FPS | </summary>
            _60 = 7,
            ///<summary> 80 FPS | </summary>
            _80 = 8,
            ///<summary> 90 FPS | </summary>
            _90 = 9,
            ///<summary> 100 FPS | </summary>
            _100 = 10,
            ///<summary> 120 FPS | </summary>
            _120 = 11,
            ///<summary> 240 FPS | </summary>
            _240 = 12,
            ///<summary> 12.5 FPS | </summary>
            _12_5 = 13,
            ///<summary>  | </summary>
            ENUM_END = 14,

        };

        ///<summary>  </summary>
        public enum GOPRO_FIELD_OF_VIEW
        {
            ///<summary> 0x00: Wide | </summary>
            WIDE = 0,
            ///<summary> 0x01: Medium | </summary>
            MEDIUM = 1,
            ///<summary> 0x02: Narrow | </summary>
            NARROW = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary>  </summary>
        public enum GOPRO_VIDEO_SETTINGS_FLAGS
        {
            ///<summary> 0=NTSC, 1=PAL | </summary>
            GOPRO_VIDEO_SETTINGS_TV_MODE = 1,
            ///<summary>  | </summary>
            ENUM_END = 2,

        };

        ///<summary>  </summary>
        public enum GOPRO_PHOTO_RESOLUTION
        {
            ///<summary> 5MP Medium | </summary>
            _5MP_MEDIUM = 0,
            ///<summary> 7MP Medium | </summary>
            _7MP_MEDIUM = 1,
            ///<summary> 7MP Wide | </summary>
            _7MP_WIDE = 2,
            ///<summary> 10MP Wide | </summary>
            _10MP_WIDE = 3,
            ///<summary> 12MP Wide | </summary>
            _12MP_WIDE = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary>  </summary>
        public enum GOPRO_PROTUNE_WHITE_BALANCE
        {
            ///<summary> Auto | </summary>
            AUTO = 0,
            ///<summary> 3000K | </summary>
            _3000K = 1,
            ///<summary> 5500K | </summary>
            _5500K = 2,
            ///<summary> 6500K | </summary>
            _6500K = 3,
            ///<summary> Camera Raw | </summary>
            RAW = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary>  </summary>
        public enum GOPRO_PROTUNE_COLOUR
        {
            ///<summary> Auto | </summary>
            STANDARD = 0,
            ///<summary> Neutral | </summary>
            NEUTRAL = 1,
            ///<summary>  | </summary>
            ENUM_END = 2,

        };

        ///<summary>  </summary>
        public enum GOPRO_PROTUNE_GAIN
        {
            ///<summary> ISO 400 | </summary>
            _400 = 0,
            ///<summary> ISO 800 (Only Hero 4) | </summary>
            _800 = 1,
            ///<summary> ISO 1600 | </summary>
            _1600 = 2,
            ///<summary> ISO 3200 (Only Hero 4) | </summary>
            _3200 = 3,
            ///<summary> ISO 6400 | </summary>
            _6400 = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary>  </summary>
        public enum GOPRO_PROTUNE_SHARPNESS
        {
            ///<summary> Low Sharpness | </summary>
            LOW = 0,
            ///<summary> Medium Sharpness | </summary>
            MEDIUM = 1,
            ///<summary> High Sharpness | </summary>
            HIGH = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary>  </summary>
        public enum GOPRO_PROTUNE_EXPOSURE
        {
            ///<summary> -5.0 EV (Hero 3+ Only) | </summary>
            NEG_5_0 = 0,
            ///<summary> -4.5 EV (Hero 3+ Only) | </summary>
            NEG_4_5 = 1,
            ///<summary> -4.0 EV (Hero 3+ Only) | </summary>
            NEG_4_0 = 2,
            ///<summary> -3.5 EV (Hero 3+ Only) | </summary>
            NEG_3_5 = 3,
            ///<summary> -3.0 EV (Hero 3+ Only) | </summary>
            NEG_3_0 = 4,
            ///<summary> -2.5 EV (Hero 3+ Only) | </summary>
            NEG_2_5 = 5,
            ///<summary> -2.0 EV | </summary>
            NEG_2_0 = 6,
            ///<summary> -1.5 EV | </summary>
            NEG_1_5 = 7,
            ///<summary> -1.0 EV | </summary>
            NEG_1_0 = 8,
            ///<summary> -0.5 EV | </summary>
            NEG_0_5 = 9,
            ///<summary> 0.0 EV | </summary>
            ZERO = 10,
            ///<summary> +0.5 EV | </summary>
            POS_0_5 = 11,
            ///<summary> +1.0 EV | </summary>
            POS_1_0 = 12,
            ///<summary> +1.5 EV | </summary>
            POS_1_5 = 13,
            ///<summary> +2.0 EV | </summary>
            POS_2_0 = 14,
            ///<summary> +2.5 EV (Hero 3+ Only) | </summary>
            POS_2_5 = 15,
            ///<summary> +3.0 EV (Hero 3+ Only) | </summary>
            POS_3_0 = 16,
            ///<summary> +3.5 EV (Hero 3+ Only) | </summary>
            POS_3_5 = 17,
            ///<summary> +4.0 EV (Hero 3+ Only) | </summary>
            POS_4_0 = 18,
            ///<summary> +4.5 EV (Hero 3+ Only) | </summary>
            POS_4_5 = 19,
            ///<summary> +5.0 EV (Hero 3+ Only) | </summary>
            POS_5_0 = 20,
            ///<summary>  | </summary>
            ENUM_END = 21,

        };

        ///<summary>  </summary>
        public enum GOPRO_CHARGING
        {
            ///<summary> Charging disabled | </summary>
            DISABLED = 0,
            ///<summary> Charging enabled | </summary>
            ENABLED = 1,
            ///<summary>  | </summary>
            ENUM_END = 2,

        };

        ///<summary>  </summary>
        public enum GOPRO_MODEL
        {
            ///<summary> Unknown gopro model | </summary>
            UNKNOWN = 0,
            ///<summary> Hero 3+ Silver (HeroBus not supported by GoPro) | </summary>
            HERO_3_PLUS_SILVER = 1,
            ///<summary> Hero 3+ Black | </summary>
            HERO_3_PLUS_BLACK = 2,
            ///<summary> Hero 4 Silver | </summary>
            HERO_4_SILVER = 3,
            ///<summary> Hero 4 Black | </summary>
            HERO_4_BLACK = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary>  </summary>
        public enum GOPRO_BURST_RATE
        {
            ///<summary> 3 Shots / 1 Second | </summary>
            _3_IN_1_SECOND = 0,
            ///<summary> 5 Shots / 1 Second | </summary>
            _5_IN_1_SECOND = 1,
            ///<summary> 10 Shots / 1 Second | </summary>
            _10_IN_1_SECOND = 2,
            ///<summary> 10 Shots / 2 Second | </summary>
            _10_IN_2_SECOND = 3,
            ///<summary> 10 Shots / 3 Second (Hero 4 Only) | </summary>
            _10_IN_3_SECOND = 4,
            ///<summary> 30 Shots / 1 Second | </summary>
            _30_IN_1_SECOND = 5,
            ///<summary> 30 Shots / 2 Second | </summary>
            _30_IN_2_SECOND = 6,
            ///<summary> 30 Shots / 3 Second | </summary>
            _30_IN_3_SECOND = 7,
            ///<summary> 30 Shots / 6 Second | </summary>
            _30_IN_6_SECOND = 8,
            ///<summary>  | </summary>
            ENUM_END = 9,

        };

        ///<summary>  </summary>
        public enum LED_CONTROL_PATTERN
        {
            ///<summary> LED patterns off (return control to regular vehicle control) | </summary>
            OFF = 0,
            ///<summary> LEDs show pattern during firmware update | </summary>
            FIRMWAREUPDATE = 1,
            ///<summary> Custom Pattern using custom bytes fields | </summary>
            CUSTOM = 255,
            ///<summary>  | </summary>
            ENUM_END = 256,

        };

        ///<summary> Flags in EKF_STATUS message </summary>
        public enum EKF_STATUS_FLAGS
        {
            ///<summary> set if EKF's attitude estimate is good | </summary>
            EKF_ATTITUDE = 1,
            ///<summary> set if EKF's horizontal velocity estimate is good | </summary>
            EKF_VELOCITY_HORIZ = 2,
            ///<summary> set if EKF's vertical velocity estimate is good | </summary>
            EKF_VELOCITY_VERT = 4,
            ///<summary> set if EKF's horizontal position (relative) estimate is good | </summary>
            EKF_POS_HORIZ_REL = 8,
            ///<summary> set if EKF's horizontal position (absolute) estimate is good | </summary>
            EKF_POS_HORIZ_ABS = 16,
            ///<summary> set if EKF's vertical position (absolute) estimate is good | </summary>
            EKF_POS_VERT_ABS = 32,
            ///<summary> set if EKF's vertical position (above ground) estimate is good | </summary>
            EKF_POS_VERT_AGL = 64,
            ///<summary> EKF is in constant position mode and does not know it's absolute or relative position | </summary>
            EKF_CONST_POS_MODE = 128,
            ///<summary> set if EKF's predicted horizontal position (relative) estimate is good | </summary>
            EKF_PRED_POS_HORIZ_REL = 256,
            ///<summary> set if EKF's predicted horizontal position (absolute) estimate is good | </summary>
            EKF_PRED_POS_HORIZ_ABS = 512,
            ///<summary>  | </summary>
            ENUM_END = 513,

        };

        ///<summary>  </summary>
        public enum PID_TUNING_AXIS
        {
            ///<summary>  | </summary>
            PID_TUNING_ROLL = 1,
            ///<summary>  | </summary>
            PID_TUNING_PITCH = 2,
            ///<summary>  | </summary>
            PID_TUNING_YAW = 3,
            ///<summary>  | </summary>
            PID_TUNING_ACCZ = 4,
            ///<summary>  | </summary>
            PID_TUNING_STEER = 5,
            ///<summary>  | </summary>
            ENUM_END = 6,

        };

        ///<summary>  </summary>
        public enum MAG_CAL_STATUS
        {
            ///<summary>  | </summary>
            MAG_CAL_NOT_STARTED = 0,
            ///<summary>  | </summary>
            MAG_CAL_WAITING_TO_START = 1,
            ///<summary>  | </summary>
            MAG_CAL_RUNNING_STEP_ONE = 2,
            ///<summary>  | </summary>
            MAG_CAL_RUNNING_STEP_TWO = 3,
            ///<summary>  | </summary>
            MAG_CAL_SUCCESS = 4,
            ///<summary>  | </summary>
            MAG_CAL_FAILED = 5,
            ///<summary>  | </summary>
            ENUM_END = 6,

        };

        ///<summary> Special ACK block numbers control activation of dataflash log streaming </summary>
        public enum MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS
        {
            ///<summary> UAV to stop sending DataFlash blocks | </summary>
            MAV_REMOTE_LOG_DATA_BLOCK_STOP = 2147483645,
            ///<summary> UAV to start sending DataFlash blocks | </summary>
            MAV_REMOTE_LOG_DATA_BLOCK_START = 2147483646,
            ///<summary>  | </summary>
            ENUM_END = 2147483647,

        };

        ///<summary> Possible remote log data block statuses </summary>
        public enum MAV_REMOTE_LOG_DATA_BLOCK_STATUSES
        {
            ///<summary> This block has NOT been received | </summary>
            MAV_REMOTE_LOG_DATA_BLOCK_NACK = 0,
            ///<summary> This block has been received | </summary>
            MAV_REMOTE_LOG_DATA_BLOCK_ACK = 1,
            ///<summary>  | </summary>
            ENUM_END = 2,

        };



        ///<summary> Micro air vehicle / autopilot classes. This identifies the individual model. </summary>
        public enum MAV_AUTOPILOT
        {
            ///<summary> Generic autopilot, full support for everything | </summary>
            GENERIC = 0,
            ///<summary> Reserved for future use. | </summary>
            RESERVED = 1,
            ///<summary> SLUGS autopilot, http://slugsuav.soe.ucsc.edu | </summary>
            SLUGS = 2,
            ///<summary> ArduPilotMega / ArduCopter, http://diydrones.com | </summary>
            ARDUPILOTMEGA = 3,
            ///<summary> OpenPilot, http://openpilot.org | </summary>
            OPENPILOT = 4,
            ///<summary> Generic autopilot only supporting simple waypoints | </summary>
            GENERIC_WAYPOINTS_ONLY = 5,
            ///<summary> Generic autopilot supporting waypoints and other simple navigation commands | </summary>
            GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
            ///<summary> Generic autopilot supporting the full mission command set | </summary>
            GENERIC_MISSION_FULL = 7,
            ///<summary> No valid autopilot, e.g. a GCS or other MAVLink component | </summary>
            INVALID = 8,
            ///<summary> PPZ UAV - http://nongnu.org/paparazzi | </summary>
            PPZ = 9,
            ///<summary> UAV Dev Board | </summary>
            UDB = 10,
            ///<summary> FlexiPilot | </summary>
            FP = 11,
            ///<summary> PX4 Autopilot - http://pixhawk.ethz.ch/px4/ | </summary>
            PX4 = 12,
            ///<summary> SMACCMPilot - http://smaccmpilot.org | </summary>
            SMACCMPILOT = 13,
            ///<summary> AutoQuad -- http://autoquad.org | </summary>
            AUTOQUAD = 14,
            ///<summary> Armazila -- http://armazila.com | </summary>
            ARMAZILA = 15,
            ///<summary> Aerob -- http://aerob.ru | </summary>
            AEROB = 16,
            ///<summary> ASLUAV autopilot -- http://www.asl.ethz.ch | </summary>
            ASLUAV = 17,
            ///<summary>  | </summary>
            ENUM_END = 18,

        };

        ///<summary>  </summary>
        public enum MAV_TYPE
        {
            ///<summary> Generic micro air vehicle. | </summary>
            GENERIC = 0,
            ///<summary> Fixed wing aircraft. | </summary>
            FIXED_WING = 1,
            ///<summary> Quadrotor | </summary>
            QUADROTOR = 2,
            ///<summary> Coaxial helicopter | </summary>
            COAXIAL = 3,
            ///<summary> Normal helicopter with tail rotor. | </summary>
            HELICOPTER = 4,
            ///<summary> Ground installation | </summary>
            ANTENNA_TRACKER = 5,
            ///<summary> Operator control unit / ground control station | </summary>
            GCS = 6,
            ///<summary> Airship, controlled | </summary>
            AIRSHIP = 7,
            ///<summary> Free balloon, uncontrolled | </summary>
            FREE_BALLOON = 8,
            ///<summary> Rocket | </summary>
            ROCKET = 9,
            ///<summary> Ground rover | </summary>
            GROUND_ROVER = 10,
            ///<summary> Surface vessel, boat, ship | </summary>
            SURFACE_BOAT = 11,
            ///<summary> Submarine | </summary>
            SUBMARINE = 12,
            ///<summary> Hexarotor | </summary>
            HEXAROTOR = 13,
            ///<summary> Octorotor | </summary>
            OCTOROTOR = 14,
            ///<summary> Octorotor | </summary>
            TRICOPTER = 15,
            ///<summary> Flapping wing | </summary>
            FLAPPING_WING = 16,
            ///<summary> Flapping wing | </summary>
            KITE = 17,
            ///<summary> Onboard companion controller | </summary>
            ONBOARD_CONTROLLER = 18,
            ///<summary> Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | </summary>
            VTOL_DUOROTOR = 19,
            ///<summary> Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | </summary>
            VTOL_QUADROTOR = 20,
            ///<summary> Tiltrotor VTOL | </summary>
            VTOL_TILTROTOR = 21,
            ///<summary> VTOL reserved 2 | </summary>
            VTOL_RESERVED2 = 22,
            ///<summary> VTOL reserved 3 | </summary>
            VTOL_RESERVED3 = 23,
            ///<summary> VTOL reserved 4 | </summary>
            VTOL_RESERVED4 = 24,
            ///<summary> VTOL reserved 5 | </summary>
            VTOL_RESERVED5 = 25,
            ///<summary> Onboard gimbal | </summary>
            GIMBAL = 26,
            ///<summary> Onboard ADSB peripheral | </summary>
            ADSB = 27,
            ///<summary>  | </summary>
            ENUM_END = 28,

        };

        ///<summary> These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65. </summary>
        public enum FIRMWARE_VERSION_TYPE
        {
            ///<summary> development release | </summary>
            DEV = 0,
            ///<summary> alpha release | </summary>
            ALPHA = 64,
            ///<summary> beta release | </summary>
            BETA = 128,
            ///<summary> release candidate | </summary>
            RC = 192,
            ///<summary> official stable release | </summary>
            OFFICIAL = 255,
            ///<summary>  | </summary>
            ENUM_END = 256,

        };

        ///<summary> These flags encode the MAV mode. </summary>
        public enum MAV_MODE_FLAG
        {
            ///<summary> 0b00000001 Reserved for future use. | </summary>
            CUSTOM_MODE_ENABLED = 1,
            ///<summary> 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | </summary>
            TEST_ENABLED = 2,
            ///<summary> 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | </summary>
            AUTO_ENABLED = 4,
            ///<summary> 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | </summary>
            GUIDED_ENABLED = 8,
            ///<summary> 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | </summary>
            STABILIZE_ENABLED = 16,
            ///<summary> 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | </summary>
            HIL_ENABLED = 32,
            ///<summary> 0b01000000 remote control input is enabled. | </summary>
            MANUAL_INPUT_ENABLED = 64,
            ///<summary> 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | </summary>
            SAFETY_ARMED = 128,
            ///<summary>  | </summary>
            ENUM_END = 129,

        };

        ///<summary> These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not. </summary>
        public enum MAV_MODE_FLAG_DECODE_POSITION
        {
            ///<summary> Eighth bit: 00000001 | </summary>
            CUSTOM_MODE = 1,
            ///<summary> Seventh bit: 00000010 | </summary>
            TEST = 2,
            ///<summary> Sixt bit:   00000100 | </summary>
            AUTO = 4,
            ///<summary> Fifth bit:  00001000 | </summary>
            GUIDED = 8,
            ///<summary> Fourth bit: 00010000 | </summary>
            STABILIZE = 16,
            ///<summary> Third bit:  00100000 | </summary>
            HIL = 32,
            ///<summary> Second bit: 01000000 | </summary>
            MANUAL = 64,
            ///<summary> First bit:  10000000 | </summary>
            SAFETY = 128,
            ///<summary>  | </summary>
            ENUM_END = 129,

        };

        ///<summary> Override command, pauses current mission execution and moves immediately to a position </summary>
        public enum MAV_GOTO
        {
            ///<summary> Hold at the current position. | </summary>
            DO_HOLD = 0,
            ///<summary> Continue with the next item in mission execution. | </summary>
            DO_CONTINUE = 1,
            ///<summary> Hold at the current position of the system | </summary>
            HOLD_AT_CURRENT_POSITION = 2,
            ///<summary> Hold at the position specified in the parameters of the DO_HOLD action | </summary>
            HOLD_AT_SPECIFIED_POSITION = 3,
            ///<summary>  | </summary>
            ENUM_END = 4,

        };

        ///<summary> These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override. </summary>
        public enum MAV_MODE
        {
            ///<summary> System is not ready to fly, booting, calibrating, etc. No flag is set. | </summary>
            PREFLIGHT = 0,
            ///<summary> System is allowed to be active, under manual (RC) control, no stabilization | </summary>
            MANUAL_DISARMED = 64,
            ///<summary> UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | </summary>
            TEST_DISARMED = 66,
            ///<summary> System is allowed to be active, under assisted RC control. | </summary>
            STABILIZE_DISARMED = 80,
            ///<summary> System is allowed to be active, under autonomous control, manual setpoint | </summary>
            GUIDED_DISARMED = 88,
            ///<summary> System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | </summary>
            AUTO_DISARMED = 92,
            ///<summary> System is allowed to be active, under manual (RC) control, no stabilization | </summary>
            MANUAL_ARMED = 192,
            ///<summary> UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | </summary>
            TEST_ARMED = 194,
            ///<summary> System is allowed to be active, under assisted RC control. | </summary>
            STABILIZE_ARMED = 208,
            ///<summary> System is allowed to be active, under autonomous control, manual setpoint | </summary>
            GUIDED_ARMED = 216,
            ///<summary> System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | </summary>
            AUTO_ARMED = 220,
            ///<summary>  | </summary>
            ENUM_END = 221,

        };

        ///<summary>  </summary>
        public enum MAV_STATE
        {
            ///<summary> Uninitialized system, state is unknown. | </summary>
            UNINIT = 0,
            ///<summary> System is booting up. | </summary>
            BOOT = 1,
            ///<summary> System is calibrating and not flight-ready. | </summary>
            CALIBRATING = 2,
            ///<summary> System is grounded and on standby. It can be launched any time. | </summary>
            STANDBY = 3,
            ///<summary> System is active and might be already airborne. Motors are engaged. | </summary>
            ACTIVE = 4,
            ///<summary> System is in a non-normal flight mode. It can however still navigate. | </summary>
            CRITICAL = 5,
            ///<summary> System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | </summary>
            EMERGENCY = 6,
            ///<summary> System just initialized its power-down sequence, will shut down now. | </summary>
            POWEROFF = 7,
            ///<summary>  | </summary>
            ENUM_END = 8,

        };

        ///<summary>  </summary>
        public enum MAV_COMPONENT
        {
            ///<summary>  | </summary>
            MAV_COMP_ID_ALL = 0,
            ///<summary>  | </summary>
            MAV_COMP_ID_CAMERA = 100,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO1 = 140,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO2 = 141,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO3 = 142,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO4 = 143,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO5 = 144,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO6 = 145,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO7 = 146,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO8 = 147,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO9 = 148,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO10 = 149,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO11 = 150,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO12 = 151,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO13 = 152,
            ///<summary>  | </summary>
            MAV_COMP_ID_SERVO14 = 153,
            ///<summary>  | </summary>
            MAV_COMP_ID_GIMBAL = 154,
            ///<summary>  | </summary>
            MAV_COMP_ID_LOG = 155,
            ///<summary>  | </summary>
            MAV_COMP_ID_ADSB = 156,
            ///<summary>  | </summary>
            MAV_COMP_ID_MAPPER = 180,
            ///<summary>  | </summary>
            MAV_COMP_ID_MISSIONPLANNER = 190,
            ///<summary>  | </summary>
            MAV_COMP_ID_PATHPLANNER = 195,
            ///<summary>  | </summary>
            MAV_COMP_ID_IMU = 200,
            ///<summary>  | </summary>
            MAV_COMP_ID_IMU_2 = 201,
            ///<summary>  | </summary>
            MAV_COMP_ID_IMU_3 = 202,
            ///<summary>  | </summary>
            MAV_COMP_ID_GPS = 220,
            ///<summary>  | </summary>
            MAV_COMP_ID_UDP_BRIDGE = 240,
            ///<summary>  | </summary>
            MAV_COMP_ID_UART_BRIDGE = 241,
            ///<summary>  | </summary>
            MAV_COMP_ID_SYSTEM_CONTROL = 250,
            ///<summary>  | </summary>
            ENUM_END = 251,

        };

        ///<summary> These encode the sensors whose status is sent as part of the SYS_STATUS message. </summary>
        public enum MAV_SYS_STATUS_SENSOR
        {
            ///<summary> 0x01 3D gyro | </summary>
            _3D_GYRO = 1,
            ///<summary> 0x02 3D accelerometer | </summary>
            _3D_ACCEL = 2,
            ///<summary> 0x04 3D magnetometer | </summary>
            _3D_MAG = 4,
            ///<summary> 0x08 absolute pressure | </summary>
            ABSOLUTE_PRESSURE = 8,
            ///<summary> 0x10 differential pressure | </summary>
            DIFFERENTIAL_PRESSURE = 16,
            ///<summary> 0x20 GPS | </summary>
            GPS = 32,
            ///<summary> 0x40 optical flow | </summary>
            OPTICAL_FLOW = 64,
            ///<summary> 0x80 computer vision position | </summary>
            VISION_POSITION = 128,
            ///<summary> 0x100 laser based position | </summary>
            LASER_POSITION = 256,
            ///<summary> 0x200 external ground truth (Vicon or Leica) | </summary>
            EXTERNAL_GROUND_TRUTH = 512,
            ///<summary> 0x400 3D angular rate control | </summary>
            ANGULAR_RATE_CONTROL = 1024,
            ///<summary> 0x800 attitude stabilization | </summary>
            ATTITUDE_STABILIZATION = 2048,
            ///<summary> 0x1000 yaw position | </summary>
            YAW_POSITION = 4096,
            ///<summary> 0x2000 z/altitude control | </summary>
            Z_ALTITUDE_CONTROL = 8192,
            ///<summary> 0x4000 x/y position control | </summary>
            XY_POSITION_CONTROL = 16384,
            ///<summary> 0x8000 motor outputs / control | </summary>
            MOTOR_OUTPUTS = 32768,
            ///<summary> 0x10000 rc receiver | </summary>
            RC_RECEIVER = 65536,
            ///<summary> 0x20000 2nd 3D gyro | </summary>
            _3D_GYRO2 = 131072,
            ///<summary> 0x40000 2nd 3D accelerometer | </summary>
            _3D_ACCEL2 = 262144,
            ///<summary> 0x80000 2nd 3D magnetometer | </summary>
            _3D_MAG2 = 524288,
            ///<summary> 0x100000 geofence | </summary>
            MAV_SYS_STATUS_GEOFENCE = 1048576,
            ///<summary> 0x200000 AHRS subsystem health | </summary>
            MAV_SYS_STATUS_AHRS = 2097152,
            ///<summary> 0x400000 Terrain subsystem health | </summary>
            MAV_SYS_STATUS_TERRAIN = 4194304,
            ///<summary> 0x800000 Motors are reversed | </summary>
            MAV_SYS_STATUS_REVERSE_MOTOR = 8388608,
            ///<summary>  | </summary>
            ENUM_END = 8388609,

        };

        ///<summary>  </summary>
        public enum MAV_FRAME
        {
            ///<summary> Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | </summary>
            GLOBAL = 0,
            ///<summary> Local coordinate frame, Z-up (x: north, y: east, z: down). | </summary>
            LOCAL_NED = 1,
            ///<summary> NOT a coordinate frame, indicates a mission command. | </summary>
            MISSION = 2,
            ///<summary> Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | </summary>
            GLOBAL_RELATIVE_ALT = 3,
            ///<summary> Local coordinate frame, Z-down (x: east, y: north, z: up) | </summary>
            LOCAL_ENU = 4,
            ///<summary> Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | </summary>
            GLOBAL_INT = 5,
            ///<summary> Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | </summary>
            GLOBAL_RELATIVE_ALT_INT = 6,
            ///<summary> Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | </summary>
            LOCAL_OFFSET_NED = 7,
            ///<summary> Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | </summary>
            BODY_NED = 8,
            ///<summary> Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | </summary>
            BODY_OFFSET_NED = 9,
            ///<summary> Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | </summary>
            GLOBAL_TERRAIN_ALT = 10,
            ///<summary> Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | </summary>
            GLOBAL_TERRAIN_ALT_INT = 11,
            ///<summary>  | </summary>
            ENUM_END = 12,

        };

        ///<summary>  </summary>
        public enum MAVLINK_DATA_STREAM_TYPE
        {
            ///<summary>  | </summary>
            MAVLINK_DATA_STREAM_IMG_JPEG = 1,
            ///<summary>  | </summary>
            MAVLINK_DATA_STREAM_IMG_BMP = 2,
            ///<summary>  | </summary>
            MAVLINK_DATA_STREAM_IMG_RAW8U = 3,
            ///<summary>  | </summary>
            MAVLINK_DATA_STREAM_IMG_RAW32U = 4,
            ///<summary>  | </summary>
            MAVLINK_DATA_STREAM_IMG_PGM = 5,
            ///<summary>  | </summary>
            MAVLINK_DATA_STREAM_IMG_PNG = 6,
            ///<summary>  | </summary>
            ENUM_END = 7,

        };

        ///<summary>  </summary>
        public enum FENCE_ACTION
        {
            ///<summary> Disable fenced mode | </summary>
            NONE = 0,
            ///<summary> Switched to guided mode to return point (fence point 0) | </summary>
            GUIDED = 1,
            ///<summary> Report fence breach, but don't take action | </summary>
            REPORT = 2,
            ///<summary> Switched to guided mode to return point (fence point 0) with manual throttle control | </summary>
            GUIDED_THR_PASS = 3,
            ///<summary>  | </summary>
            ENUM_END = 4,

        };

        ///<summary>  </summary>
        public enum FENCE_BREACH
        {
            ///<summary> No last fence breach | </summary>
            NONE = 0,
            ///<summary> Breached minimum altitude | </summary>
            MINALT = 1,
            ///<summary> Breached maximum altitude | </summary>
            MAXALT = 2,
            ///<summary> Breached fence boundary | </summary>
            BOUNDARY = 3,
            ///<summary>  | </summary>
            ENUM_END = 4,

        };

        ///<summary> Enumeration of possible mount operation modes </summary>
        public enum MAV_MOUNT_MODE
        {
            ///<summary> Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization | </summary>
            RETRACT = 0,
            ///<summary> Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | </summary>
            NEUTRAL = 1,
            ///<summary> Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | </summary>
            MAVLINK_TARGETING = 2,
            ///<summary> Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | </summary>
            RC_TARGETING = 3,
            ///<summary> Load neutral position and start to point to Lat,Lon,Alt | </summary>
            GPS_POINT = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary> THIS INTERFACE IS DEPRECATED AS OF JULY 2015. Please use MESSAGE_INTERVAL instead. A data stream is not a fixed set of messages, but rather a      recommendation to the autopilot software. Individual autopilots may or may not obey      the recommended messages. </summary>
        public enum MAV_DATA_STREAM
        {
            ///<summary> Enable all data streams | </summary>
            ALL = 0,
            ///<summary> Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | </summary>
            RAW_SENSORS = 1,
            ///<summary> Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | </summary>
            EXTENDED_STATUS = 2,
            ///<summary> Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | </summary>
            RC_CHANNELS = 3,
            ///<summary> Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | </summary>
            RAW_CONTROLLER = 4,
            ///<summary> Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. | </summary>
            POSITION = 6,
            ///<summary> Dependent on the autopilot | </summary>
            EXTRA1 = 10,
            ///<summary> Dependent on the autopilot | </summary>
            EXTRA2 = 11,
            ///<summary> Dependent on the autopilot | </summary>
            EXTRA3 = 12,
            ///<summary>  | </summary>
            ENUM_END = 13,

        };

        ///<summary>  The ROI (region of interest) for the vehicle. This can be                 be used by the vehicle for camera/vehicle attitude alignment (see                 MAV_CMD_NAV_ROI). </summary>
        public enum MAV_ROI
        {
            ///<summary> No region of interest. | </summary>
            NONE = 0,
            ///<summary> Point toward next MISSION. | </summary>
            WPNEXT = 1,
            ///<summary> Point toward given MISSION. | </summary>
            WPINDEX = 2,
            ///<summary> Point toward fixed location. | </summary>
            LOCATION = 3,
            ///<summary> Point toward of given id. | </summary>
            TARGET = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary> ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission. </summary>
        public enum MAV_CMD_ACK
        {
            ///<summary> Command / mission item is ok. | </summary>
            OK = 1,
            ///<summary> Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | </summary>
            ERR_FAIL = 2,
            ///<summary> The system is refusing to accept this command from this source / communication partner. | </summary>
            ERR_ACCESS_DENIED = 3,
            ///<summary> Command or mission item is not supported, other commands would be accepted. | </summary>
            ERR_NOT_SUPPORTED = 4,
            ///<summary> The coordinate frame of this command / mission item is not supported. | </summary>
            ERR_COORDINATE_FRAME_NOT_SUPPORTED = 5,
            ///<summary> The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible. | </summary>
            ERR_COORDINATES_OUT_OF_RANGE = 6,
            ///<summary> The X or latitude value is out of range. | </summary>
            ERR_X_LAT_OUT_OF_RANGE = 7,
            ///<summary> The Y or longitude value is out of range. | </summary>
            ERR_Y_LON_OUT_OF_RANGE = 8,
            ///<summary> The Z or altitude value is out of range. | </summary>
            ERR_Z_ALT_OUT_OF_RANGE = 9,
            ///<summary>  | </summary>
            ENUM_END = 10,

        };

        ///<summary> Specifies the datatype of a MAVLink parameter. </summary>
        public enum MAV_PARAM_TYPE
        {
            ///<summary> 8-bit unsigned integer | </summary>
            UINT8 = 1,
            ///<summary> 8-bit signed integer | </summary>
            INT8 = 2,
            ///<summary> 16-bit unsigned integer | </summary>
            UINT16 = 3,
            ///<summary> 16-bit signed integer | </summary>
            INT16 = 4,
            ///<summary> 32-bit unsigned integer | </summary>
            UINT32 = 5,
            ///<summary> 32-bit signed integer | </summary>
            INT32 = 6,
            ///<summary> 64-bit unsigned integer | </summary>
            UINT64 = 7,
            ///<summary> 64-bit signed integer | </summary>
            INT64 = 8,
            ///<summary> 32-bit floating-point | </summary>
            REAL32 = 9,
            ///<summary> 64-bit floating-point | </summary>
            REAL64 = 10,
            ///<summary>  | </summary>
            ENUM_END = 11,

        };

        ///<summary> result from a mavlink command </summary>
        public enum MAV_RESULT
        {
            ///<summary> Command ACCEPTED and EXECUTED | </summary>
            ACCEPTED = 0,
            ///<summary> Command TEMPORARY REJECTED/DENIED | </summary>
            TEMPORARILY_REJECTED = 1,
            ///<summary> Command PERMANENTLY DENIED | </summary>
            DENIED = 2,
            ///<summary> Command UNKNOWN/UNSUPPORTED | </summary>
            UNSUPPORTED = 3,
            ///<summary> Command executed, but failed | </summary>
            FAILED = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary> result in a mavlink mission ack </summary>
        public enum MAV_MISSION_RESULT
        {
            ///<summary> mission accepted OK | </summary>
            MAV_MISSION_ACCEPTED = 0,
            ///<summary> generic error / not accepting mission commands at all right now | </summary>
            MAV_MISSION_ERROR = 1,
            ///<summary> coordinate frame is not supported | </summary>
            MAV_MISSION_UNSUPPORTED_FRAME = 2,
            ///<summary> command is not supported | </summary>
            MAV_MISSION_UNSUPPORTED = 3,
            ///<summary> mission item exceeds storage space | </summary>
            MAV_MISSION_NO_SPACE = 4,
            ///<summary> one of the parameters has an invalid value | </summary>
            MAV_MISSION_INVALID = 5,
            ///<summary> param1 has an invalid value | </summary>
            MAV_MISSION_INVALID_PARAM1 = 6,
            ///<summary> param2 has an invalid value | </summary>
            MAV_MISSION_INVALID_PARAM2 = 7,
            ///<summary> param3 has an invalid value | </summary>
            MAV_MISSION_INVALID_PARAM3 = 8,
            ///<summary> param4 has an invalid value | </summary>
            MAV_MISSION_INVALID_PARAM4 = 9,
            ///<summary> x/param5 has an invalid value | </summary>
            MAV_MISSION_INVALID_PARAM5_X = 10,
            ///<summary> y/param6 has an invalid value | </summary>
            MAV_MISSION_INVALID_PARAM6_Y = 11,
            ///<summary> param7 has an invalid value | </summary>
            MAV_MISSION_INVALID_PARAM7 = 12,
            ///<summary> received waypoint out of sequence | </summary>
            MAV_MISSION_INVALID_SEQUENCE = 13,
            ///<summary> not accepting any mission commands from this communication partner | </summary>
            MAV_MISSION_DENIED = 14,
            ///<summary>  | </summary>
            ENUM_END = 15,

        };

        ///<summary> Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/. </summary>
        public enum MAV_SEVERITY
        {
            ///<summary> System is unusable. This is a "panic" condition. | </summary>
            EMERGENCY = 0,
            ///<summary> Action should be taken immediately. Indicates error in non-critical systems. | </summary>
            ALERT = 1,
            ///<summary> Action must be taken immediately. Indicates failure in a primary system. | </summary>
            CRITICAL = 2,
            ///<summary> Indicates an error in secondary/redundant systems. | </summary>
            ERROR = 3,
            ///<summary> Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | </summary>
            WARNING = 4,
            ///<summary> An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | </summary>
            NOTICE = 5,
            ///<summary> Normal operational messages. Useful for logging. No action is required for these messages. | </summary>
            INFO = 6,
            ///<summary> Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | </summary>
            DEBUG = 7,
            ///<summary>  | </summary>
            ENUM_END = 8,

        };

        ///<summary> Power supply status flags (bitmask) </summary>
        public enum MAV_POWER_STATUS
        {
            ///<summary> main brick power supply valid | </summary>
            BRICK_VALID = 1,
            ///<summary> main servo power supply valid for FMU | </summary>
            SERVO_VALID = 2,
            ///<summary> USB power is connected | </summary>
            USB_CONNECTED = 4,
            ///<summary> peripheral supply is in over-current state | </summary>
            PERIPH_OVERCURRENT = 8,
            ///<summary> hi-power peripheral supply is in over-current state | </summary>
            PERIPH_HIPOWER_OVERCURRENT = 16,
            ///<summary> Power status has changed since boot | </summary>
            CHANGED = 32,
            ///<summary>  | </summary>
            ENUM_END = 33,

        };

        ///<summary> SERIAL_CONTROL device types </summary>
        public enum SERIAL_CONTROL_DEV
        {
            ///<summary> First telemetry port | </summary>
            TELEM1 = 0,
            ///<summary> Second telemetry port | </summary>
            TELEM2 = 1,
            ///<summary> First GPS port | </summary>
            GPS1 = 2,
            ///<summary> Second GPS port | </summary>
            GPS2 = 3,
            ///<summary> system shell | </summary>
            SHELL = 10,
            ///<summary>  | </summary>
            ENUM_END = 11,

        };

        ///<summary> SERIAL_CONTROL flags (bitmask) </summary>
        public enum SERIAL_CONTROL_FLAG
        {
            ///<summary> Set if this is a reply | </summary>
            REPLY = 1,
            ///<summary> Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message | </summary>
            RESPOND = 2,
            ///<summary> Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set | </summary>
            EXCLUSIVE = 4,
            ///<summary> Block on writes to the serial port | </summary>
            BLOCKING = 8,
            ///<summary> Send multiple replies until port is drained | </summary>
            MULTI = 16,
            ///<summary>  | </summary>
            ENUM_END = 17,

        };

        ///<summary> Enumeration of distance sensor types </summary>
        public enum MAV_DISTANCE_SENSOR
        {
            ///<summary> Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units | </summary>
            LASER = 0,
            ///<summary> Ultrasound rangefinder, e.g. MaxBotix units | </summary>
            ULTRASOUND = 1,
            ///<summary> Infrared rangefinder, e.g. Sharp units | </summary>
            INFRARED = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary> Enumeration of sensor orientation, according to its rotations </summary>
        public enum MAV_SENSOR_ORIENTATION
        {
            ///<summary> Roll: 0, Pitch: 0, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_NONE = 0,
            ///<summary> Roll: 0, Pitch: 0, Yaw: 45 | </summary>
            MAV_SENSOR_ROTATION_YAW_45 = 1,
            ///<summary> Roll: 0, Pitch: 0, Yaw: 90 | </summary>
            MAV_SENSOR_ROTATION_YAW_90 = 2,
            ///<summary> Roll: 0, Pitch: 0, Yaw: 135 | </summary>
            MAV_SENSOR_ROTATION_YAW_135 = 3,
            ///<summary> Roll: 0, Pitch: 0, Yaw: 180 | </summary>
            MAV_SENSOR_ROTATION_YAW_180 = 4,
            ///<summary> Roll: 0, Pitch: 0, Yaw: 225 | </summary>
            MAV_SENSOR_ROTATION_YAW_225 = 5,
            ///<summary> Roll: 0, Pitch: 0, Yaw: 270 | </summary>
            MAV_SENSOR_ROTATION_YAW_270 = 6,
            ///<summary> Roll: 0, Pitch: 0, Yaw: 315 | </summary>
            MAV_SENSOR_ROTATION_YAW_315 = 7,
            ///<summary> Roll: 180, Pitch: 0, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_180 = 8,
            ///<summary> Roll: 180, Pitch: 0, Yaw: 45 | </summary>
            MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9,
            ///<summary> Roll: 180, Pitch: 0, Yaw: 90 | </summary>
            MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10,
            ///<summary> Roll: 180, Pitch: 0, Yaw: 135 | </summary>
            MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11,
            ///<summary> Roll: 0, Pitch: 180, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_PITCH_180 = 12,
            ///<summary> Roll: 180, Pitch: 0, Yaw: 225 | </summary>
            MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13,
            ///<summary> Roll: 180, Pitch: 0, Yaw: 270 | </summary>
            MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14,
            ///<summary> Roll: 180, Pitch: 0, Yaw: 315 | </summary>
            MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15,
            ///<summary> Roll: 90, Pitch: 0, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_90 = 16,
            ///<summary> Roll: 90, Pitch: 0, Yaw: 45 | </summary>
            MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17,
            ///<summary> Roll: 90, Pitch: 0, Yaw: 90 | </summary>
            MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18,
            ///<summary> Roll: 90, Pitch: 0, Yaw: 135 | </summary>
            MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19,
            ///<summary> Roll: 270, Pitch: 0, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_270 = 20,
            ///<summary> Roll: 270, Pitch: 0, Yaw: 45 | </summary>
            MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21,
            ///<summary> Roll: 270, Pitch: 0, Yaw: 90 | </summary>
            MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22,
            ///<summary> Roll: 270, Pitch: 0, Yaw: 135 | </summary>
            MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23,
            ///<summary> Roll: 0, Pitch: 90, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_PITCH_90 = 24,
            ///<summary> Roll: 0, Pitch: 270, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_PITCH_270 = 25,
            ///<summary> Roll: 0, Pitch: 180, Yaw: 90 | </summary>
            MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26,
            ///<summary> Roll: 0, Pitch: 180, Yaw: 270 | </summary>
            MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27,
            ///<summary> Roll: 90, Pitch: 90, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28,
            ///<summary> Roll: 180, Pitch: 90, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29,
            ///<summary> Roll: 270, Pitch: 90, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30,
            ///<summary> Roll: 90, Pitch: 180, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31,
            ///<summary> Roll: 270, Pitch: 180, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32,
            ///<summary> Roll: 90, Pitch: 270, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33,
            ///<summary> Roll: 180, Pitch: 270, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34,
            ///<summary> Roll: 270, Pitch: 270, Yaw: 0 | </summary>
            MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35,
            ///<summary> Roll: 90, Pitch: 180, Yaw: 90 | </summary>
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
            ///<summary> Roll: 90, Pitch: 0, Yaw: 270 | </summary>
            MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37,
            ///<summary> Roll: 315, Pitch: 315, Yaw: 315 | </summary>
            MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38,
            ///<summary>  | </summary>
            ENUM_END = 39,

        };

        ///<summary> Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability. </summary>
        public enum MAV_PROTOCOL_CAPABILITY
        {
            ///<summary> Autopilot supports MISSION float message type. | </summary>
            MISSION_FLOAT = 1,
            ///<summary> Autopilot supports the new param float message type. | </summary>
            PARAM_FLOAT = 2,
            ///<summary> Autopilot supports MISSION_INT scaled integer message type. | </summary>
            MISSION_INT = 4,
            ///<summary> Autopilot supports COMMAND_INT scaled integer message type. | </summary>
            COMMAND_INT = 8,
            ///<summary> Autopilot supports the new param union message type. | </summary>
            PARAM_UNION = 16,
            ///<summary> Autopilot supports the new FILE_TRANSFER_PROTOCOL message type. | </summary>
            FTP = 32,
            ///<summary> Autopilot supports commanding attitude offboard. | </summary>
            SET_ATTITUDE_TARGET = 64,
            ///<summary> Autopilot supports commanding position and velocity targets in local NED frame. | </summary>
            SET_POSITION_TARGET_LOCAL_NED = 128,
            ///<summary> Autopilot supports commanding position and velocity targets in global scaled integers. | </summary>
            SET_POSITION_TARGET_GLOBAL_INT = 256,
            ///<summary> Autopilot supports terrain protocol / data handling. | </summary>
            TERRAIN = 512,
            ///<summary> Autopilot supports direct actuator control. | </summary>
            SET_ACTUATOR_TARGET = 1024,
            ///<summary> Autopilot supports the flight termination command. | </summary>
            FLIGHT_TERMINATION = 2048,
            ///<summary> Autopilot supports onboard compass calibration. | </summary>
            COMPASS_CALIBRATION = 4096,
            ///<summary>  | </summary>
            ENUM_END = 4097,

        };

        ///<summary> Enumeration of estimator types </summary>
        public enum MAV_ESTIMATOR_TYPE
        {
            ///<summary> This is a naive estimator without any real covariance feedback. | </summary>
            NAIVE = 1,
            ///<summary> Computer vision based estimate. Might be up to scale. | </summary>
            VISION = 2,
            ///<summary> Visual-inertial estimate. | </summary>
            VIO = 3,
            ///<summary> Plain GPS estimate. | </summary>
            GPS = 4,
            ///<summary> Estimator integrating GPS and inertial sensing. | </summary>
            GPS_INS = 5,
            ///<summary>  | </summary>
            ENUM_END = 6,

        };

        ///<summary> Enumeration of battery types </summary>
        public enum MAV_BATTERY_TYPE
        {
            ///<summary> Not specified. | </summary>
            UNKNOWN = 0,
            ///<summary> Lithium polymer battery | </summary>
            LIPO = 1,
            ///<summary> Lithium-iron-phosphate battery | </summary>
            LIFE = 2,
            ///<summary> Lithium-ION battery | </summary>
            LION = 3,
            ///<summary> Nickel metal hydride battery | </summary>
            NIMH = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary> Enumeration of battery functions </summary>
        public enum MAV_BATTERY_FUNCTION
        {
            ///<summary> Battery function is unknown | </summary>
            UNKNOWN = 0,
            ///<summary> Battery supports all flight systems | </summary>
            ALL = 1,
            ///<summary> Battery for the propulsion system | </summary>
            PROPULSION = 2,
            ///<summary> Avionics battery | </summary>
            AVIONICS = 3,
            ///<summary> Payload battery | </summary>
            MAV_BATTERY_TYPE_PAYLOAD = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary> Enumeration of VTOL states </summary>
        public enum MAV_VTOL_STATE
        {
            ///<summary> MAV is not configured as VTOL | </summary>
            UNDEFINED = 0,
            ///<summary> VTOL is in transition from multicopter to fixed-wing | </summary>
            TRANSITION_TO_FW = 1,
            ///<summary> VTOL is in transition from fixed-wing to multicopter | </summary>
            TRANSITION_TO_MC = 2,
            ///<summary> VTOL is in multicopter state | </summary>
            MC = 3,
            ///<summary> VTOL is in fixed-wing state | </summary>
            FW = 4,
            ///<summary>  | </summary>
            ENUM_END = 5,

        };

        ///<summary> Enumeration of landed detector states </summary>
        public enum MAV_LANDED_STATE
        {
            ///<summary> MAV landed state is unknown | </summary>
            UNDEFINED = 0,
            ///<summary> MAV is landed (on ground) | </summary>
            ON_GROUND = 1,
            ///<summary> MAV is in air | </summary>
            IN_AIR = 2,
            ///<summary>  | </summary>
            ENUM_END = 3,

        };

        ///<summary> Enumeration of the ADSB altimeter types </summary>
        public enum ADSB_ALTITUDE_TYPE
        {
            ///<summary> Altitude reported from a Baro source using QNH reference | </summary>
            PRESSURE_QNH = 0,
            ///<summary> Altitude reported from a GNSS source | </summary>
            GEOMETRIC = 1,
            ///<summary>  | </summary>
            ENUM_END = 2,

        };

        ///<summary> ADSB classification for the type of vehicle emitting the transponder signal </summary>
        public enum ADSB_EMITTER_TYPE
        {
            ///<summary>  | </summary>
            NO_INFO = 0,
            ///<summary>  | </summary>
            LIGHT = 1,
            ///<summary>  | </summary>
            SMALL = 2,
            ///<summary>  | </summary>
            LARGE = 3,
            ///<summary>  | </summary>
            HIGH_VORTEX_LARGE = 4,
            ///<summary>  | </summary>
            HEAVY = 5,
            ///<summary>  | </summary>
            HIGHLY_MANUV = 6,
            ///<summary>  | </summary>
            ROTOCRAFT = 7,
            ///<summary>  | </summary>
            UNASSIGNED = 8,
            ///<summary>  | </summary>
            GLIDER = 9,
            ///<summary>  | </summary>
            LIGHTER_AIR = 10,
            ///<summary>  | </summary>
            PARACHUTE = 11,
            ///<summary>  | </summary>
            ULTRA_LIGHT = 12,
            ///<summary>  | </summary>
            UNASSIGNED2 = 13,
            ///<summary>  | </summary>
            UAV = 14,
            ///<summary>  | </summary>
            SPACE = 15,
            ///<summary>  | </summary>
            UNASSGINED3 = 16,
            ///<summary>  | </summary>
            EMERGENCY_SURFACE = 17,
            ///<summary>  | </summary>
            SERVICE_SURFACE = 18,
            ///<summary>  | </summary>
            POINT_OBSTACLE = 19,
            ///<summary>  | </summary>
            ENUM_END = 20,

        };

        ///<summary> These flags indicate status such as data validity of each data source. Set = data valid </summary>
        public enum ADSB_FLAGS
        {
            ///<summary>  | </summary>
            VALID_COORDS = 1,
            ///<summary>  | </summary>
            VALID_ALTITUDE = 2,
            ///<summary>  | </summary>
            VALID_HEADING = 4,
            ///<summary>  | </summary>
            VALID_VELOCITY = 8,
            ///<summary>  | </summary>
            VALID_CALLSIGN = 16,
            ///<summary>  | </summary>
            VALID_SQUAWK = 32,
            ///<summary>  | </summary>
            SIMULATED = 64,
            ///<summary>  | </summary>
            ENUM_END = 65,

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_sensor_offsets_t
        {
            /// <summary> magnetic declination (radians) </summary>
            public Single mag_declination;
            /// <summary> raw pressure from barometer </summary>
            public Int32 raw_press;
            /// <summary> raw temperature from barometer </summary>
            public Int32 raw_temp;
            /// <summary> gyro X calibration </summary>
            public Single gyro_cal_x;
            /// <summary> gyro Y calibration </summary>
            public Single gyro_cal_y;
            /// <summary> gyro Z calibration </summary>
            public Single gyro_cal_z;
            /// <summary> accel X calibration </summary>
            public Single accel_cal_x;
            /// <summary> accel Y calibration </summary>
            public Single accel_cal_y;
            /// <summary> accel Z calibration </summary>
            public Single accel_cal_z;
            /// <summary> magnetometer X offset </summary>
            public Int16 mag_ofs_x;
            /// <summary> magnetometer Y offset </summary>
            public Int16 mag_ofs_y;
            /// <summary> magnetometer Z offset </summary>
            public Int16 mag_ofs_z;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_set_mag_offsets_t
        {
            /// <summary> magnetometer X offset </summary>
            public Int16 mag_ofs_x;
            /// <summary> magnetometer Y offset </summary>
            public Int16 mag_ofs_y;
            /// <summary> magnetometer Z offset </summary>
            public Int16 mag_ofs_z;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_meminfo_t
        {
            /// <summary> heap top </summary>
            public UInt16 brkval;
            /// <summary> free memory </summary>
            public UInt16 freemem;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_ap_adc_t
        {
            /// <summary> ADC output 1 </summary>
            public UInt16 adc1;
            /// <summary> ADC output 2 </summary>
            public UInt16 adc2;
            /// <summary> ADC output 3 </summary>
            public UInt16 adc3;
            /// <summary> ADC output 4 </summary>
            public UInt16 adc4;
            /// <summary> ADC output 5 </summary>
            public UInt16 adc5;
            /// <summary> ADC output 6 </summary>
            public UInt16 adc6;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_digicam_configure_t
        {
            /// <summary> Correspondent value to given extra_param </summary>
            public Single extra_value;
            /// <summary> Divisor number //e.g. 1000 means 1/1000 (0 means ignore) </summary>
            public UInt16 shutter_speed;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Mode enumeration from 1 to N //P, TV, AV, M, Etc (0 means ignore) </summary>
            public byte mode;
            /// <summary> F stop number x 10 //e.g. 28 means 2.8 (0 means ignore) </summary>
            public byte aperture;
            /// <summary> ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore) </summary>
            public byte iso;
            /// <summary> Exposure type enumeration from 1 to N (0 means ignore) </summary>
            public byte exposure_type;
            /// <summary> Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once </summary>
            public byte command_id;
            /// <summary> Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off) </summary>
            public byte engine_cut_off;
            /// <summary> Extra parameters enumeration (0 means ignore) </summary>
            public byte extra_param;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_digicam_control_t
        {
            /// <summary> Correspondent value to given extra_param </summary>
            public Single extra_value;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> 0: stop, 1: start or keep it up //Session control e.g. show/hide lens </summary>
            public byte session;
            /// <summary> 1 to N //Zoom's absolute position (0 means ignore) </summary>
            public byte zoom_pos;
            /// <summary> -100 to 100 //Zooming step value to offset zoom from the current position </summary>
            public byte zoom_step;
            /// <summary> 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus </summary>
            public byte focus_lock;
            /// <summary> 0: ignore, 1: shot or start filming </summary>
            public byte shot;
            /// <summary> Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once </summary>
            public byte command_id;
            /// <summary> Extra parameters enumeration (0 means ignore) </summary>
            public byte extra_param;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mount_configure_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> mount operating mode (see MAV_MOUNT_MODE enum) </summary>
            public byte mount_mode;
            /// <summary> (1 = yes, 0 = no) </summary>
            public byte stab_roll;
            /// <summary> (1 = yes, 0 = no) </summary>
            public byte stab_pitch;
            /// <summary> (1 = yes, 0 = no) </summary>
            public byte stab_yaw;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mount_control_t
        {
            /// <summary> pitch(deg*100) or lat, depending on mount mode </summary>
            public Int32 input_a;
            /// <summary> roll(deg*100) or lon depending on mount mode </summary>
            public Int32 input_b;
            /// <summary> yaw(deg*100) or alt (in cm) depending on mount mode </summary>
            public Int32 input_c;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING) </summary>
            public byte save_position;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mount_status_t
        {
            /// <summary> pitch(deg*100) </summary>
            public Int32 pointing_a;
            /// <summary> roll(deg*100) </summary>
            public Int32 pointing_b;
            /// <summary> yaw(deg*100) </summary>
            public Int32 pointing_c;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_fence_point_t
        {
            /// <summary> Latitude of point </summary>
            public Single lat;
            /// <summary> Longitude of point </summary>
            public Single lng;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> point index (first point is 1, 0 is for return point) </summary>
            public byte idx;
            /// <summary> total number of points (for sanity checking) </summary>
            public byte count;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_fence_fetch_point_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> point index (first point is 1, 0 is for return point) </summary>
            public byte idx;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_fence_status_t
        {
            /// <summary> time of last breach in milliseconds since boot </summary>
            public UInt32 breach_time;
            /// <summary> number of fence breaches </summary>
            public UInt16 breach_count;
            /// <summary> 0 if currently inside fence, 1 if outside </summary>
            public byte breach_status;
            /// <summary> last breach type (see FENCE_BREACH_* enum) </summary>
            public byte breach_type;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_ahrs_t
        {
            /// <summary> X gyro drift estimate rad/s </summary>
            public Single omegaIx;
            /// <summary> Y gyro drift estimate rad/s </summary>
            public Single omegaIy;
            /// <summary> Z gyro drift estimate rad/s </summary>
            public Single omegaIz;
            /// <summary> average accel_weight </summary>
            public Single accel_weight;
            /// <summary> average renormalisation value </summary>
            public Single renorm_val;
            /// <summary> average error_roll_pitch value </summary>
            public Single error_rp;
            /// <summary> average error_yaw value </summary>
            public Single error_yaw;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_simstate_t
        {
            /// <summary> Roll angle (rad) </summary>
            public Single roll;
            /// <summary> Pitch angle (rad) </summary>
            public Single pitch;
            /// <summary> Yaw angle (rad) </summary>
            public Single yaw;
            /// <summary> X acceleration m/s/s </summary>
            public Single xacc;
            /// <summary> Y acceleration m/s/s </summary>
            public Single yacc;
            /// <summary> Z acceleration m/s/s </summary>
            public Single zacc;
            /// <summary> Angular speed around X axis rad/s </summary>
            public Single xgyro;
            /// <summary> Angular speed around Y axis rad/s </summary>
            public Single ygyro;
            /// <summary> Angular speed around Z axis rad/s </summary>
            public Single zgyro;
            /// <summary> Latitude in degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude in degrees * 1E7 </summary>
            public Int32 lng;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_hwstatus_t
        {
            /// <summary> board voltage (mV) </summary>
            public UInt16 Vcc;
            /// <summary> I2C error count </summary>
            public byte I2Cerr;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_radio_t
        {
            /// <summary> receive errors </summary>
            public UInt16 rxerrors;
            /// <summary> count of error corrected packets </summary>
            public UInt16 @fixed;
            /// <summary> local signal strength </summary>
            public byte rssi;
            /// <summary> remote signal strength </summary>
            public byte remrssi;
            /// <summary> how full the tx buffer is as a percentage </summary>
            public byte txbuf;
            /// <summary> background noise level </summary>
            public byte noise;
            /// <summary> remote background noise level </summary>
            public byte remnoise;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_limits_status_t
        {
            /// <summary> time of last breach in milliseconds since boot </summary>
            public UInt32 last_trigger;
            /// <summary> time of last recovery action in milliseconds since boot </summary>
            public UInt32 last_action;
            /// <summary> time of last successful recovery in milliseconds since boot </summary>
            public UInt32 last_recovery;
            /// <summary> time of last all-clear in milliseconds since boot </summary>
            public UInt32 last_clear;
            /// <summary> number of fence breaches </summary>
            public UInt16 breach_count;
            /// <summary> state of AP_Limits, (see enum LimitState, LIMITS_STATE) </summary>
            public byte limits_state;
            /// <summary> AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE) </summary>
            public byte mods_enabled;
            /// <summary> AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE) </summary>
            public byte mods_required;
            /// <summary> AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE) </summary>
            public byte mods_triggered;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_wind_t
        {
            /// <summary> wind direction that wind is coming from (degrees) </summary>
            public Single direction;
            /// <summary> wind speed in ground plane (m/s) </summary>
            public Single speed;
            /// <summary> vertical wind speed (m/s) </summary>
            public Single speed_z;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_data16_t
        {
            /// <summary> data type </summary>
            public byte type;
            /// <summary> data length </summary>
            public byte len;
            /// <summary> raw data </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] data;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_data32_t
        {
            /// <summary> data type </summary>
            public byte type;
            /// <summary> data length </summary>
            public byte len;
            /// <summary> raw data </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public byte[] data;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_data64_t
        {
            /// <summary> data type </summary>
            public byte type;
            /// <summary> data length </summary>
            public byte len;
            /// <summary> raw data </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)]
            public byte[] data;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_data96_t
        {
            /// <summary> data type </summary>
            public byte type;
            /// <summary> data length </summary>
            public byte len;
            /// <summary> raw data </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 96)]
            public byte[] data;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_rangefinder_t
        {
            /// <summary> distance in meters </summary>
            public Single distance;
            /// <summary> raw voltage if available, zero otherwise </summary>
            public Single voltage;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_airspeed_autocal_t
        {
            /// <summary> GPS velocity north m/s </summary>
            public Single vx;
            /// <summary> GPS velocity east m/s </summary>
            public Single vy;
            /// <summary> GPS velocity down m/s </summary>
            public Single vz;
            /// <summary> Differential pressure pascals </summary>
            public Single diff_pressure;
            /// <summary> Estimated to true airspeed ratio </summary>
            public Single EAS2TAS;
            /// <summary> Airspeed ratio </summary>
            public Single ratio;
            /// <summary> EKF state x </summary>
            public Single state_x;
            /// <summary> EKF state y </summary>
            public Single state_y;
            /// <summary> EKF state z </summary>
            public Single state_z;
            /// <summary> EKF Pax </summary>
            public Single Pax;
            /// <summary> EKF Pby </summary>
            public Single Pby;
            /// <summary> EKF Pcz </summary>
            public Single Pcz;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_rally_point_t
        {
            /// <summary> Latitude of point in degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude of point in degrees * 1E7 </summary>
            public Int32 lng;
            /// <summary> Transit / loiter altitude in meters relative to home </summary>
            public Int16 alt;
            /// <summary> Break altitude in meters relative to home </summary>
            public Int16 break_alt;
            /// <summary> Heading to aim for when landing. In centi-degrees. </summary>
            public UInt16 land_dir;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> point index (first point is 0) </summary>
            public byte idx;
            /// <summary> total number of points (for sanity checking) </summary>
            public byte count;
            /// <summary> See RALLY_FLAGS enum for definition of the bitmask. </summary>
            public byte flags;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_rally_fetch_point_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> point index (first point is 0) </summary>
            public byte idx;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_compassmot_status_t
        {
            /// <summary> current (amps) </summary>
            public Single current;
            /// <summary> Motor Compensation X </summary>
            public Single CompensationX;
            /// <summary> Motor Compensation Y </summary>
            public Single CompensationY;
            /// <summary> Motor Compensation Z </summary>
            public Single CompensationZ;
            /// <summary> throttle (percent*10) </summary>
            public UInt16 throttle;
            /// <summary> interference (percent) </summary>
            public UInt16 interference;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_ahrs2_t
        {
            /// <summary> Roll angle (rad) </summary>
            public Single roll;
            /// <summary> Pitch angle (rad) </summary>
            public Single pitch;
            /// <summary> Yaw angle (rad) </summary>
            public Single yaw;
            /// <summary> Altitude (MSL) </summary>
            public Single altitude;
            /// <summary> Latitude in degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude in degrees * 1E7 </summary>
            public Int32 lng;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_camera_status_t
        {
            /// <summary> Image timestamp (microseconds since UNIX epoch, according to camera clock) </summary>
            public UInt64 time_usec;
            /// <summary> Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum) </summary>
            public Single p1;
            /// <summary> Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum) </summary>
            public Single p2;
            /// <summary> Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum) </summary>
            public Single p3;
            /// <summary> Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum) </summary>
            public Single p4;
            /// <summary> Image index </summary>
            public UInt16 img_idx;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Camera ID </summary>
            public byte cam_idx;
            /// <summary> See CAMERA_STATUS_TYPES enum for definition of the bitmask </summary>
            public byte event_id;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_camera_feedback_t
        {
            /// <summary> Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB) </summary>
            public UInt64 time_usec;
            /// <summary> Latitude in (deg * 1E7) </summary>
            public Int32 lat;
            /// <summary> Longitude in (deg * 1E7) </summary>
            public Int32 lng;
            /// <summary> Altitude Absolute (meters AMSL) </summary>
            public Single alt_msl;
            /// <summary> Altitude Relative (meters above HOME location) </summary>
            public Single alt_rel;
            /// <summary> Camera Roll angle (earth frame, degrees, +-180) </summary>
            public Single roll;
            /// <summary> Camera Pitch angle (earth frame, degrees, +-180) </summary>
            public Single pitch;
            /// <summary> Camera Yaw (earth frame, degrees, 0-360, true) </summary>
            public Single yaw;
            /// <summary> Focal Length (mm) </summary>
            public Single foc_len;
            /// <summary> Image index </summary>
            public UInt16 img_idx;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Camera ID </summary>
            public byte cam_idx;
            /// <summary> See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask </summary>
            public byte flags;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_battery2_t
        {
            /// <summary> voltage in millivolts </summary>
            public UInt16 voltage;
            /// <summary> Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current </summary>
            public Int16 current_battery;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_ahrs3_t
        {
            /// <summary> Roll angle (rad) </summary>
            public Single roll;
            /// <summary> Pitch angle (rad) </summary>
            public Single pitch;
            /// <summary> Yaw angle (rad) </summary>
            public Single yaw;
            /// <summary> Altitude (MSL) </summary>
            public Single altitude;
            /// <summary> Latitude in degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude in degrees * 1E7 </summary>
            public Int32 lng;
            /// <summary> test variable1 </summary>
            public Single v1;
            /// <summary> test variable2 </summary>
            public Single v2;
            /// <summary> test variable3 </summary>
            public Single v3;
            /// <summary> test variable4 </summary>
            public Single v4;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_autopilot_version_request_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_remote_log_data_block_t
        {
            /// <summary> log data block sequence number </summary>
            public UInt32 seqno;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> log data block </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 200)]
            public byte[] data;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_remote_log_block_status_t
        {
            /// <summary> log data block sequence number </summary>
            public UInt32 seqno;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> log data block status </summary>
            public byte status;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_led_control_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Instance (LED instance to control or 255 for all LEDs) </summary>
            public byte instance;
            /// <summary> Pattern (see LED_PATTERN_ENUM) </summary>
            public byte pattern;
            /// <summary> Custom Byte Length </summary>
            public byte custom_len;
            /// <summary> Custom Bytes </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 24)]
            public byte[] custom_bytes;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mag_cal_progress_t
        {
            /// <summary> Body frame direction vector for display </summary>
            public Single direction_x;
            /// <summary> Body frame direction vector for display </summary>
            public Single direction_y;
            /// <summary> Body frame direction vector for display </summary>
            public Single direction_z;
            /// <summary> Compass being calibrated </summary>
            public byte compass_id;
            /// <summary> Bitmask of compasses being calibrated </summary>
            public byte cal_mask;
            /// <summary> Status (see MAG_CAL_STATUS enum) </summary>
            public byte cal_status;
            /// <summary> Attempt number </summary>
            public byte attempt;
            /// <summary> Completion percentage </summary>
            public byte completion_pct;
            /// <summary> Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
            public byte[] completion_mask;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mag_cal_report_t
        {
            /// <summary> RMS milligauss residuals </summary>
            public Single fitness;
            /// <summary> X offset </summary>
            public Single ofs_x;
            /// <summary> Y offset </summary>
            public Single ofs_y;
            /// <summary> Z offset </summary>
            public Single ofs_z;
            /// <summary> X diagonal (matrix 11) </summary>
            public Single diag_x;
            /// <summary> Y diagonal (matrix 22) </summary>
            public Single diag_y;
            /// <summary> Z diagonal (matrix 33) </summary>
            public Single diag_z;
            /// <summary> X off-diagonal (matrix 12 and 21) </summary>
            public Single offdiag_x;
            /// <summary> Y off-diagonal (matrix 13 and 31) </summary>
            public Single offdiag_y;
            /// <summary> Z off-diagonal (matrix 32 and 23) </summary>
            public Single offdiag_z;
            /// <summary> Compass being calibrated </summary>
            public byte compass_id;
            /// <summary> Bitmask of compasses being calibrated </summary>
            public byte cal_mask;
            /// <summary> Status (see MAG_CAL_STATUS enum) </summary>
            public byte cal_status;
            /// <summary> 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters </summary>
            public byte autosaved;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_ekf_status_report_t
        {
            /// <summary> Velocity variance </summary>
            public Single velocity_variance;
            /// <summary> Horizontal Position variance </summary>
            public Single pos_horiz_variance;
            /// <summary> Vertical Position variance </summary>
            public Single pos_vert_variance;
            /// <summary> Compass variance </summary>
            public Single compass_variance;
            /// <summary> Terrain Altitude variance </summary>
            public Single terrain_alt_variance;
            /// <summary> Flags </summary>
            public UInt16 flags;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_pid_tuning_t
        {
            /// <summary> desired rate (degrees/s) </summary>
            public Single desired;
            /// <summary> achieved rate (degrees/s) </summary>
            public Single achieved;
            /// <summary> FF component </summary>
            public Single FF;
            /// <summary> P component </summary>
            public Single P;
            /// <summary> I component </summary>
            public Single I;
            /// <summary> D component </summary>
            public Single D;
            /// <summary> axis </summary>
            public byte axis;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gimbal_report_t
        {
            /// <summary> Time since last update (seconds) </summary>
            public Single delta_time;
            /// <summary> Delta angle X (radians) </summary>
            public Single delta_angle_x;
            /// <summary> Delta angle Y (radians) </summary>
            public Single delta_angle_y;
            /// <summary> Delta angle X (radians) </summary>
            public Single delta_angle_z;
            /// <summary> Delta velocity X (m/s) </summary>
            public Single delta_velocity_x;
            /// <summary> Delta velocity Y (m/s) </summary>
            public Single delta_velocity_y;
            /// <summary> Delta velocity Z (m/s) </summary>
            public Single delta_velocity_z;
            /// <summary> Joint ROLL (radians) </summary>
            public Single joint_roll;
            /// <summary> Joint EL (radians) </summary>
            public Single joint_el;
            /// <summary> Joint AZ (radians) </summary>
            public Single joint_az;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gimbal_control_t
        {
            /// <summary> Demanded angular rate X (rad/s) </summary>
            public Single demanded_rate_x;
            /// <summary> Demanded angular rate Y (rad/s) </summary>
            public Single demanded_rate_y;
            /// <summary> Demanded angular rate Z (rad/s) </summary>
            public Single demanded_rate_z;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gimbal_torque_cmd_report_t
        {
            /// <summary> Roll Torque Command </summary>
            public Int16 rl_torque_cmd;
            /// <summary> Elevation Torque Command </summary>
            public Int16 el_torque_cmd;
            /// <summary> Azimuth Torque Command </summary>
            public Int16 az_torque_cmd;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gopro_heartbeat_t
        {
            /// <summary> Status </summary>
            public byte status;
            /// <summary> Current capture mode </summary>
            public byte capture_mode;
            /// <summary> additional status bits </summary>
            public byte flags;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gopro_get_request_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Command ID </summary>
            public byte cmd_id;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gopro_get_response_t
        {
            /// <summary> Command ID </summary>
            public byte cmd_id;
            /// <summary> Status </summary>
            public byte status;
            /// <summary> Value </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public byte[] value;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gopro_set_request_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Command ID </summary>
            public byte cmd_id;
            /// <summary> Value </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public byte[] value;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gopro_set_response_t
        {
            /// <summary> Command ID </summary>
            public byte cmd_id;
            /// <summary> Status </summary>
            public byte status;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_rpm_t
        {
            /// <summary> RPM Sensor1 </summary>
            public Single rpm1;
            /// <summary> RPM Sensor2 </summary>
            public Single rpm2;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_heartbeat_t
        {
            /// <summary> A bitfield for use for autopilot-specific flags. </summary>
            public UInt32 custom_mode;
            /// <summary> Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM) </summary>
            public byte type;
            /// <summary> Autopilot type / class. defined in MAV_AUTOPILOT ENUM </summary>
            public byte autopilot;
            /// <summary> System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h </summary>
            public byte base_mode;
            /// <summary> System status flag, see MAV_STATE ENUM </summary>
            public byte system_status;
            /// <summary> MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version </summary>
            public byte mavlink_version;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_sys_status_t
        {
            /// <summary> Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR </summary>
            public UInt32 onboard_control_sensors_present;
            /// <summary> Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR </summary>
            public UInt32 onboard_control_sensors_enabled;
            /// <summary> Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR </summary>
            public UInt32 onboard_control_sensors_health;
            /// <summary> Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000 </summary>
            public UInt16 load;
            /// <summary> Battery voltage, in millivolts (1 = 1 millivolt) </summary>
            public UInt16 voltage_battery;
            /// <summary> Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current </summary>
            public Int16 current_battery;
            /// <summary> Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV) </summary>
            public UInt16 drop_rate_comm;
            /// <summary> Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV) </summary>
            public UInt16 errors_comm;
            /// <summary> Autopilot-specific errors </summary>
            public UInt16 errors_count1;
            /// <summary> Autopilot-specific errors </summary>
            public UInt16 errors_count2;
            /// <summary> Autopilot-specific errors </summary>
            public UInt16 errors_count3;
            /// <summary> Autopilot-specific errors </summary>
            public UInt16 errors_count4;
            /// <summary> Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery </summary>
            public byte battery_remaining;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_system_time_t
        {
            /// <summary> Timestamp of the master clock in microseconds since UNIX epoch. </summary>
            public UInt64 time_unix_usec;
            /// <summary> Timestamp of the component clock since boot time in milliseconds. </summary>
            public UInt32 time_boot_ms;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_ping_t
        {
            /// <summary> Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009) </summary>
            public UInt64 time_usec;
            /// <summary> PING sequence </summary>
            public UInt32 seq;
            /// <summary> 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system </summary>
            public byte target_system;
            /// <summary> 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_change_operator_control_t
        {
            /// <summary> System the GCS requests control for </summary>
            public byte target_system;
            /// <summary> 0: request control of this MAV, 1: Release control of this MAV </summary>
            public byte control_request;
            /// <summary> 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch. </summary>
            public byte version;
            /// <summary> Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-" </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 25)]
            public byte[] passkey;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_change_operator_control_ack_t
        {
            /// <summary> ID of the GCS this message  </summary>
            public byte gcs_system_id;
            /// <summary> 0: request control of this MAV, 1: Release control of this MAV </summary>
            public byte control_request;
            /// <summary> 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control </summary>
            public byte ack;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_auth_key_t
        {
            /// <summary> key </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public byte[] key;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_set_mode_t
        {
            /// <summary> The new autopilot-specific mode. This field can be ignored by an autopilot. </summary>
            public UInt32 custom_mode;
            /// <summary> The system setting the mode </summary>
            public byte target_system;
            /// <summary> The new base mode </summary>
            public byte base_mode;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_param_request_read_t
        {
            /// <summary> Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored) </summary>
            public Int16 param_index;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] param_id;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_param_request_list_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_param_value_t
        {
            /// <summary> Onboard parameter value </summary>
            public Single param_value;
            /// <summary> Total number of onboard parameters </summary>
            public UInt16 param_count;
            /// <summary> Index of this onboard parameter </summary>
            public UInt16 param_index;
            /// <summary> Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] param_id;
            /// <summary> Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types. </summary>
            public byte param_type;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_param_set_t
        {
            /// <summary> Onboard parameter value </summary>
            public Single param_value;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] param_id;
            /// <summary> Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types. </summary>
            public byte param_type;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gps_raw_int_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> Latitude (WGS84), in degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude (WGS84), in degrees * 1E7 </summary>
            public Int32 lon;
            /// <summary> Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude. </summary>
            public Int32 alt;
            /// <summary> GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX </summary>
            public UInt16 eph;
            /// <summary> GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX </summary>
            public UInt16 epv;
            /// <summary> GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX </summary>
            public UInt16 vel;
            /// <summary> Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX </summary>
            public UInt16 cog;
            /// <summary> 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. </summary>
            public byte fix_type;
            /// <summary> Number of satellites visible. If unknown, set to 255 </summary>
            public byte satellites_visible;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gps_status_t
        {
            /// <summary> Number of satellites visible </summary>
            public byte satellites_visible;
            /// <summary> Global satellite ID </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 20)]
            public byte[] satellite_prn;
            /// <summary> 0: Satellite not used, 1: used for localization </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 20)]
            public byte[] satellite_used;
            /// <summary> Elevation (0: right on top of receiver, 90: on the horizon) of satellite </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 20)]
            public byte[] satellite_elevation;
            /// <summary> Direction of satellite, 0: 0 deg, 255: 360 deg. </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 20)]
            public byte[] satellite_azimuth;
            /// <summary> Signal to noise ratio of satellite </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 20)]
            public byte[] satellite_snr;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_scaled_imu_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> X acceleration (mg) </summary>
            public Int16 xacc;
            /// <summary> Y acceleration (mg) </summary>
            public Int16 yacc;
            /// <summary> Z acceleration (mg) </summary>
            public Int16 zacc;
            /// <summary> Angular speed around X axis (millirad /sec) </summary>
            public Int16 xgyro;
            /// <summary> Angular speed around Y axis (millirad /sec) </summary>
            public Int16 ygyro;
            /// <summary> Angular speed around Z axis (millirad /sec) </summary>
            public Int16 zgyro;
            /// <summary> X Magnetic field (milli tesla) </summary>
            public Int16 xmag;
            /// <summary> Y Magnetic field (milli tesla) </summary>
            public Int16 ymag;
            /// <summary> Z Magnetic field (milli tesla) </summary>
            public Int16 zmag;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_raw_imu_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> X acceleration (raw) </summary>
            public Int16 xacc;
            /// <summary> Y acceleration (raw) </summary>
            public Int16 yacc;
            /// <summary> Z acceleration (raw) </summary>
            public Int16 zacc;
            /// <summary> Angular speed around X axis (raw) </summary>
            public Int16 xgyro;
            /// <summary> Angular speed around Y axis (raw) </summary>
            public Int16 ygyro;
            /// <summary> Angular speed around Z axis (raw) </summary>
            public Int16 zgyro;
            /// <summary> X Magnetic field (raw) </summary>
            public Int16 xmag;
            /// <summary> Y Magnetic field (raw) </summary>
            public Int16 ymag;
            /// <summary> Z Magnetic field (raw) </summary>
            public Int16 zmag;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_raw_pressure_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> Absolute pressure (raw) </summary>
            public Int16 press_abs;
            /// <summary> Differential pressure 1 (raw, 0 if nonexistent) </summary>
            public Int16 press_diff1;
            /// <summary> Differential pressure 2 (raw, 0 if nonexistent) </summary>
            public Int16 press_diff2;
            /// <summary> Raw Temperature measurement (raw) </summary>
            public Int16 temperature;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_scaled_pressure_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Absolute pressure (hectopascal) </summary>
            public Single press_abs;
            /// <summary> Differential pressure 1 (hectopascal) </summary>
            public Single press_diff;
            /// <summary> Temperature measurement (0.01 degrees celsius) </summary>
            public Int16 temperature;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_attitude_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Roll angle (rad, -pi..+pi) </summary>
            public Single roll;
            /// <summary> Pitch angle (rad, -pi..+pi) </summary>
            public Single pitch;
            /// <summary> Yaw angle (rad, -pi..+pi) </summary>
            public Single yaw;
            /// <summary> Roll angular speed (rad/s) </summary>
            public Single rollspeed;
            /// <summary> Pitch angular speed (rad/s) </summary>
            public Single pitchspeed;
            /// <summary> Yaw angular speed (rad/s) </summary>
            public Single yawspeed;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_attitude_quaternion_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Quaternion component 1, w (1 in null-rotation) </summary>
            public Single q1;
            /// <summary> Quaternion component 2, x (0 in null-rotation) </summary>
            public Single q2;
            /// <summary> Quaternion component 3, y (0 in null-rotation) </summary>
            public Single q3;
            /// <summary> Quaternion component 4, z (0 in null-rotation) </summary>
            public Single q4;
            /// <summary> Roll angular speed (rad/s) </summary>
            public Single rollspeed;
            /// <summary> Pitch angular speed (rad/s) </summary>
            public Single pitchspeed;
            /// <summary> Yaw angular speed (rad/s) </summary>
            public Single yawspeed;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_local_position_ned_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> X Position </summary>
            public Single x;
            /// <summary> Y Position </summary>
            public Single y;
            /// <summary> Z Position </summary>
            public Single z;
            /// <summary> X Speed </summary>
            public Single vx;
            /// <summary> Y Speed </summary>
            public Single vy;
            /// <summary> Z Speed </summary>
            public Single vz;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_global_position_int_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Latitude, expressed as degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude, expressed as degrees * 1E7 </summary>
            public Int32 lon;
            /// <summary> Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well) </summary>
            public Int32 alt;
            /// <summary> Altitude above ground in meters, expressed as * 1000 (millimeters) </summary>
            public Int32 relative_alt;
            /// <summary> Ground X Speed (Latitude, positive north), expressed as m/s * 100 </summary>
            public Int16 vx;
            /// <summary> Ground Y Speed (Longitude, positive east), expressed as m/s * 100 </summary>
            public Int16 vy;
            /// <summary> Ground Z Speed (Altitude, positive down), expressed as m/s * 100 </summary>
            public Int16 vz;
            /// <summary> Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX </summary>
            public UInt16 hdg;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_rc_channels_scaled_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX. </summary>
            public Int16 chan1_scaled;
            /// <summary> RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX. </summary>
            public Int16 chan2_scaled;
            /// <summary> RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX. </summary>
            public Int16 chan3_scaled;
            /// <summary> RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX. </summary>
            public Int16 chan4_scaled;
            /// <summary> RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX. </summary>
            public Int16 chan5_scaled;
            /// <summary> RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX. </summary>
            public Int16 chan6_scaled;
            /// <summary> RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX. </summary>
            public Int16 chan7_scaled;
            /// <summary> RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX. </summary>
            public Int16 chan8_scaled;
            /// <summary> Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos. </summary>
            public byte port;
            /// <summary> Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown. </summary>
            public byte rssi;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_rc_channels_raw_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan1_raw;
            /// <summary> RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan2_raw;
            /// <summary> RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan3_raw;
            /// <summary> RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan4_raw;
            /// <summary> RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan5_raw;
            /// <summary> RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan6_raw;
            /// <summary> RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan7_raw;
            /// <summary> RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan8_raw;
            /// <summary> Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos. </summary>
            public byte port;
            /// <summary> Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown. </summary>
            public byte rssi;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_servo_output_raw_t
        {
            /// <summary> Timestamp (microseconds since system boot) </summary>
            public UInt32 time_usec;
            /// <summary> Servo output 1 value, in microseconds </summary>
            public UInt16 servo1_raw;
            /// <summary> Servo output 2 value, in microseconds </summary>
            public UInt16 servo2_raw;
            /// <summary> Servo output 3 value, in microseconds </summary>
            public UInt16 servo3_raw;
            /// <summary> Servo output 4 value, in microseconds </summary>
            public UInt16 servo4_raw;
            /// <summary> Servo output 5 value, in microseconds </summary>
            public UInt16 servo5_raw;
            /// <summary> Servo output 6 value, in microseconds </summary>
            public UInt16 servo6_raw;
            /// <summary> Servo output 7 value, in microseconds </summary>
            public UInt16 servo7_raw;
            /// <summary> Servo output 8 value, in microseconds </summary>
            public UInt16 servo8_raw;
            /// <summary> Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos. </summary>
            public byte port;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_request_partial_list_t
        {
            /// <summary> Start index, 0 by default </summary>
            public Int16 start_index;
            /// <summary> End index, -1 by default (-1: send list to end). Else a valid index of the list </summary>
            public Int16 end_index;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_write_partial_list_t
        {
            /// <summary> Start index, 0 by default and smaller / equal to the largest index of the current onboard list. </summary>
            public Int16 start_index;
            /// <summary> End index, equal or greater than start index. </summary>
            public Int16 end_index;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_item_t
        {
            /// <summary> PARAM1, see MAV_CMD enum </summary>
            public Single param1;
            /// <summary> PARAM2, see MAV_CMD enum </summary>
            public Single param2;
            /// <summary> PARAM3, see MAV_CMD enum </summary>
            public Single param3;
            /// <summary> PARAM4, see MAV_CMD enum </summary>
            public Single param4;
            /// <summary> PARAM5 / local: x position, global: latitude </summary>
            public Single x;
            /// <summary> PARAM6 / y position: global: longitude </summary>
            public Single y;
            /// <summary> PARAM7 / z position: global: altitude (relative or absolute, depending on frame. </summary>
            public Single z;
            /// <summary> Sequence </summary>
            public UInt16 seq;
            /// <summary> The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs </summary>
            public UInt16 command;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h </summary>
            public byte frame;
            /// <summary> false:0, true:1 </summary>
            public byte current;
            /// <summary> autocontinue to next wp </summary>
            public byte autocontinue;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_request_t
        {
            /// <summary> Sequence </summary>
            public UInt16 seq;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_set_current_t
        {
            /// <summary> Sequence </summary>
            public UInt16 seq;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_current_t
        {
            /// <summary> Sequence </summary>
            public UInt16 seq;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_request_list_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_count_t
        {
            /// <summary> Number of mission items in the sequence </summary>
            public UInt16 count;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_clear_all_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_item_reached_t
        {
            /// <summary> Sequence </summary>
            public UInt16 seq;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_ack_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> See MAV_MISSION_RESULT enum </summary>
            public byte type;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_set_gps_global_origin_t
        {
            /// <summary> Latitude (WGS84), in degrees * 1E7 </summary>
            public Int32 latitude;
            /// <summary> Longitude (WGS84, in degrees * 1E7 </summary>
            public Int32 longitude;
            /// <summary> Altitude (AMSL), in meters * 1000 (positive for up) </summary>
            public Int32 altitude;
            /// <summary> System ID </summary>
            public byte target_system;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gps_global_origin_t
        {
            /// <summary> Latitude (WGS84), in degrees * 1E7 </summary>
            public Int32 latitude;
            /// <summary> Longitude (WGS84), in degrees * 1E7 </summary>
            public Int32 longitude;
            /// <summary> Altitude (AMSL), in meters * 1000 (positive for up) </summary>
            public Int32 altitude;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_param_map_rc_t
        {
            /// <summary> Initial parameter value </summary>
            public Single param_value0;
            /// <summary> Scale, maps the RC range [-1, 1] to a parameter value </summary>
            public Single scale;
            /// <summary> Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation) </summary>
            public Single param_value_min;
            /// <summary> Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation) </summary>
            public Single param_value_max;
            /// <summary> Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index. </summary>
            public Int16 param_index;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] param_id;
            /// <summary> Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob on the RC. </summary>
            public byte parameter_rc_channel_index;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_request_int_t
        {
            /// <summary> Sequence </summary>
            public UInt16 seq;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_safety_set_allowed_area_t
        {
            /// <summary> x position 1 / Latitude 1 </summary>
            public Single p1x;
            /// <summary> y position 1 / Longitude 1 </summary>
            public Single p1y;
            /// <summary> z position 1 / Altitude 1 </summary>
            public Single p1z;
            /// <summary> x position 2 / Latitude 2 </summary>
            public Single p2x;
            /// <summary> y position 2 / Longitude 2 </summary>
            public Single p2y;
            /// <summary> z position 2 / Altitude 2 </summary>
            public Single p2z;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down. </summary>
            public byte frame;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_safety_allowed_area_t
        {
            /// <summary> x position 1 / Latitude 1 </summary>
            public Single p1x;
            /// <summary> y position 1 / Longitude 1 </summary>
            public Single p1y;
            /// <summary> z position 1 / Altitude 1 </summary>
            public Single p1z;
            /// <summary> x position 2 / Latitude 2 </summary>
            public Single p2x;
            /// <summary> y position 2 / Longitude 2 </summary>
            public Single p2y;
            /// <summary> z position 2 / Altitude 2 </summary>
            public Single p2z;
            /// <summary> Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down. </summary>
            public byte frame;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_attitude_quaternion_cov_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public float q;
            /// <summary> Roll angular speed (rad/s) </summary>
            public Single rollspeed;
            /// <summary> Pitch angular speed (rad/s) </summary>
            public Single pitchspeed;
            /// <summary> Yaw angular speed (rad/s) </summary>
            public Single yawspeed;
            /// <summary> Attitude covariance </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
            public float covariance;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_nav_controller_output_t
        {
            /// <summary> Current desired roll in degrees </summary>
            public Single nav_roll;
            /// <summary> Current desired pitch in degrees </summary>
            public Single nav_pitch;
            /// <summary> Current altitude error in meters </summary>
            public Single alt_error;
            /// <summary> Current airspeed error in meters/second </summary>
            public Single aspd_error;
            /// <summary> Current crosstrack error on x-y plane in meters </summary>
            public Single xtrack_error;
            /// <summary> Current desired heading in degrees </summary>
            public Int16 nav_bearing;
            /// <summary> Bearing to current MISSION/target in degrees </summary>
            public Int16 target_bearing;
            /// <summary> Distance to active MISSION in meters </summary>
            public UInt16 wp_dist;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_global_position_int_cov_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver. </summary>
            public UInt64 time_utc;
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Latitude, expressed as degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude, expressed as degrees * 1E7 </summary>
            public Int32 lon;
            /// <summary> Altitude in meters, expressed as * 1000 (millimeters), above MSL </summary>
            public Int32 alt;
            /// <summary> Altitude above ground in meters, expressed as * 1000 (millimeters) </summary>
            public Int32 relative_alt;
            /// <summary> Ground X Speed (Latitude), expressed as m/s </summary>
            public Single vx;
            /// <summary> Ground Y Speed (Longitude), expressed as m/s </summary>
            public Single vy;
            /// <summary> Ground Z Speed (Altitude), expressed as m/s </summary>
            public Single vz;
            /// <summary> Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
            public float covariance;
            /// <summary> Class id of the estimator this estimate originated from. </summary>
            public byte estimator_type;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_local_position_ned_cov_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver. </summary>
            public UInt64 time_utc;
            /// <summary> Timestamp (milliseconds since system boot). 0 for system without monotonic timestamp </summary>
            public UInt32 time_boot_ms;
            /// <summary> X Position </summary>
            public Single x;
            /// <summary> Y Position </summary>
            public Single y;
            /// <summary> Z Position </summary>
            public Single z;
            /// <summary> X Speed (m/s) </summary>
            public Single vx;
            /// <summary> Y Speed (m/s) </summary>
            public Single vy;
            /// <summary> Z Speed (m/s) </summary>
            public Single vz;
            /// <summary> X Acceleration (m/s^2) </summary>
            public Single ax;
            /// <summary> Y Acceleration (m/s^2) </summary>
            public Single ay;
            /// <summary> Z Acceleration (m/s^2) </summary>
            public Single az;
            /// <summary> Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 45)]
            public float covariance;
            /// <summary> Class id of the estimator this estimate originated from. </summary>
            public byte estimator_type;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_rc_channels_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan1_raw;
            /// <summary> RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan2_raw;
            /// <summary> RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan3_raw;
            /// <summary> RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan4_raw;
            /// <summary> RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan5_raw;
            /// <summary> RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan6_raw;
            /// <summary> RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan7_raw;
            /// <summary> RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan8_raw;
            /// <summary> RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan9_raw;
            /// <summary> RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan10_raw;
            /// <summary> RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan11_raw;
            /// <summary> RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan12_raw;
            /// <summary> RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan13_raw;
            /// <summary> RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan14_raw;
            /// <summary> RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan15_raw;
            /// <summary> RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan16_raw;
            /// <summary> RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan17_raw;
            /// <summary> RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused. </summary>
            public UInt16 chan18_raw;
            /// <summary> Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available. </summary>
            public byte chancount;
            /// <summary> Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown. </summary>
            public byte rssi;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_request_data_stream_t
        {
            /// <summary> The requested message rate </summary>
            public UInt16 req_message_rate;
            /// <summary> The target requested to send the message stream. </summary>
            public byte target_system;
            /// <summary> The target requested to send the message stream. </summary>
            public byte target_component;
            /// <summary> The ID of the requested data stream </summary>
            public byte req_stream_id;
            /// <summary> 1 to start sending, 0 to stop sending. </summary>
            public byte start_stop;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_data_stream_t
        {
            /// <summary> The message rate </summary>
            public UInt16 message_rate;
            /// <summary> The ID of the requested data stream </summary>
            public byte stream_id;
            /// <summary> 1 stream is enabled, 0 stream is stopped. </summary>
            public byte on_off;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_manual_control_t
        {
            /// <summary> X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle. </summary>
            public Int16 x;
            /// <summary> Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle. </summary>
            public Int16 y;
            /// <summary> Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust. </summary>
            public Int16 z;
            /// <summary> R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle. </summary>
            public Int16 r;
            /// <summary> A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1. </summary>
            public UInt16 buttons;
            /// <summary> The system to be controlled. </summary>
            public byte target;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_rc_channels_override_t
        {
            /// <summary> RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
            public UInt16 chan1_raw;
            /// <summary> RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
            public UInt16 chan2_raw;
            /// <summary> RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
            public UInt16 chan3_raw;
            /// <summary> RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
            public UInt16 chan4_raw;
            /// <summary> RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
            public UInt16 chan5_raw;
            /// <summary> RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
            public UInt16 chan6_raw;
            /// <summary> RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
            public UInt16 chan7_raw;
            /// <summary> RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
            public UInt16 chan8_raw;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_mission_item_int_t
        {
            /// <summary> PARAM1, see MAV_CMD enum </summary>
            public Single param1;
            /// <summary> PARAM2, see MAV_CMD enum </summary>
            public Single param2;
            /// <summary> PARAM3, see MAV_CMD enum </summary>
            public Single param3;
            /// <summary> PARAM4, see MAV_CMD enum </summary>
            public Single param4;
            /// <summary> PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7 </summary>
            public Int32 x;
            /// <summary> PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7 </summary>
            public Int32 y;
            /// <summary> PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame. </summary>
            public Single z;
            /// <summary> Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4). </summary>
            public UInt16 seq;
            /// <summary> The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs </summary>
            public UInt16 command;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h </summary>
            public byte frame;
            /// <summary> false:0, true:1 </summary>
            public byte current;
            /// <summary> autocontinue to next wp </summary>
            public byte autocontinue;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_vfr_hud_t
        {
            /// <summary> Current airspeed in m/s </summary>
            public Single airspeed;
            /// <summary> Current ground speed in m/s </summary>
            public Single groundspeed;
            /// <summary> Current altitude (MSL), in meters </summary>
            public Single alt;
            /// <summary> Current climb rate in meters/second </summary>
            public Single climb;
            /// <summary> Current heading in degrees, in compass units (0..360, 0=north) </summary>
            public Int16 heading;
            /// <summary> Current throttle setting in integer percent, 0 to 100 </summary>
            public UInt16 throttle;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_command_int_t
        {
            /// <summary> PARAM1, see MAV_CMD enum </summary>
            public Single param1;
            /// <summary> PARAM2, see MAV_CMD enum </summary>
            public Single param2;
            /// <summary> PARAM3, see MAV_CMD enum </summary>
            public Single param3;
            /// <summary> PARAM4, see MAV_CMD enum </summary>
            public Single param4;
            /// <summary> PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7 </summary>
            public Int32 x;
            /// <summary> PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7 </summary>
            public Int32 y;
            /// <summary> PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame. </summary>
            public Single z;
            /// <summary> The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs </summary>
            public UInt16 command;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h </summary>
            public byte frame;
            /// <summary> false:0, true:1 </summary>
            public byte current;
            /// <summary> autocontinue to next wp </summary>
            public byte autocontinue;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_command_long_t
        {
            /// <summary> Parameter 1, as defined by MAV_CMD enum. </summary>
            public Single param1;
            /// <summary> Parameter 2, as defined by MAV_CMD enum. </summary>
            public Single param2;
            /// <summary> Parameter 3, as defined by MAV_CMD enum. </summary>
            public Single param3;
            /// <summary> Parameter 4, as defined by MAV_CMD enum. </summary>
            public Single param4;
            /// <summary> Parameter 5, as defined by MAV_CMD enum. </summary>
            public Single param5;
            /// <summary> Parameter 6, as defined by MAV_CMD enum. </summary>
            public Single param6;
            /// <summary> Parameter 7, as defined by MAV_CMD enum. </summary>
            public Single param7;
            /// <summary> Command ID, as defined by MAV_CMD enum. </summary>
            public UInt16 command;
            /// <summary> System which should execute the command </summary>
            public byte target_system;
            /// <summary> Component which should execute the command, 0 for all components </summary>
            public byte target_component;
            /// <summary> 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) </summary>
            public byte confirmation;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_command_ack_t
        {
            /// <summary> Command ID, as defined by MAV_CMD enum. </summary>
            public UInt16 command;
            /// <summary> See MAV_RESULT enum </summary>
            public byte result;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_manual_setpoint_t
        {
            /// <summary> Timestamp in milliseconds since system boot </summary>
            public UInt32 time_boot_ms;
            /// <summary> Desired roll rate in radians per second </summary>
            public Single roll;
            /// <summary> Desired pitch rate in radians per second </summary>
            public Single pitch;
            /// <summary> Desired yaw rate in radians per second </summary>
            public Single yaw;
            /// <summary> Collective thrust, normalized to 0 .. 1 </summary>
            public Single thrust;
            /// <summary> Flight mode switch position, 0.. 255 </summary>
            public byte mode_switch;
            /// <summary> Override mode switch position, 0.. 255 </summary>
            public byte manual_override_switch;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_set_attitude_target_t
        {
            /// <summary> Timestamp in milliseconds since system boot </summary>
            public UInt32 time_boot_ms;
            /// <summary> Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public float[] q;
            /// <summary> Body roll rate in radians per second </summary>
            public Single body_roll_rate;
            /// <summary> Body roll rate in radians per second </summary>
            public Single body_pitch_rate;
            /// <summary> Body roll rate in radians per second </summary>
            public Single body_yaw_rate;
            /// <summary> Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust) </summary>
            public Single thrust;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude </summary>
            public byte type_mask;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_attitude_target_t
        {
            /// <summary> Timestamp in milliseconds since system boot </summary>
            public UInt32 time_boot_ms;
            /// <summary> Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public float[] q;

            /// <summary> Body roll rate in radians per second </summary>
            public Single body_roll_rate;
            /// <summary> Body roll rate in radians per second </summary>
            public Single body_pitch_rate;
            /// <summary> Body roll rate in radians per second </summary>
            public Single body_yaw_rate;
            /// <summary> Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust) </summary>
            public Single thrust;
            /// <summary> Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude </summary>
            public byte type_mask;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_set_position_target_local_ned_t
        {
            /// <summary> Timestamp in milliseconds since system boot </summary>
            public UInt32 time_boot_ms;
            /// <summary> X Position in NED frame in meters </summary>
            public Single x;
            /// <summary> Y Position in NED frame in meters </summary>
            public Single y;
            /// <summary> Z Position in NED frame in meters (note, altitude is negative in NED) </summary>
            public Single z;
            /// <summary> X velocity in NED frame in meter / s </summary>
            public Single vx;
            /// <summary> Y velocity in NED frame in meter / s </summary>
            public Single vy;
            /// <summary> Z velocity in NED frame in meter / s </summary>
            public Single vz;
            /// <summary> X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afx;
            /// <summary> Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afy;
            /// <summary> Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afz;
            /// <summary> yaw setpoint in rad </summary>
            public Single yaw;
            /// <summary> yaw rate setpoint in rad/s </summary>
            public Single yaw_rate;
            /// <summary> Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate </summary>
            public UInt16 type_mask;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9 </summary>
            public byte coordinate_frame;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_position_target_local_ned_t
        {
            /// <summary> Timestamp in milliseconds since system boot </summary>
            public UInt32 time_boot_ms;
            /// <summary> X Position in NED frame in meters </summary>
            public Single x;
            /// <summary> Y Position in NED frame in meters </summary>
            public Single y;
            /// <summary> Z Position in NED frame in meters (note, altitude is negative in NED) </summary>
            public Single z;
            /// <summary> X velocity in NED frame in meter / s </summary>
            public Single vx;
            /// <summary> Y velocity in NED frame in meter / s </summary>
            public Single vy;
            /// <summary> Z velocity in NED frame in meter / s </summary>
            public Single vz;
            /// <summary> X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afx;
            /// <summary> Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afy;
            /// <summary> Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afz;
            /// <summary> yaw setpoint in rad </summary>
            public Single yaw;
            /// <summary> yaw rate setpoint in rad/s </summary>
            public Single yaw_rate;
            /// <summary> Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate </summary>
            public UInt16 type_mask;
            /// <summary> Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9 </summary>
            public byte coordinate_frame;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_set_position_target_global_int_t
        {
            /// <summary> Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency. </summary>
            public UInt32 time_boot_ms;
            /// <summary> X Position in WGS84 frame in 1e7 * meters </summary>
            public Int32 lat_int;
            /// <summary> Y Position in WGS84 frame in 1e7 * meters </summary>
            public Int32 lon_int;
            /// <summary> Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT </summary>
            public Single alt;
            /// <summary> X velocity in NED frame in meter / s </summary>
            public Single vx;
            /// <summary> Y velocity in NED frame in meter / s </summary>
            public Single vy;
            /// <summary> Z velocity in NED frame in meter / s </summary>
            public Single vz;
            /// <summary> X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afx;
            /// <summary> Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afy;
            /// <summary> Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afz;
            /// <summary> yaw setpoint in rad </summary>
            public Single yaw;
            /// <summary> yaw rate setpoint in rad/s </summary>
            public Single yaw_rate;
            /// <summary> Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate </summary>
            public UInt16 type_mask;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11 </summary>
            public byte coordinate_frame;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_position_target_global_int_t
        {
            /// <summary> Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency. </summary>
            public UInt32 time_boot_ms;
            /// <summary> X Position in WGS84 frame in 1e7 * meters </summary>
            public Int32 lat_int;
            /// <summary> Y Position in WGS84 frame in 1e7 * meters </summary>
            public Int32 lon_int;
            /// <summary> Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT </summary>
            public Single alt;
            /// <summary> X velocity in NED frame in meter / s </summary>
            public Single vx;
            /// <summary> Y velocity in NED frame in meter / s </summary>
            public Single vy;
            /// <summary> Z velocity in NED frame in meter / s </summary>
            public Single vz;
            /// <summary> X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afx;
            /// <summary> Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afy;
            /// <summary> Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N </summary>
            public Single afz;
            /// <summary> yaw setpoint in rad </summary>
            public Single yaw;
            /// <summary> yaw rate setpoint in rad/s </summary>
            public Single yaw_rate;
            /// <summary> Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate </summary>
            public UInt16 type_mask;
            /// <summary> Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11 </summary>
            public byte coordinate_frame;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_local_position_ned_system_global_offset_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> X Position </summary>
            public Single x;
            /// <summary> Y Position </summary>
            public Single y;
            /// <summary> Z Position </summary>
            public Single z;
            /// <summary> Roll </summary>
            public Single roll;
            /// <summary> Pitch </summary>
            public Single pitch;
            /// <summary> Yaw </summary>
            public Single yaw;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_hil_state_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> Roll angle (rad) </summary>
            public Single roll;
            /// <summary> Pitch angle (rad) </summary>
            public Single pitch;
            /// <summary> Yaw angle (rad) </summary>
            public Single yaw;
            /// <summary> Body frame roll / phi angular speed (rad/s) </summary>
            public Single rollspeed;
            /// <summary> Body frame pitch / theta angular speed (rad/s) </summary>
            public Single pitchspeed;
            /// <summary> Body frame yaw / psi angular speed (rad/s) </summary>
            public Single yawspeed;
            /// <summary> Latitude, expressed as * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude, expressed as * 1E7 </summary>
            public Int32 lon;
            /// <summary> Altitude in meters, expressed as * 1000 (millimeters) </summary>
            public Int32 alt;
            /// <summary> Ground X Speed (Latitude), expressed as m/s * 100 </summary>
            public Int16 vx;
            /// <summary> Ground Y Speed (Longitude), expressed as m/s * 100 </summary>
            public Int16 vy;
            /// <summary> Ground Z Speed (Altitude), expressed as m/s * 100 </summary>
            public Int16 vz;
            /// <summary> X acceleration (mg) </summary>
            public Int16 xacc;
            /// <summary> Y acceleration (mg) </summary>
            public Int16 yacc;
            /// <summary> Z acceleration (mg) </summary>
            public Int16 zacc;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_hil_controls_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> Control output -1 .. 1 </summary>
            public Single roll_ailerons;
            /// <summary> Control output -1 .. 1 </summary>
            public Single pitch_elevator;
            /// <summary> Control output -1 .. 1 </summary>
            public Single yaw_rudder;
            /// <summary> Throttle 0 .. 1 </summary>
            public Single throttle;
            /// <summary> Aux 1, -1 .. 1 </summary>
            public Single aux1;
            /// <summary> Aux 2, -1 .. 1 </summary>
            public Single aux2;
            /// <summary> Aux 3, -1 .. 1 </summary>
            public Single aux3;
            /// <summary> Aux 4, -1 .. 1 </summary>
            public Single aux4;
            /// <summary> System mode (MAV_MODE) </summary>
            public byte mode;
            /// <summary> Navigation mode (MAV_NAV_MODE) </summary>
            public byte nav_mode;

        };



        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_hil_actuator_controls_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            // Flags as bitfield, reserved for future use.
            public UInt64 flags;
            // Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public float[] controls;
            // System mode (MAV_MODE), includes arming state.
            public byte mode;

        };

        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_hil_rc_inputs_raw_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> RC channel 1 value, in microseconds </summary>
            public UInt16 chan1_raw;
            /// <summary> RC channel 2 value, in microseconds </summary>
            public UInt16 chan2_raw;
            /// <summary> RC channel 3 value, in microseconds </summary>
            public UInt16 chan3_raw;
            /// <summary> RC channel 4 value, in microseconds </summary>
            public UInt16 chan4_raw;
            /// <summary> RC channel 5 value, in microseconds </summary>
            public UInt16 chan5_raw;
            /// <summary> RC channel 6 value, in microseconds </summary>
            public UInt16 chan6_raw;
            /// <summary> RC channel 7 value, in microseconds </summary>
            public UInt16 chan7_raw;
            /// <summary> RC channel 8 value, in microseconds </summary>
            public UInt16 chan8_raw;
            /// <summary> RC channel 9 value, in microseconds </summary>
            public UInt16 chan9_raw;
            /// <summary> RC channel 10 value, in microseconds </summary>
            public UInt16 chan10_raw;
            /// <summary> RC channel 11 value, in microseconds </summary>
            public UInt16 chan11_raw;
            /// <summary> RC channel 12 value, in microseconds </summary>
            public UInt16 chan12_raw;
            /// <summary> Receive signal strength indicator, 0: 0%, 255: 100% </summary>
            public byte rssi;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_optical_flow_t
        {
            /// <summary> Timestamp (UNIX) </summary>
            public UInt64 time_usec;
            /// <summary> Flow in meters in x-sensor direction, angular-speed compensated </summary>
            public Single flow_comp_m_x;
            /// <summary> Flow in meters in y-sensor direction, angular-speed compensated </summary>
            public Single flow_comp_m_y;
            /// <summary> Ground distance in meters. Positive value: distance known. Negative value: Unknown distance </summary>
            public Single ground_distance;
            /// <summary> Flow in pixels * 10 in x-sensor direction (dezi-pixels) </summary>
            public Int16 flow_x;
            /// <summary> Flow in pixels * 10 in y-sensor direction (dezi-pixels) </summary>
            public Int16 flow_y;
            /// <summary> Sensor ID </summary>
            public byte sensor_id;
            /// <summary> Optical flow quality / confidence. 0: bad, 255: maximum quality </summary>
            public byte quality;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_global_vision_position_estimate_t
        {
            /// <summary> Timestamp (microseconds, synced to UNIX time or since system boot) </summary>
            public UInt64 usec;
            /// <summary> Global X position </summary>
            public Single x;
            /// <summary> Global Y position </summary>
            public Single y;
            /// <summary> Global Z position </summary>
            public Single z;
            /// <summary> Roll angle in rad </summary>
            public Single roll;
            /// <summary> Pitch angle in rad </summary>
            public Single pitch;
            /// <summary> Yaw angle in rad </summary>
            public Single yaw;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_vision_position_estimate_t
        {
            /// <summary> Timestamp (microseconds, synced to UNIX time or since system boot) </summary>
            public UInt64 usec;
            /// <summary> Global X position </summary>
            public Single x;
            /// <summary> Global Y position </summary>
            public Single y;
            /// <summary> Global Z position </summary>
            public Single z;
            /// <summary> Roll angle in rad </summary>
            public Single roll;
            /// <summary> Pitch angle in rad </summary>
            public Single pitch;
            /// <summary> Yaw angle in rad </summary>
            public Single yaw;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_vision_speed_estimate_t
        {
            /// <summary> Timestamp (microseconds, synced to UNIX time or since system boot) </summary>
            public UInt64 usec;
            /// <summary> Global X speed </summary>
            public Single x;
            /// <summary> Global Y speed </summary>
            public Single y;
            /// <summary> Global Z speed </summary>
            public Single z;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_vicon_position_estimate_t
        {
            /// <summary> Timestamp (microseconds, synced to UNIX time or since system boot) </summary>
            public UInt64 usec;
            /// <summary> Global X position </summary>
            public Single x;
            /// <summary> Global Y position </summary>
            public Single y;
            /// <summary> Global Z position </summary>
            public Single z;
            /// <summary> Roll angle in rad </summary>
            public Single roll;
            /// <summary> Pitch angle in rad </summary>
            public Single pitch;
            /// <summary> Yaw angle in rad </summary>
            public Single yaw;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_highres_imu_t
        {
            /// <summary> Timestamp (microseconds, synced to UNIX time or since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> X acceleration (m/s^2) </summary>
            public Single xacc;
            /// <summary> Y acceleration (m/s^2) </summary>
            public Single yacc;
            /// <summary> Z acceleration (m/s^2) </summary>
            public Single zacc;
            /// <summary> Angular speed around X axis (rad / sec) </summary>
            public Single xgyro;
            /// <summary> Angular speed around Y axis (rad / sec) </summary>
            public Single ygyro;
            /// <summary> Angular speed around Z axis (rad / sec) </summary>
            public Single zgyro;
            /// <summary> X Magnetic field (Gauss) </summary>
            public Single xmag;
            /// <summary> Y Magnetic field (Gauss) </summary>
            public Single ymag;
            /// <summary> Z Magnetic field (Gauss) </summary>
            public Single zmag;
            /// <summary> Absolute pressure in millibar </summary>
            public Single abs_pressure;
            /// <summary> Differential pressure in millibar </summary>
            public Single diff_pressure;
            /// <summary> Altitude calculated from pressure </summary>
            public Single pressure_alt;
            /// <summary> Temperature in degrees celsius </summary>
            public Single temperature;
            /// <summary> Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature </summary>
            public UInt16 fields_updated;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_optical_flow_rad_t
        {
            /// <summary> Timestamp (microseconds, synced to UNIX time or since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the. </summary>
            public UInt32 integration_time_us;
            /// <summary> Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.) </summary>
            public Single integrated_x;
            /// <summary> Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.) </summary>
            public Single integrated_y;
            /// <summary> RH rotation around X axis (rad) </summary>
            public Single integrated_xgyro;
            /// <summary> RH rotation around Y axis (rad) </summary>
            public Single integrated_ygyro;
            /// <summary> RH rotation around Z axis (rad) </summary>
            public Single integrated_zgyro;
            /// <summary> Time in microseconds since the distance was sampled. </summary>
            public UInt32 time_delta_distance_us;
            /// <summary> Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance. </summary>
            public Single distance;
            /// <summary> Temperature * 100 in centi-degrees Celsius </summary>
            public Int16 temperature;
            /// <summary> Sensor ID </summary>
            public byte sensor_id;
            /// <summary> Optical flow quality / confidence. 0: no valid flow, 255: maximum quality </summary>
            public byte quality;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_hil_sensor_t
        {
            /// <summary> Timestamp (microseconds, synced to UNIX time or since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> X acceleration (m/s^2) </summary>
            public Single xacc;
            /// <summary> Y acceleration (m/s^2) </summary>
            public Single yacc;
            /// <summary> Z acceleration (m/s^2) </summary>
            public Single zacc;
            /// <summary> Angular speed around X axis in body frame (rad / sec) </summary>
            public Single xgyro;
            /// <summary> Angular speed around Y axis in body frame (rad / sec) </summary>
            public Single ygyro;
            /// <summary> Angular speed around Z axis in body frame (rad / sec) </summary>
            public Single zgyro;
            /// <summary> X Magnetic field (Gauss) </summary>
            public Single xmag;
            /// <summary> Y Magnetic field (Gauss) </summary>
            public Single ymag;
            /// <summary> Z Magnetic field (Gauss) </summary>
            public Single zmag;
            /// <summary> Absolute pressure in millibar </summary>
            public Single abs_pressure;
            /// <summary> Differential pressure (airspeed) in millibar </summary>
            public Single diff_pressure;
            /// <summary> Altitude calculated from pressure </summary>
            public Single pressure_alt;
            /// <summary> Temperature in degrees celsius </summary>
            public Single temperature;
            /// <summary> Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature </summary>
            public UInt32 fields_updated;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_sim_state_t
        {
            /// <summary> True attitude quaternion component 1, w (1 in null-rotation) </summary>
            public Single q1;
            /// <summary> True attitude quaternion component 2, x (0 in null-rotation) </summary>
            public Single q2;
            /// <summary> True attitude quaternion component 3, y (0 in null-rotation) </summary>
            public Single q3;
            /// <summary> True attitude quaternion component 4, z (0 in null-rotation) </summary>
            public Single q4;
            /// <summary> Attitude roll expressed as Euler angles, not recommended except for human-readable outputs </summary>
            public Single roll;
            /// <summary> Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs </summary>
            public Single pitch;
            /// <summary> Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs </summary>
            public Single yaw;
            /// <summary> X acceleration m/s/s </summary>
            public Single xacc;
            /// <summary> Y acceleration m/s/s </summary>
            public Single yacc;
            /// <summary> Z acceleration m/s/s </summary>
            public Single zacc;
            /// <summary> Angular speed around X axis rad/s </summary>
            public Single xgyro;
            /// <summary> Angular speed around Y axis rad/s </summary>
            public Single ygyro;
            /// <summary> Angular speed around Z axis rad/s </summary>
            public Single zgyro;
            /// <summary> Latitude in degrees </summary>
            public Single lat;
            /// <summary> Longitude in degrees </summary>
            public Single lon;
            /// <summary> Altitude in meters </summary>
            public Single alt;
            /// <summary> Horizontal position standard deviation </summary>
            public Single std_dev_horz;
            /// <summary> Vertical position standard deviation </summary>
            public Single std_dev_vert;
            /// <summary> True velocity in m/s in NORTH direction in earth-fixed NED frame </summary>
            public Single vn;
            /// <summary> True velocity in m/s in EAST direction in earth-fixed NED frame </summary>
            public Single ve;
            /// <summary> True velocity in m/s in DOWN direction in earth-fixed NED frame </summary>
            public Single vd;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_radio_status_t
        {
            /// <summary> Receive errors </summary>
            public UInt16 rxerrors;
            /// <summary> Count of error corrected packets </summary>
            public UInt16 @fixed;
            /// <summary> Local signal strength </summary>
            public byte rssi;
            /// <summary> Remote signal strength </summary>
            public byte remrssi;
            /// <summary> Remaining free buffer space in percent. </summary>
            public byte txbuf;
            /// <summary> Background noise level </summary>
            public byte noise;
            /// <summary> Remote background noise level </summary>
            public byte remnoise;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_file_transfer_protocol_t
        {
            /// <summary> Network ID (0 for broadcast) </summary>
            public byte target_network;
            /// <summary> System ID (0 for broadcast) </summary>
            public byte target_system;
            /// <summary> Component ID (0 for broadcast) </summary>
            public byte target_component;
            /// <summary> Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification. </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 251)]
            public byte[] payload;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_timesync_t
        {
            /// <summary> Time sync timestamp 1 </summary>
            public Int64 tc1;
            /// <summary> Time sync timestamp 2 </summary>
            public Int64 ts1;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_camera_trigger_t
        {
            /// <summary> Timestamp for the image frame in microseconds </summary>
            public UInt64 time_usec;
            /// <summary> Image frame sequence </summary>
            public UInt32 seq;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_hil_gps_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> Latitude (WGS84), in degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude (WGS84), in degrees * 1E7 </summary>
            public Int32 lon;
            /// <summary> Altitude (AMSL, not WGS84), in meters * 1000 (positive for up) </summary>
            public Int32 alt;
            /// <summary> GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535 </summary>
            public UInt16 eph;
            /// <summary> GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535 </summary>
            public UInt16 epv;
            /// <summary> GPS ground speed (m/s * 100). If unknown, set to: 65535 </summary>
            public UInt16 vel;
            /// <summary> GPS velocity in cm/s in NORTH direction in earth-fixed NED frame </summary>
            public Int16 vn;
            /// <summary> GPS velocity in cm/s in EAST direction in earth-fixed NED frame </summary>
            public Int16 ve;
            /// <summary> GPS velocity in cm/s in DOWN direction in earth-fixed NED frame </summary>
            public Int16 vd;
            /// <summary> Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535 </summary>
            public UInt16 cog;
            /// <summary> 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. </summary>
            public byte fix_type;
            /// <summary> Number of satellites visible. If unknown, set to 255 </summary>
            public byte satellites_visible;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_hil_optical_flow_t
        {
            /// <summary> Timestamp (microseconds, synced to UNIX time or since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the. </summary>
            public UInt32 integration_time_us;
            /// <summary> Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.) </summary>
            public Single integrated_x;
            /// <summary> Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.) </summary>
            public Single integrated_y;
            /// <summary> RH rotation around X axis (rad) </summary>
            public Single integrated_xgyro;
            /// <summary> RH rotation around Y axis (rad) </summary>
            public Single integrated_ygyro;
            /// <summary> RH rotation around Z axis (rad) </summary>
            public Single integrated_zgyro;
            /// <summary> Time in microseconds since the distance was sampled. </summary>
            public UInt32 time_delta_distance_us;
            /// <summary> Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance. </summary>
            public Single distance;
            /// <summary> Temperature * 100 in centi-degrees Celsius </summary>
            public Int16 temperature;
            /// <summary> Sensor ID </summary>
            public byte sensor_id;
            /// <summary> Optical flow quality / confidence. 0: no valid flow, 255: maximum quality </summary>
            public byte quality;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_hil_state_quaternion_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public float[] attitude_quaternion;
            /// <summary> Body frame roll / phi angular speed (rad/s) </summary>
            public Single rollspeed;
            /// <summary> Body frame pitch / theta angular speed (rad/s) </summary>
            public Single pitchspeed;
            /// <summary> Body frame yaw / psi angular speed (rad/s) </summary>
            public Single yawspeed;
            /// <summary> Latitude, expressed as * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude, expressed as * 1E7 </summary>
            public Int32 lon;
            /// <summary> Altitude in meters, expressed as * 1000 (millimeters) </summary>
            public Int32 alt;
            /// <summary> Ground X Speed (Latitude), expressed as m/s * 100 </summary>
            public Int16 vx;
            /// <summary> Ground Y Speed (Longitude), expressed as m/s * 100 </summary>
            public Int16 vy;
            /// <summary> Ground Z Speed (Altitude), expressed as m/s * 100 </summary>
            public Int16 vz;
            /// <summary> Indicated airspeed, expressed as m/s * 100 </summary>
            public UInt16 ind_airspeed;
            /// <summary> True airspeed, expressed as m/s * 100 </summary>
            public UInt16 true_airspeed;
            /// <summary> X acceleration (mg) </summary>
            public Int16 xacc;
            /// <summary> Y acceleration (mg) </summary>
            public Int16 yacc;
            /// <summary> Z acceleration (mg) </summary>
            public Int16 zacc;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_scaled_imu2_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> X acceleration (mg) </summary>
            public Int16 xacc;
            /// <summary> Y acceleration (mg) </summary>
            public Int16 yacc;
            /// <summary> Z acceleration (mg) </summary>
            public Int16 zacc;
            /// <summary> Angular speed around X axis (millirad /sec) </summary>
            public Int16 xgyro;
            /// <summary> Angular speed around Y axis (millirad /sec) </summary>
            public Int16 ygyro;
            /// <summary> Angular speed around Z axis (millirad /sec) </summary>
            public Int16 zgyro;
            /// <summary> X Magnetic field (milli tesla) </summary>
            public Int16 xmag;
            /// <summary> Y Magnetic field (milli tesla) </summary>
            public Int16 ymag;
            /// <summary> Z Magnetic field (milli tesla) </summary>
            public Int16 zmag;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_log_request_list_t
        {
            /// <summary> First log id (0 for first available) </summary>
            public UInt16 start;
            /// <summary> Last log id (0xffff for last available) </summary>
            public UInt16 end;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_log_entry_t
        {
            /// <summary> UTC timestamp of log in seconds since 1970, or 0 if not available </summary>
            public UInt32 time_utc;
            /// <summary> Size of the log (may be approximate) in bytes </summary>
            public UInt32 size;
            /// <summary> Log id </summary>
            public UInt16 id;
            /// <summary> Total number of logs </summary>
            public UInt16 num_logs;
            /// <summary> High log number </summary>
            public UInt16 last_log_num;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_log_request_data_t
        {
            /// <summary> Offset into the log </summary>
            public UInt32 ofs;
            /// <summary> Number of bytes </summary>
            public UInt32 count;
            /// <summary> Log id (from LOG_ENTRY reply) </summary>
            public UInt16 id;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_log_data_t
        {
            /// <summary> Offset into the log </summary>
            public UInt32 ofs;
            /// <summary> Log id (from LOG_ENTRY reply) </summary>
            public UInt16 id;
            /// <summary> Number of bytes (zero for end of log) </summary>
            public byte count;
            /// <summary> log data </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 90)]
            public byte[] data;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_log_erase_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_log_request_end_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gps_inject_data_t
        {
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;
            /// <summary> data length </summary>
            public byte len;
            /// <summary> raw data (110 is enough for 12 satellites of RTCMv2) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 110)]
            public byte[] data;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gps2_raw_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> Latitude (WGS84), in degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude (WGS84), in degrees * 1E7 </summary>
            public Int32 lon;
            /// <summary> Altitude (AMSL, not WGS84), in meters * 1000 (positive for up) </summary>
            public Int32 alt;
            /// <summary> Age of DGPS info </summary>
            public UInt32 dgps_age;
            /// <summary> GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX </summary>
            public UInt16 eph;
            /// <summary> GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX </summary>
            public UInt16 epv;
            /// <summary> GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX </summary>
            public UInt16 vel;
            /// <summary> Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX </summary>
            public UInt16 cog;
            /// <summary> 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS fix, 5: RTK Fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. </summary>
            public byte fix_type;
            /// <summary> Number of satellites visible. If unknown, set to 255 </summary>
            public byte satellites_visible;
            /// <summary> Number of DGPS satellites </summary>
            public byte dgps_numch;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_power_status_t
        {
            /// <summary> 5V rail voltage in millivolts </summary>
            public UInt16 Vcc;
            /// <summary> servo rail voltage in millivolts </summary>
            public UInt16 Vservo;
            /// <summary> power supply status flags (see MAV_POWER_STATUS enum) </summary>
            public UInt16 flags;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_serial_control_t
        {
            /// <summary> Baudrate of transfer. Zero means no change. </summary>
            public UInt32 baudrate;
            /// <summary> Timeout for reply data in milliseconds </summary>
            public UInt16 timeout;
            /// <summary> See SERIAL_CONTROL_DEV enum </summary>
            public byte device;
            /// <summary> See SERIAL_CONTROL_FLAG enum </summary>
            public byte flags;
            /// <summary> how many bytes in this transfer </summary>
            public byte count;
            /// <summary> serial data </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 70)]
            public byte[] data;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gps_rtk_t
        {
            /// <summary> Time since boot of last baseline message received in ms. </summary>
            public UInt32 time_last_baseline_ms;
            /// <summary> GPS Time of Week of last baseline </summary>
            public UInt32 tow;
            /// <summary> Current baseline in ECEF x or NED north component in mm. </summary>
            public Int32 baseline_a_mm;
            /// <summary> Current baseline in ECEF y or NED east component in mm. </summary>
            public Int32 baseline_b_mm;
            /// <summary> Current baseline in ECEF z or NED down component in mm. </summary>
            public Int32 baseline_c_mm;
            /// <summary> Current estimate of baseline accuracy. </summary>
            public UInt32 accuracy;
            /// <summary> Current number of integer ambiguity hypotheses. </summary>
            public Int32 iar_num_hypotheses;
            /// <summary> GPS Week Number of last baseline </summary>
            public UInt16 wn;
            /// <summary> Identification of connected RTK receiver. </summary>
            public byte rtk_receiver_id;
            /// <summary> GPS-specific health report for RTK data. </summary>
            public byte rtk_health;
            /// <summary> Rate of baseline messages being received by GPS, in HZ </summary>
            public byte rtk_rate;
            /// <summary> Current number of sats used for RTK calculation. </summary>
            public byte nsats;
            /// <summary> Coordinate system of baseline. 0 == ECEF, 1 == NED </summary>
            public byte baseline_coords_type;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_gps2_rtk_t
        {
            /// <summary> Time since boot of last baseline message received in ms. </summary>
            public UInt32 time_last_baseline_ms;
            /// <summary> GPS Time of Week of last baseline </summary>
            public UInt32 tow;
            /// <summary> Current baseline in ECEF x or NED north component in mm. </summary>
            public Int32 baseline_a_mm;
            /// <summary> Current baseline in ECEF y or NED east component in mm. </summary>
            public Int32 baseline_b_mm;
            /// <summary> Current baseline in ECEF z or NED down component in mm. </summary>
            public Int32 baseline_c_mm;
            /// <summary> Current estimate of baseline accuracy. </summary>
            public UInt32 accuracy;
            /// <summary> Current number of integer ambiguity hypotheses. </summary>
            public Int32 iar_num_hypotheses;
            /// <summary> GPS Week Number of last baseline </summary>
            public UInt16 wn;
            /// <summary> Identification of connected RTK receiver. </summary>
            public byte rtk_receiver_id;
            /// <summary> GPS-specific health report for RTK data. </summary>
            public byte rtk_health;
            /// <summary> Rate of baseline messages being received by GPS, in HZ </summary>
            public byte rtk_rate;
            /// <summary> Current number of sats used for RTK calculation. </summary>
            public byte nsats;
            /// <summary> Coordinate system of baseline. 0 == ECEF, 1 == NED </summary>
            public byte baseline_coords_type;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_scaled_imu3_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> X acceleration (mg) </summary>
            public Int16 xacc;
            /// <summary> Y acceleration (mg) </summary>
            public Int16 yacc;
            /// <summary> Z acceleration (mg) </summary>
            public Int16 zacc;
            /// <summary> Angular speed around X axis (millirad /sec) </summary>
            public Int16 xgyro;
            /// <summary> Angular speed around Y axis (millirad /sec) </summary>
            public Int16 ygyro;
            /// <summary> Angular speed around Z axis (millirad /sec) </summary>
            public Int16 zgyro;
            /// <summary> X Magnetic field (milli tesla) </summary>
            public Int16 xmag;
            /// <summary> Y Magnetic field (milli tesla) </summary>
            public Int16 ymag;
            /// <summary> Z Magnetic field (milli tesla) </summary>
            public Int16 zmag;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_data_transmission_handshake_t
        {
            /// <summary> total data size in bytes (set on ACK only) </summary>
            public UInt32 size;
            /// <summary> Width of a matrix or image </summary>
            public UInt16 width;
            /// <summary> Height of a matrix or image </summary>
            public UInt16 height;
            /// <summary> number of packets being sent (set on ACK only) </summary>
            public UInt16 packets;
            /// <summary> type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h) </summary>
            public byte type;
            /// <summary> payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only) </summary>
            public byte payload;
            /// <summary> JPEG quality out of [1,100] </summary>
            public byte jpg_quality;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_encapsulated_data_t
        {
            /// <summary> sequence number (starting with 0 on every transmission) </summary>
            public UInt16 seqnr;
            /// <summary> image data bytes </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 253)]
            public byte[] data;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_distance_sensor_t
        {
            /// <summary> Time since system boot </summary>
            public UInt32 time_boot_ms;
            /// <summary> Minimum distance the sensor can measure in centimeters </summary>
            public UInt16 min_distance;
            /// <summary> Maximum distance the sensor can measure in centimeters </summary>
            public UInt16 max_distance;
            /// <summary> Current distance reading </summary>
            public UInt16 current_distance;
            /// <summary> Type from MAV_DISTANCE_SENSOR enum. </summary>
            public byte type;
            /// <summary> Onboard ID of the sensor </summary>
            public byte id;
            /// <summary> Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. </summary>
            public byte orientation;
            /// <summary> Measurement covariance in centimeters, 0 for unknown / invalid readings </summary>
            public byte covariance;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_terrain_request_t
        {
            /// <summary> Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits) </summary>
            public UInt64 mask;
            /// <summary> Latitude of SW corner of first grid (degrees *10^7) </summary>
            public Int32 lat;
            /// <summary> Longitude of SW corner of first grid (in degrees *10^7) </summary>
            public Int32 lon;
            /// <summary> Grid spacing in meters </summary>
            public UInt16 grid_spacing;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_terrain_data_t
        {
            /// <summary> Latitude of SW corner of first grid (degrees *10^7) </summary>
            public Int32 lat;
            /// <summary> Longitude of SW corner of first grid (in degrees *10^7) </summary>
            public Int32 lon;
            /// <summary> Grid spacing in meters </summary>
            public UInt16 grid_spacing;
            /// <summary> Terrain data in meters AMSL </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public Int16[] data;
            /// <summary> bit within the terrain request mask </summary>
            public byte gridbit;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_terrain_check_t
        {
            /// <summary> Latitude (degrees *10^7) </summary>
            public Int32 lat;
            /// <summary> Longitude (degrees *10^7) </summary>
            public Int32 lon;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_terrain_report_t
        {
            /// <summary> Latitude (degrees *10^7) </summary>
            public Int32 lat;
            /// <summary> Longitude (degrees *10^7) </summary>
            public Int32 lon;
            /// <summary> Terrain height in meters AMSL </summary>
            public Single terrain_height;
            /// <summary> Current vehicle height above lat/lon terrain height (meters) </summary>
            public Single current_height;
            /// <summary> grid spacing (zero if terrain at this location unavailable) </summary>
            public UInt16 spacing;
            /// <summary> Number of 4x4 terrain blocks waiting to be received or read from disk </summary>
            public UInt16 pending;
            /// <summary> Number of 4x4 terrain blocks in memory </summary>
            public UInt16 loaded;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_scaled_pressure2_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Absolute pressure (hectopascal) </summary>
            public Single press_abs;
            /// <summary> Differential pressure 1 (hectopascal) </summary>
            public Single press_diff;
            /// <summary> Temperature measurement (0.01 degrees celsius) </summary>
            public Int16 temperature;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_att_pos_mocap_t
        {
            /// <summary> Timestamp (micros since boot or Unix epoch) </summary>
            public UInt64 time_usec;
            /// <summary> Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public float[] q;
            /// <summary> X position in meters (NED) </summary>
            public Single x;
            /// <summary> Y position in meters (NED) </summary>
            public Single y;
            /// <summary> Z position in meters (NED) </summary>
            public Single z;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_set_actuator_control_target_t
        {
            /// <summary> Timestamp (micros since boot or Unix epoch) </summary>
            public UInt64 time_usec;
            /// <summary> Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs. </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public float controls;
            /// <summary> Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances. </summary>
            public byte group_mlx;
            /// <summary> System ID </summary>
            public byte target_system;
            /// <summary> Component ID </summary>
            public byte target_component;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_actuator_control_target_t
        {
            /// <summary> Timestamp (micros since boot or Unix epoch) </summary>
            public UInt64 time_usec;
            /// <summary> Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs. </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public float[] controls;
            /// <summary> Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances. </summary>
            public byte group_mlx;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_altitude_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt64 time_usec;
            /// <summary> This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights. </summary>
            public Single altitude_monotonic;
            /// <summary> This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude. </summary>
            public Single altitude_amsl;
            /// <summary> This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive. </summary>
            public Single altitude_local;
            /// <summary> This is the altitude above the home position. It resets on each change of the current home position. </summary>
            public Single altitude_relative;
            /// <summary> This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown. </summary>
            public Single altitude_terrain;
            /// <summary> This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available. </summary>
            public Single bottom_clearance;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_resource_request_t
        {
            /// <summary> Request ID. This ID should be re-used when sending back URI contents </summary>
            public byte request_id;
            /// <summary> The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary </summary>
            public byte uri_type;
            /// <summary> The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum) </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 120)]
            public byte[] uri;
            /// <summary> The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream. </summary>
            public byte transfer_type;
            /// <summary> The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP). </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 120)]
            public byte[] storage;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_scaled_pressure3_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Absolute pressure (hectopascal) </summary>
            public Single press_abs;
            /// <summary> Differential pressure 1 (hectopascal) </summary>
            public Single press_diff;
            /// <summary> Temperature measurement (0.01 degrees celsius) </summary>
            public Int16 temperature;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_control_system_state_t
        {
            /// <summary> Timestamp (micros since boot or Unix epoch) </summary>
            public UInt64 time_usec;
            /// <summary> X acceleration in body frame </summary>
            public Single x_acc;
            /// <summary> Y acceleration in body frame </summary>
            public Single y_acc;
            /// <summary> Z acceleration in body frame </summary>
            public Single z_acc;
            /// <summary> X velocity in body frame </summary>
            public Single x_vel;
            /// <summary> Y velocity in body frame </summary>
            public Single y_vel;
            /// <summary> Z velocity in body frame </summary>
            public Single z_vel;
            /// <summary> X position in local frame </summary>
            public Single x_pos;
            /// <summary> Y position in local frame </summary>
            public Single y_pos;
            /// <summary> Z position in local frame </summary>
            public Single z_pos;
            /// <summary> Airspeed, set to -1 if unknown </summary>
            public Single airspeed;
            /// <summary> Variance of body velocity estimate </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public float vel_variance;
            /// <summary> Variance in local position </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public float pos_variance;
            /// <summary> The attitude, represented as Quaternion </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public float[] q;
            /// <summary> Angular rate in roll axis </summary>
            public Single roll_rate;
            /// <summary> Angular rate in pitch axis </summary>
            public Single pitch_rate;
            /// <summary> Angular rate in yaw axis </summary>
            public Single yaw_rate;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_battery_status_t
        {
            /// <summary> Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate </summary>
            public Int32 current_consumed;
            /// <summary> Consumed energy, in 100*Joules (integrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate </summary>
            public Int32 energy_consumed;
            /// <summary> Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature. </summary>
            public Int16 temperature;
            /// <summary> Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value. </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
            public UInt16[] voltages;
            /// <summary> Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current </summary>
            public Int16 current_battery;
            /// <summary> Battery ID </summary>
            public byte id;
            /// <summary> Function of the battery </summary>
            public byte battery_function;
            /// <summary> Type (chemistry) of the battery </summary>
            public byte type;
            /// <summary> Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery </summary>
            public byte battery_remaining;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_autopilot_version_t
        {
            /// <summary> bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum) </summary>
            public UInt64 capabilities;
            /// <summary> UID if provided by hardware </summary>
            public UInt64 uid;
            /// <summary> Firmware version number </summary>
            public UInt32 flight_sw_version;
            /// <summary> Middleware version number </summary>
            public UInt32 middleware_sw_version;
            /// <summary> Operating system version number </summary>
            public UInt32 os_sw_version;
            /// <summary> HW / board version (last 8 bytes should be silicon ID, if any) </summary>
            public UInt32 board_version;
            /// <summary> ID of the board vendor </summary>
            public UInt16 vendor_id;
            /// <summary> ID of the product </summary>
            public UInt16 product_id;
            /// <summary> Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases. </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public byte[] flight_custom_version;
            /// <summary> Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases. </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public byte[] middleware_custom_version;
            /// <summary> Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases. </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public byte[] os_custom_version;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_landing_target_t
        {
            /// <summary> Timestamp (micros since boot or Unix epoch) </summary>
            public UInt64 time_usec;
            /// <summary> X-axis angular offset (in radians) of the target from the center of the image </summary>
            public Single angle_x;
            /// <summary> Y-axis angular offset (in radians) of the target from the center of the image </summary>
            public Single angle_y;
            /// <summary> Distance to the target from the vehicle in meters </summary>
            public Single distance;
            /// <summary> Size in radians of target along x-axis </summary>
            public Single size_x;
            /// <summary> Size in radians of target along y-axis </summary>
            public Single size_y;
            /// <summary> The ID of the target if multiple targets are present </summary>
            public byte target_num;
            /// <summary> MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc. </summary>
            public byte frame;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_vibration_t
        {
            /// <summary> Timestamp (micros since boot or Unix epoch) </summary>
            public UInt64 time_usec;
            /// <summary> Vibration levels on X-axis </summary>
            public Single vibration_x;
            /// <summary> Vibration levels on Y-axis </summary>
            public Single vibration_y;
            /// <summary> Vibration levels on Z-axis </summary>
            public Single vibration_z;
            /// <summary> first accelerometer clipping count </summary>
            public UInt32 clipping_0;
            /// <summary> second accelerometer clipping count </summary>
            public UInt32 clipping_1;
            /// <summary> third accelerometer clipping count </summary>
            public UInt32 clipping_2;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_home_position_t
        {
            /// <summary> Latitude (WGS84), in degrees * 1E7 </summary>
            public Int32 latitude;
            /// <summary> Longitude (WGS84, in degrees * 1E7 </summary>
            public Int32 longitude;
            /// <summary> Altitude (AMSL), in meters * 1000 (positive for up) </summary>
            public Int32 altitude;
            /// <summary> Local X position of this position in the local coordinate frame </summary>
            public Single x;
            /// <summary> Local Y position of this position in the local coordinate frame </summary>
            public Single y;
            /// <summary> Local Z position of this position in the local coordinate frame </summary>
            public Single z;
            /// <summary> World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public float[] q;

            /// <summary> Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone. </summary>
            public Single approach_x;
            /// <summary> Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone. </summary>
            public Single approach_y;
            /// <summary> Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone. </summary>
            public Single approach_z;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_set_home_position_t
        {
            /// <summary> Latitude (WGS84), in degrees * 1E7 </summary>
            public Int32 latitude;
            /// <summary> Longitude (WGS84, in degrees * 1E7 </summary>
            public Int32 longitude;
            /// <summary> Altitude (AMSL), in meters * 1000 (positive for up) </summary>
            public Int32 altitude;
            /// <summary> Local X position of this position in the local coordinate frame </summary>
            public Single x;
            /// <summary> Local Y position of this position in the local coordinate frame </summary>
            public Single y;
            /// <summary> Local Z position of this position in the local coordinate frame </summary>
            public Single z;
            /// <summary> World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public float[] q;

            /// <summary> Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone. </summary>
            public Single approach_x;
            /// <summary> Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone. </summary>
            public Single approach_y;
            /// <summary> Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone. </summary>
            public Single approach_z;
            /// <summary> System ID. </summary>
            public byte target_system;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_message_interval_t
        {
            /// <summary> The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent. </summary>
            public Int32 interval_us;
            /// <summary> The ID of the requested MAVLink message. v1.0 is limited to 254 messages. </summary>
            public UInt16 message_id;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_extended_sys_state_t
        {
            /// <summary> The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration. </summary>
            public byte vtol_state;
            /// <summary> The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown. </summary>
            public byte landed_state;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_adsb_vehicle_t
        {
            /// <summary> ICAO address </summary>
            public UInt32 ICAO_address;
            /// <summary> Latitude, expressed as degrees * 1E7 </summary>
            public Int32 lat;
            /// <summary> Longitude, expressed as degrees * 1E7 </summary>
            public Int32 lon;
            /// <summary> Altitude(ASL) in millimeters </summary>
            public Int32 altitude;
            /// <summary> Course over ground in centidegrees </summary>
            public UInt16 heading;
            /// <summary> The horizontal velocity in centimeters/second </summary>
            public UInt16 hor_velocity;
            /// <summary> The vertical velocity in centimeters/second, positive is up </summary>
            public Int16 ver_velocity;
            /// <summary> Flags to indicate various statuses including valid data fields </summary>
            public UInt16 flags;
            /// <summary> Squawk code </summary>
            public UInt16 squawk;
            /// <summary> Type from ADSB_ALTITUDE_TYPE enum </summary>
            public byte altitude_type;
            /// <summary> The callsign, 8+null </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
            public byte[] callsign;
            /// <summary> Type from ADSB_EMITTER_TYPE enum </summary>
            public byte emitter_type;
            /// <summary> Time since last communication in seconds </summary>
            public byte tslc;

        };


        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct mavlink_collision_t
        {
            // Unique identifier, domain based on src field
            public UInt32 id;
            // Estimated time until collision occurs (seconds)
            public float time_to_minimum_delta;
            // Closest vertical distance in meters between vehicle and object
            public float altitude_minimum_delta;
            // Closest horizontal distance in meteres between vehicle and object
            public float horizontal_minimum_delta;
            // Collision data source
            public byte src;
            // Action that is being taken to avoid this collision
            public byte action;
            // How concerned the aircraft is about this collision
            public byte threat_level;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_v2_extension_t
        {
            /// <summary> A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase. </summary>
            public UInt16 message_type;
            /// <summary> Network ID (0 for broadcast) </summary>
            public byte target_network;
            /// <summary> System ID (0 for broadcast) </summary>
            public byte target_system;
            /// <summary> Component ID (0 for broadcast) </summary>
            public byte target_component;
            /// <summary> Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification. </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 249)]
            public byte[] payload;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_memory_vect_t
        {
            /// <summary> Starting address of the debug variables </summary>
            public UInt16 address;
            /// <summary> Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below </summary>
            public byte ver;
            /// <summary> Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14 </summary>
            public byte type;
            /// <summary> Memory contents at specified address </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public byte[] value;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_debug_vect_t
        {
            /// <summary> Timestamp </summary>
            public UInt64 time_usec;
            /// <summary> x </summary>
            public Single x;
            /// <summary> y </summary>
            public Single y;
            /// <summary> z </summary>
            public Single z;
            /// <summary> Name </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
            public byte[] name;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_named_value_float_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Floating point value </summary>
            public Single value;
            /// <summary> Name of the debug variable </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
            public byte[] name;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_named_value_int_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> Signed integer value </summary>
            public Int32 value;
            /// <summary> Name of the debug variable </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
            public byte[] name;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_statustext_t
        {
            /// <summary> Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY. </summary>
            public byte severity;
            /// <summary> Status text message, without null termination character </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 50)]
            public byte[] text;

        };


        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_debug_t
        {
            /// <summary> Timestamp (milliseconds since system boot) </summary>
            public UInt32 time_boot_ms;
            /// <summary> DEBUG value </summary>
            public Single value;
            /// <summary> index of debug variable </summary>
            public byte ind;

        };

        // custom message from the simulator 
        [StructLayout(LayoutKind.Sequential)]
        public struct mavlink_telemetry
        {
            public const int MessageId = 204;
            public int messages_sent;        // number of messages sent since the last telemetry message
            public int messages_received;    // number of messages received since the last telemetry message
            public int messages_handled;     // number of messages handled since the last telemetry message
            public int crc_errors;           // # crc errors detected in mavlink stream since the last telemetry message
            public int handler_microseconds; // total time spent in the handlers in microseconds since the last telemetry message
            public int render_time;          // total time spent rendering frames since the last telemetry message
            public int wifi_rssi;            // if this device is communicating over wifi this is the signal strength.
            public int udpate_rate;          // rate at which update() is being called on MavLinkMultiRotorApi
            public int actuation_delay;      // delay from HIL_SENSOR to HIL_ACTUATORCONTROLS response
            public int sensor_rate;          // rate we are sending HIL_SENSOR messages
            public int lock_step_resets;     // total number of lock_step resets
            public int update_time;          // avg time spent inside MavLinkMultiRotorApi::update method.
        };
    }
}