// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "MavLinkMessages.hpp"
using namespace mavlinkcom;

int MavLinkHeartbeat::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->custom_mode), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->type), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->autopilot), 5);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->base_mode), 6);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->system_status), 7);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->mavlink_version), 8);
    return 9;
}

int MavLinkHeartbeat::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->custom_mode), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->type), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->autopilot), 5);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->base_mode), 6);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->system_status), 7);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->mavlink_version), 8);
    return 9;
}

int MavLinkSysStatus::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->onboard_control_sensors_present), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->onboard_control_sensors_enabled), 4);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->onboard_control_sensors_health), 8);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->load), 12);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->voltage_battery), 14);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->current_battery), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->drop_rate_comm), 18);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->errors_comm), 20);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->errors_count1), 22);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->errors_count2), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->errors_count3), 26);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->errors_count4), 28);
    pack_int8_t(buffer, reinterpret_cast<const int8_t*>(&this->battery_remaining), 30);
    return 31;
}

int MavLinkSysStatus::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->onboard_control_sensors_present), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->onboard_control_sensors_enabled), 4);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->onboard_control_sensors_health), 8);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->load), 12);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->voltage_battery), 14);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->current_battery), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->drop_rate_comm), 18);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->errors_comm), 20);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->errors_count1), 22);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->errors_count2), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->errors_count3), 26);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->errors_count4), 28);
    unpack_int8_t(buffer, reinterpret_cast<int8_t*>(&this->battery_remaining), 30);
    return 31;
}

int MavLinkSystemTime::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_unix_usec), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 8);
    return 12;
}

int MavLinkSystemTime::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_unix_usec), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 8);
    return 12;
}

int MavLinkPing::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->seq), 8);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 12);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 13);
    return 14;
}

int MavLinkPing::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->seq), 8);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 12);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 13);
    return 14;
}

int MavLinkChangeOperatorControl::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->control_request), 1);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->version), 2);
    pack_char_array(25, buffer, reinterpret_cast<const char*>(&this->passkey[0]), 3);
    return 28;
}

int MavLinkChangeOperatorControl::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->control_request), 1);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->version), 2);
    unpack_char_array(25, buffer, reinterpret_cast<char*>(&this->passkey[0]), 3);
    return 28;
}

int MavLinkChangeOperatorControlAck::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->gcs_system_id), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->control_request), 1);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->ack), 2);
    return 3;
}

int MavLinkChangeOperatorControlAck::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->gcs_system_id), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->control_request), 1);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->ack), 2);
    return 3;
}

int MavLinkAuthKey::pack(char* buffer) const {
    pack_char_array(32, buffer, reinterpret_cast<const char*>(&this->key[0]), 0);
    return 32;
}

int MavLinkAuthKey::unpack(const char* buffer) {
    unpack_char_array(32, buffer, reinterpret_cast<char*>(&this->key[0]), 0);
    return 32;
}

int MavLinkSetMode::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->custom_mode), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->base_mode), 5);
    return 6;
}

int MavLinkSetMode::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->custom_mode), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->base_mode), 5);
    return 6;
}

int MavLinkParamRequestRead::pack(char* buffer) const {
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->param_index), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 3);
    pack_char_array(16, buffer, reinterpret_cast<const char*>(&this->param_id[0]), 4);
    return 20;
}

int MavLinkParamRequestRead::unpack(const char* buffer) {
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->param_index), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 3);
    unpack_char_array(16, buffer, reinterpret_cast<char*>(&this->param_id[0]), 4);
    return 20;
}

int MavLinkParamRequestList::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkParamRequestList::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkParamValue::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->param_value), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->param_count), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->param_index), 6);
    pack_char_array(16, buffer, reinterpret_cast<const char*>(&this->param_id[0]), 8);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->param_type), 24);
    return 25;
}

int MavLinkParamValue::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->param_value), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->param_count), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->param_index), 6);
    unpack_char_array(16, buffer, reinterpret_cast<char*>(&this->param_id[0]), 8);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->param_type), 24);
    return 25;
}

int MavLinkParamSet::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->param_value), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 5);
    pack_char_array(16, buffer, reinterpret_cast<const char*>(&this->param_id[0]), 6);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->param_type), 22);
    return 23;
}

int MavLinkParamSet::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->param_value), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 5);
    unpack_char_array(16, buffer, reinterpret_cast<char*>(&this->param_id[0]), 6);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->param_type), 22);
    return 23;
}

int MavLinkGpsRawInt::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->alt), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->eph), 20);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->epv), 22);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->vel), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->cog), 26);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->fix_type), 28);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->satellites_visible), 29);
    return 30;
}

int MavLinkGpsRawInt::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->alt), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->eph), 20);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->epv), 22);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->vel), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->cog), 26);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->fix_type), 28);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->satellites_visible), 29);
    return 30;
}

int MavLinkGpsStatus::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->satellites_visible), 0);
    pack_uint8_t_array(20, buffer, reinterpret_cast<const uint8_t*>(&this->satellite_prn[0]), 1);
    pack_uint8_t_array(20, buffer, reinterpret_cast<const uint8_t*>(&this->satellite_used[0]), 21);
    pack_uint8_t_array(20, buffer, reinterpret_cast<const uint8_t*>(&this->satellite_elevation[0]), 41);
    pack_uint8_t_array(20, buffer, reinterpret_cast<const uint8_t*>(&this->satellite_azimuth[0]), 61);
    pack_uint8_t_array(20, buffer, reinterpret_cast<const uint8_t*>(&this->satellite_snr[0]), 81);
    return 101;
}

int MavLinkGpsStatus::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->satellites_visible), 0);
    unpack_uint8_t_array(20, buffer, reinterpret_cast<uint8_t*>(&this->satellite_prn[0]), 1);
    unpack_uint8_t_array(20, buffer, reinterpret_cast<uint8_t*>(&this->satellite_used[0]), 21);
    unpack_uint8_t_array(20, buffer, reinterpret_cast<uint8_t*>(&this->satellite_elevation[0]), 41);
    unpack_uint8_t_array(20, buffer, reinterpret_cast<uint8_t*>(&this->satellite_azimuth[0]), 61);
    unpack_uint8_t_array(20, buffer, reinterpret_cast<uint8_t*>(&this->satellite_snr[0]), 81);
    return 101;
}

int MavLinkScaledImu::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xacc), 4);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->yacc), 6);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zacc), 8);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xgyro), 10);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ygyro), 12);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zgyro), 14);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xmag), 16);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ymag), 18);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zmag), 20);
    return 22;
}

int MavLinkScaledImu::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xacc), 4);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->yacc), 6);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zacc), 8);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xgyro), 10);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ygyro), 12);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zgyro), 14);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xmag), 16);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ymag), 18);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zmag), 20);
    return 22;
}

int MavLinkRawImu::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xacc), 8);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->yacc), 10);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zacc), 12);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xgyro), 14);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ygyro), 16);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zgyro), 18);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xmag), 20);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ymag), 22);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zmag), 24);
    return 26;
}

int MavLinkRawImu::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xacc), 8);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->yacc), 10);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zacc), 12);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xgyro), 14);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ygyro), 16);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zgyro), 18);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xmag), 20);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ymag), 22);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zmag), 24);
    return 26;
}

int MavLinkRawPressure::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->press_abs), 8);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->press_diff1), 10);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->press_diff2), 12);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->temperature), 14);
    return 16;
}

int MavLinkRawPressure::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->press_abs), 8);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->press_diff1), 10);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->press_diff2), 12);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->temperature), 14);
    return 16;
}

int MavLinkScaledPressure::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->press_abs), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->press_diff), 8);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->temperature), 12);
    return 14;
}

int MavLinkScaledPressure::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->press_abs), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->press_diff), 8);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->temperature), 12);
    return 14;
}

int MavLinkAttitude::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->rollspeed), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitchspeed), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yawspeed), 24);
    return 28;
}

int MavLinkAttitude::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->rollspeed), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitchspeed), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yawspeed), 24);
    return 28;
}

int MavLinkAttitudeQuaternion::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->q1), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->q2), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->q3), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->q4), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->rollspeed), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitchspeed), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yawspeed), 28);
    return 32;
}

int MavLinkAttitudeQuaternion::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->q1), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->q2), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->q3), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->q4), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->rollspeed), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitchspeed), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yawspeed), 28);
    return 32;
}

int MavLinkLocalPositionNed::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vx), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vy), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vz), 24);
    return 28;
}

int MavLinkLocalPositionNed::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vx), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vy), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vz), 24);
    return 28;
}

int MavLinkGlobalPositionInt::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->alt), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->relative_alt), 16);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vx), 20);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vy), 22);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vz), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->hdg), 26);
    return 28;
}

int MavLinkGlobalPositionInt::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->alt), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->relative_alt), 16);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vx), 20);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vy), 22);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vz), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->hdg), 26);
    return 28;
}

int MavLinkRcChannelsScaled::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->chan1_scaled), 4);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->chan2_scaled), 6);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->chan3_scaled), 8);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->chan4_scaled), 10);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->chan5_scaled), 12);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->chan6_scaled), 14);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->chan7_scaled), 16);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->chan8_scaled), 18);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->port), 20);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rssi), 21);
    return 22;
}

int MavLinkRcChannelsScaled::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->chan1_scaled), 4);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->chan2_scaled), 6);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->chan3_scaled), 8);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->chan4_scaled), 10);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->chan5_scaled), 12);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->chan6_scaled), 14);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->chan7_scaled), 16);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->chan8_scaled), 18);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->port), 20);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rssi), 21);
    return 22;
}

int MavLinkRcChannelsRaw::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan1_raw), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan2_raw), 6);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan3_raw), 8);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan4_raw), 10);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan5_raw), 12);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan6_raw), 14);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan7_raw), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan8_raw), 18);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->port), 20);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rssi), 21);
    return 22;
}

int MavLinkRcChannelsRaw::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan1_raw), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan2_raw), 6);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan3_raw), 8);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan4_raw), 10);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan5_raw), 12);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan6_raw), 14);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan7_raw), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan8_raw), 18);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->port), 20);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rssi), 21);
    return 22;
}

int MavLinkServoOutputRaw::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_usec), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo1_raw), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo2_raw), 6);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo3_raw), 8);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo4_raw), 10);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo5_raw), 12);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo6_raw), 14);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo7_raw), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo8_raw), 18);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo9_raw), 20);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo10_raw), 22);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo11_raw), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo12_raw), 26);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo13_raw), 28);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo14_raw), 30);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo15_raw), 32);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->servo16_raw), 34);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->port), 36);
    return 37;
}

int MavLinkServoOutputRaw::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_usec), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo1_raw), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo2_raw), 6);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo3_raw), 8);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo4_raw), 10);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo5_raw), 12);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo6_raw), 14);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo7_raw), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo8_raw), 18);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo9_raw), 20);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo10_raw), 22);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo11_raw), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo12_raw), 26);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo13_raw), 28);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo14_raw), 30);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo15_raw), 32);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->servo16_raw), 34);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->port), 36);
    return 37;
}

int MavLinkMissionRequestPartialList::pack(char* buffer) const {
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->start_index), 0);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->end_index), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 5);
    return 6;
}

int MavLinkMissionRequestPartialList::unpack(const char* buffer) {
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->start_index), 0);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->end_index), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 5);
    return 6;
}

int MavLinkMissionWritePartialList::pack(char* buffer) const {
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->start_index), 0);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->end_index), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 5);
    return 6;
}

int MavLinkMissionWritePartialList::unpack(const char* buffer) {
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->start_index), 0);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->end_index), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 5);
    return 6;
}

int MavLinkMissionItem::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->param1), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param2), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param3), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param4), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seq), 28);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->command), 30);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 33);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->frame), 34);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->current), 35);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->autocontinue), 36);
    return 37;
}

int MavLinkMissionItem::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->param1), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param2), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param3), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param4), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seq), 28);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->command), 30);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 33);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->frame), 34);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->current), 35);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->autocontinue), 36);
    return 37;
}

int MavLinkMissionRequest::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seq), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 3);
    return 4;
}

int MavLinkMissionRequest::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seq), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 3);
    return 4;
}

int MavLinkMissionSetCurrent::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seq), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 3);
    return 4;
}

int MavLinkMissionSetCurrent::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seq), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 3);
    return 4;
}

int MavLinkMissionCurrent::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seq), 0);
    return 2;
}

int MavLinkMissionCurrent::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seq), 0);
    return 2;
}

int MavLinkMissionRequestList::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkMissionRequestList::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkMissionCount::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->count), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 3);
    return 4;
}

int MavLinkMissionCount::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->count), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 3);
    return 4;
}

int MavLinkMissionClearAll::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkMissionClearAll::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkMissionItemReached::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seq), 0);
    return 2;
}

int MavLinkMissionItemReached::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seq), 0);
    return 2;
}

int MavLinkMissionAck::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 1);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->type), 2);
    return 3;
}

int MavLinkMissionAck::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 1);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->type), 2);
    return 3;
}

int MavLinkSetGpsGlobalOrigin::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->latitude), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->longitude), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->altitude), 8);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 12);
    return 13;
}

int MavLinkSetGpsGlobalOrigin::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->latitude), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->longitude), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->altitude), 8);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 12);
    return 13;
}

int MavLinkGpsGlobalOrigin::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->latitude), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->longitude), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->altitude), 8);
    return 12;
}

int MavLinkGpsGlobalOrigin::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->latitude), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->longitude), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->altitude), 8);
    return 12;
}

int MavLinkParamMapRc::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->param_value0), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->scale), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param_value_min), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param_value_max), 12);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->param_index), 16);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 18);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 19);
    pack_char_array(16, buffer, reinterpret_cast<const char*>(&this->param_id[0]), 20);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->parameter_rc_channel_index), 36);
    return 37;
}

int MavLinkParamMapRc::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->param_value0), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->scale), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param_value_min), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param_value_max), 12);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->param_index), 16);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 18);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 19);
    unpack_char_array(16, buffer, reinterpret_cast<char*>(&this->param_id[0]), 20);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->parameter_rc_channel_index), 36);
    return 37;
}

int MavLinkMissionRequestInt::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seq), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 3);
    return 4;
}

int MavLinkMissionRequestInt::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seq), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 3);
    return 4;
}

int MavLinkSafetySetAllowedArea::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->p1x), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p1y), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p1z), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p2x), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p2y), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p2z), 20);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 24);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 25);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->frame), 26);
    return 27;
}

int MavLinkSafetySetAllowedArea::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->p1x), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p1y), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p1z), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p2x), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p2y), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p2z), 20);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 24);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 25);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->frame), 26);
    return 27;
}

int MavLinkSafetyAllowedArea::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->p1x), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p1y), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p1z), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p2x), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p2y), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->p2z), 20);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->frame), 24);
    return 25;
}

int MavLinkSafetyAllowedArea::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->p1x), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p1y), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p1z), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p2x), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p2y), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->p2z), 20);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->frame), 24);
    return 25;
}

int MavLinkAttitudeQuaternionCov::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->q[0]), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->rollspeed), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitchspeed), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yawspeed), 28);
    pack_float_array(9, buffer, reinterpret_cast<const float*>(&this->covariance[0]), 32);
    return 68;
}

int MavLinkAttitudeQuaternionCov::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->q[0]), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->rollspeed), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitchspeed), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yawspeed), 28);
    unpack_float_array(9, buffer, reinterpret_cast<float*>(&this->covariance[0]), 32);
    return 68;
}

int MavLinkNavControllerOutput::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->nav_roll), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->nav_pitch), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->alt_error), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->aspd_error), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->xtrack_error), 16);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->nav_bearing), 20);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->target_bearing), 22);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->wp_dist), 24);
    return 26;
}

int MavLinkNavControllerOutput::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->nav_roll), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->nav_pitch), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->alt_error), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->aspd_error), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->xtrack_error), 16);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->nav_bearing), 20);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->target_bearing), 22);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->wp_dist), 24);
    return 26;
}

int MavLinkGlobalPositionIntCov::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_utc), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 16);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->alt), 20);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->relative_alt), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vx), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vy), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vz), 36);
    pack_float_array(36, buffer, reinterpret_cast<const float*>(&this->covariance[0]), 40);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->estimator_type), 184);
    return 185;
}

int MavLinkGlobalPositionIntCov::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_utc), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 16);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->alt), 20);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->relative_alt), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vx), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vy), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vz), 36);
    unpack_float_array(36, buffer, reinterpret_cast<float*>(&this->covariance[0]), 40);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->estimator_type), 184);
    return 185;
}

int MavLinkLocalPositionNedCov::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_utc), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vx), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vy), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vz), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ax), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ay), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->az), 44);
    pack_float_array(45, buffer, reinterpret_cast<const float*>(&this->covariance[0]), 48);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->estimator_type), 228);
    return 229;
}

int MavLinkLocalPositionNedCov::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_utc), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vx), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vy), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vz), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ax), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ay), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->az), 44);
    unpack_float_array(45, buffer, reinterpret_cast<float*>(&this->covariance[0]), 48);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->estimator_type), 228);
    return 229;
}

int MavLinkRcChannels::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan1_raw), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan2_raw), 6);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan3_raw), 8);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan4_raw), 10);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan5_raw), 12);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan6_raw), 14);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan7_raw), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan8_raw), 18);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan9_raw), 20);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan10_raw), 22);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan11_raw), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan12_raw), 26);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan13_raw), 28);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan14_raw), 30);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan15_raw), 32);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan16_raw), 34);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan17_raw), 36);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan18_raw), 38);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->chancount), 40);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rssi), 41);
    return 42;
}

int MavLinkRcChannels::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan1_raw), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan2_raw), 6);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan3_raw), 8);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan4_raw), 10);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan5_raw), 12);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan6_raw), 14);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan7_raw), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan8_raw), 18);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan9_raw), 20);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan10_raw), 22);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan11_raw), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan12_raw), 26);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan13_raw), 28);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan14_raw), 30);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan15_raw), 32);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan16_raw), 34);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan17_raw), 36);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan18_raw), 38);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->chancount), 40);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rssi), 41);
    return 42;
}

int MavLinkRequestDataStream::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->req_message_rate), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 3);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->req_stream_id), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->start_stop), 5);
    return 6;
}

int MavLinkRequestDataStream::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->req_message_rate), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 3);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->req_stream_id), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->start_stop), 5);
    return 6;
}

int MavLinkDataStream::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->message_rate), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->stream_id), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->on_off), 3);
    return 4;
}

int MavLinkDataStream::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->message_rate), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->stream_id), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->on_off), 3);
    return 4;
}

int MavLinkManualControl::pack(char* buffer) const {
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->x), 0);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->y), 2);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->z), 4);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->r), 6);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->buttons), 8);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target), 10);
    return 11;
}

int MavLinkManualControl::unpack(const char* buffer) {
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->x), 0);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->y), 2);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->z), 4);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->r), 6);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->buttons), 8);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target), 10);
    return 11;
}

int MavLinkRcChannelsOverride::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan1_raw), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan2_raw), 2);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan3_raw), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan4_raw), 6);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan5_raw), 8);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan6_raw), 10);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan7_raw), 12);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan8_raw), 14);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 16);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 17);
    return 18;
}

int MavLinkRcChannelsOverride::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan1_raw), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan2_raw), 2);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan3_raw), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan4_raw), 6);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan5_raw), 8);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan6_raw), 10);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan7_raw), 12);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan8_raw), 14);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 16);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 17);
    return 18;
}

int MavLinkMissionItemInt::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->param1), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param2), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param3), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param4), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->x), 16);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->y), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seq), 28);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->command), 30);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 33);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->frame), 34);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->current), 35);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->autocontinue), 36);
    return 37;
}

int MavLinkMissionItemInt::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->param1), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param2), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param3), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param4), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->x), 16);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->y), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seq), 28);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->command), 30);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 33);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->frame), 34);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->current), 35);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->autocontinue), 36);
    return 37;
}

int MavLinkVfrHud::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->airspeed), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->groundspeed), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->alt), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->climb), 12);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->heading), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->throttle), 18);
    return 20;
}

int MavLinkVfrHud::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->airspeed), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->groundspeed), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->alt), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->climb), 12);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->heading), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->throttle), 18);
    return 20;
}

int MavLinkCommandInt::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->param1), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param2), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param3), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param4), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->x), 16);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->y), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->command), 28);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 30);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 31);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->frame), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->current), 33);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->autocontinue), 34);
    return 35;
}

int MavLinkCommandInt::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->param1), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param2), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param3), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param4), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->x), 16);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->y), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->command), 28);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 30);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 31);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->frame), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->current), 33);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->autocontinue), 34);
    return 35;
}

int MavLinkCommandLong::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->param1), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param2), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param3), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param4), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param5), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param6), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->param7), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->command), 28);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 30);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 31);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->confirmation), 32);
    return 33;
}

int MavLinkCommandLong::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->param1), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param2), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param3), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param4), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param5), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param6), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->param7), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->command), 28);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 30);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 31);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->confirmation), 32);
    return 33;
}

int MavLinkCommandAck::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->command), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->result), 2);
    return 3;
}

int MavLinkCommandAck::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->command), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->result), 2);
    return 3;
}

int MavLinkManualSetpoint::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->thrust), 16);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->mode_switch), 20);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->manual_override_switch), 21);
    return 22;
}

int MavLinkManualSetpoint::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->thrust), 16);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->mode_switch), 20);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->manual_override_switch), 21);
    return 22;
}

int MavLinkSetAttitudeTarget::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->q[0]), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->body_roll_rate), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->body_pitch_rate), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->body_yaw_rate), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->thrust), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 36);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 37);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->type_mask), 38);
    return 39;
}

int MavLinkSetAttitudeTarget::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->q[0]), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->body_roll_rate), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->body_pitch_rate), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->body_yaw_rate), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->thrust), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 36);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 37);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->type_mask), 38);
    return 39;
}

int MavLinkAttitudeTarget::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->q[0]), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->body_roll_rate), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->body_pitch_rate), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->body_yaw_rate), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->thrust), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->type_mask), 36);
    return 37;
}

int MavLinkAttitudeTarget::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->q[0]), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->body_roll_rate), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->body_pitch_rate), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->body_yaw_rate), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->thrust), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->type_mask), 36);
    return 37;
}

int MavLinkSetPositionTargetLocalNed::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vx), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vy), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vz), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afx), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afy), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afz), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw_rate), 44);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->type_mask), 48);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 50);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 51);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->coordinate_frame), 52);
    return 53;
}

int MavLinkSetPositionTargetLocalNed::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vx), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vy), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vz), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afx), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afy), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afz), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw_rate), 44);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->type_mask), 48);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 50);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 51);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->coordinate_frame), 52);
    return 53;
}

int MavLinkPositionTargetLocalNed::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vx), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vy), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vz), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afx), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afy), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afz), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw_rate), 44);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->type_mask), 48);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->coordinate_frame), 50);
    return 51;
}

int MavLinkPositionTargetLocalNed::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vx), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vy), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vz), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afx), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afy), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afz), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw_rate), 44);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->type_mask), 48);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->coordinate_frame), 50);
    return 51;
}

int MavLinkSetPositionTargetGlobalInt::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat_int), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon_int), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->alt), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vx), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vy), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vz), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afx), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afy), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afz), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw_rate), 44);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->type_mask), 48);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 50);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 51);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->coordinate_frame), 52);
    return 53;
}

int MavLinkSetPositionTargetGlobalInt::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat_int), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon_int), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->alt), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vx), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vy), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vz), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afx), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afy), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afz), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw_rate), 44);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->type_mask), 48);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 50);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 51);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->coordinate_frame), 52);
    return 53;
}

int MavLinkPositionTargetGlobalInt::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat_int), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon_int), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->alt), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vx), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vy), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vz), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afx), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afy), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->afz), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw_rate), 44);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->type_mask), 48);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->coordinate_frame), 50);
    return 51;
}

int MavLinkPositionTargetGlobalInt::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat_int), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon_int), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->alt), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vx), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vy), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vz), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afx), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afy), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->afz), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw_rate), 44);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->type_mask), 48);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->coordinate_frame), 50);
    return 51;
}

int MavLinkLocalPositionNedSystemGlobalOffset::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 24);
    return 28;
}

int MavLinkLocalPositionNedSystemGlobalOffset::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 24);
    return 28;
}

int MavLinkHilState::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->rollspeed), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitchspeed), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yawspeed), 28);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 32);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 36);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->alt), 40);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vx), 44);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vy), 46);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vz), 48);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xacc), 50);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->yacc), 52);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zacc), 54);
    return 56;
}

int MavLinkHilState::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->rollspeed), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitchspeed), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yawspeed), 28);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 32);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 36);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->alt), 40);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vx), 44);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vy), 46);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vz), 48);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xacc), 50);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->yacc), 52);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zacc), 54);
    return 56;
}

int MavLinkHilControls::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll_ailerons), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch_elevator), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw_rudder), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->throttle), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->aux1), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->aux2), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->aux3), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->aux4), 36);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->mode), 40);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->nav_mode), 41);
    return 42;
}

int MavLinkHilControls::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll_ailerons), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch_elevator), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw_rudder), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->throttle), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->aux1), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->aux2), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->aux3), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->aux4), 36);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->mode), 40);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->nav_mode), 41);
    return 42;
}

int MavLinkHilRcInputsRaw::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan1_raw), 8);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan2_raw), 10);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan3_raw), 12);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan4_raw), 14);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan5_raw), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan6_raw), 18);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan7_raw), 20);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan8_raw), 22);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan9_raw), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan10_raw), 26);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan11_raw), 28);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->chan12_raw), 30);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rssi), 32);
    return 33;
}

int MavLinkHilRcInputsRaw::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan1_raw), 8);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan2_raw), 10);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan3_raw), 12);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan4_raw), 14);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan5_raw), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan6_raw), 18);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan7_raw), 20);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan8_raw), 22);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan9_raw), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan10_raw), 26);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan11_raw), 28);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->chan12_raw), 30);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rssi), 32);
    return 33;
}

int MavLinkHilActuatorControls::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->flags), 8);
    pack_float_array(16, buffer, reinterpret_cast<const float*>(&this->controls[0]), 16);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->mode), 80);
    return 81;
}

int MavLinkHilActuatorControls::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->flags), 8);
    unpack_float_array(16, buffer, reinterpret_cast<float*>(&this->controls[0]), 16);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->mode), 80);
    return 81;
}

int MavLinkOpticalFlow::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->flow_comp_m_x), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->flow_comp_m_y), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ground_distance), 16);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->flow_x), 20);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->flow_y), 22);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->sensor_id), 24);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->quality), 25);
    return 26;
}

int MavLinkOpticalFlow::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->flow_comp_m_x), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->flow_comp_m_y), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ground_distance), 16);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->flow_x), 20);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->flow_y), 22);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->sensor_id), 24);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->quality), 25);
    return 26;
}

int MavLinkGlobalVisionPositionEstimate::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 28);
    return 32;
}

int MavLinkGlobalVisionPositionEstimate::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 28);
    return 32;
}

int MavLinkVisionPositionEstimate::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 28);
    return 32;
}

int MavLinkVisionPositionEstimate::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 28);
    return 32;
}

int MavLinkVisionSpeedEstimate::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 16);
    return 20;
}

int MavLinkVisionSpeedEstimate::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 16);
    return 20;
}

int MavLinkViconPositionEstimate::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 28);
    return 32;
}

int MavLinkViconPositionEstimate::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 28);
    return 32;
}

int MavLinkHighresImu::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->xacc), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yacc), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->zacc), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->xgyro), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ygyro), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->zgyro), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->xmag), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ymag), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->zmag), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->abs_pressure), 44);
    pack_float(buffer, reinterpret_cast<const float*>(&this->diff_pressure), 48);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pressure_alt), 52);
    pack_float(buffer, reinterpret_cast<const float*>(&this->temperature), 56);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->fields_updated), 60);
    return 62;
}

int MavLinkHighresImu::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->xacc), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yacc), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->zacc), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->xgyro), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ygyro), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->zgyro), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->xmag), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ymag), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->zmag), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->abs_pressure), 44);
    unpack_float(buffer, reinterpret_cast<float*>(&this->diff_pressure), 48);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pressure_alt), 52);
    unpack_float(buffer, reinterpret_cast<float*>(&this->temperature), 56);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->fields_updated), 60);
    return 62;
}

int MavLinkOpticalFlowRad::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->integration_time_us), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_x), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_y), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_xgyro), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_ygyro), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_zgyro), 28);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_delta_distance_us), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->distance), 36);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->temperature), 40);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->sensor_id), 42);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->quality), 43);
    return 44;
}

int MavLinkOpticalFlowRad::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->integration_time_us), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_x), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_y), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_xgyro), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_ygyro), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_zgyro), 28);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_delta_distance_us), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->distance), 36);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->temperature), 40);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->sensor_id), 42);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->quality), 43);
    return 44;
}

int MavLinkHilSensor::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->xacc), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yacc), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->zacc), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->xgyro), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ygyro), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->zgyro), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->xmag), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ymag), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->zmag), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->abs_pressure), 44);
    pack_float(buffer, reinterpret_cast<const float*>(&this->diff_pressure), 48);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pressure_alt), 52);
    pack_float(buffer, reinterpret_cast<const float*>(&this->temperature), 56);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->fields_updated), 60);
    return 64;
}

int MavLinkHilSensor::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->xacc), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yacc), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->zacc), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->xgyro), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ygyro), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->zgyro), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->xmag), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ymag), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->zmag), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->abs_pressure), 44);
    unpack_float(buffer, reinterpret_cast<float*>(&this->diff_pressure), 48);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pressure_alt), 52);
    unpack_float(buffer, reinterpret_cast<float*>(&this->temperature), 56);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->fields_updated), 60);
    return 64;
}

int MavLinkSimState::pack(char* buffer) const {
    pack_float(buffer, reinterpret_cast<const float*>(&this->q1), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->q2), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->q3), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->q4), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->xacc), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yacc), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->zacc), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->xgyro), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ygyro), 44);
    pack_float(buffer, reinterpret_cast<const float*>(&this->zgyro), 48);
    pack_float(buffer, reinterpret_cast<const float*>(&this->lat), 52);
    pack_float(buffer, reinterpret_cast<const float*>(&this->lon), 56);
    pack_float(buffer, reinterpret_cast<const float*>(&this->alt), 60);
    pack_float(buffer, reinterpret_cast<const float*>(&this->std_dev_horz), 64);
    pack_float(buffer, reinterpret_cast<const float*>(&this->std_dev_vert), 68);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vn), 72);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ve), 76);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vd), 80);
    return 84;
}

int MavLinkSimState::unpack(const char* buffer) {
    unpack_float(buffer, reinterpret_cast<float*>(&this->q1), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->q2), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->q3), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->q4), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->xacc), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yacc), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->zacc), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->xgyro), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ygyro), 44);
    unpack_float(buffer, reinterpret_cast<float*>(&this->zgyro), 48);
    unpack_float(buffer, reinterpret_cast<float*>(&this->lat), 52);
    unpack_float(buffer, reinterpret_cast<float*>(&this->lon), 56);
    unpack_float(buffer, reinterpret_cast<float*>(&this->alt), 60);
    unpack_float(buffer, reinterpret_cast<float*>(&this->std_dev_horz), 64);
    unpack_float(buffer, reinterpret_cast<float*>(&this->std_dev_vert), 68);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vn), 72);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ve), 76);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vd), 80);
    return 84;
}

int MavLinkRadioStatus::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->rxerrors), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->fixed), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rssi), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->remrssi), 5);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->txbuf), 6);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->noise), 7);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->remnoise), 8);
    return 9;
}

int MavLinkRadioStatus::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->rxerrors), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->fixed), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rssi), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->remrssi), 5);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->txbuf), 6);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->noise), 7);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->remnoise), 8);
    return 9;
}

int MavLinkFileTransferProtocol::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_network), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 1);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 2);
    pack_uint8_t_array(251, buffer, reinterpret_cast<const uint8_t*>(&this->payload[0]), 3);
    return 254;
}

int MavLinkFileTransferProtocol::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_network), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 1);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 2);
    unpack_uint8_t_array(251, buffer, reinterpret_cast<uint8_t*>(&this->payload[0]), 3);
    return 254;
}

int MavLinkTimesync::pack(char* buffer) const {
    pack_int64_t(buffer, reinterpret_cast<const int64_t*>(&this->tc1), 0);
    pack_int64_t(buffer, reinterpret_cast<const int64_t*>(&this->ts1), 8);
    return 16;
}

int MavLinkTimesync::unpack(const char* buffer) {
    unpack_int64_t(buffer, reinterpret_cast<int64_t*>(&this->tc1), 0);
    unpack_int64_t(buffer, reinterpret_cast<int64_t*>(&this->ts1), 8);
    return 16;
}

int MavLinkCameraTrigger::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->seq), 8);
    return 12;
}

int MavLinkCameraTrigger::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->seq), 8);
    return 12;
}

int MavLinkHilGps::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->alt), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->eph), 20);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->epv), 22);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->vel), 24);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vn), 26);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ve), 28);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vd), 30);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->cog), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->fix_type), 34);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->satellites_visible), 35);
    return 36;
}

int MavLinkHilGps::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->alt), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->eph), 20);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->epv), 22);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->vel), 24);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vn), 26);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ve), 28);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vd), 30);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->cog), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->fix_type), 34);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->satellites_visible), 35);
    return 36;
}

int MavLinkHilOpticalFlow::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->integration_time_us), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_x), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_y), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_xgyro), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_ygyro), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->integrated_zgyro), 28);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_delta_distance_us), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->distance), 36);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->temperature), 40);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->sensor_id), 42);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->quality), 43);
    return 44;
}

int MavLinkHilOpticalFlow::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->integration_time_us), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_x), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_y), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_xgyro), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_ygyro), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->integrated_zgyro), 28);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_delta_distance_us), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->distance), 36);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->temperature), 40);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->sensor_id), 42);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->quality), 43);
    return 44;
}

int MavLinkHilStateQuaternion::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->attitude_quaternion[0]), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->rollspeed), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitchspeed), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yawspeed), 32);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 36);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 40);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->alt), 44);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vx), 48);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vy), 50);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->vz), 52);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->ind_airspeed), 54);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->true_airspeed), 56);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xacc), 58);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->yacc), 60);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zacc), 62);
    return 64;
}

int MavLinkHilStateQuaternion::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->attitude_quaternion[0]), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->rollspeed), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitchspeed), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yawspeed), 32);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 36);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 40);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->alt), 44);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vx), 48);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vy), 50);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->vz), 52);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->ind_airspeed), 54);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->true_airspeed), 56);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xacc), 58);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->yacc), 60);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zacc), 62);
    return 64;
}

int MavLinkScaledImu2::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xacc), 4);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->yacc), 6);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zacc), 8);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xgyro), 10);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ygyro), 12);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zgyro), 14);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xmag), 16);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ymag), 18);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zmag), 20);
    return 22;
}

int MavLinkScaledImu2::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xacc), 4);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->yacc), 6);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zacc), 8);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xgyro), 10);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ygyro), 12);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zgyro), 14);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xmag), 16);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ymag), 18);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zmag), 20);
    return 22;
}

int MavLinkLogRequestList::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->start), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->end), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 5);
    return 6;
}

int MavLinkLogRequestList::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->start), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->end), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 5);
    return 6;
}

int MavLinkLogEntry::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_utc), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->size), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->id), 8);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->num_logs), 10);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->last_log_num), 12);
    return 14;
}

int MavLinkLogEntry::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_utc), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->size), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->id), 8);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->num_logs), 10);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->last_log_num), 12);
    return 14;
}

int MavLinkLogRequestData::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->ofs), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->count), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->id), 8);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 10);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 11);
    return 12;
}

int MavLinkLogRequestData::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->ofs), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->count), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->id), 8);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 10);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 11);
    return 12;
}

int MavLinkLogData::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->ofs), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->id), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->count), 6);
    pack_uint8_t_array(90, buffer, reinterpret_cast<const uint8_t*>(&this->data[0]), 7);
    return 97;
}

int MavLinkLogData::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->ofs), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->id), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->count), 6);
    unpack_uint8_t_array(90, buffer, reinterpret_cast<uint8_t*>(&this->data[0]), 7);
    return 97;
}

int MavLinkLogErase::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkLogErase::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkLogRequestEnd::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkLogRequestEnd::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 1);
    return 2;
}

int MavLinkGpsInjectData::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 1);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->len), 2);
    pack_uint8_t_array(110, buffer, reinterpret_cast<const uint8_t*>(&this->data[0]), 3);
    return 113;
}

int MavLinkGpsInjectData::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 1);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->len), 2);
    unpack_uint8_t_array(110, buffer, reinterpret_cast<uint8_t*>(&this->data[0]), 3);
    return 113;
}

int MavLinkGps2Raw::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->alt), 16);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->dgps_age), 20);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->eph), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->epv), 26);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->vel), 28);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->cog), 30);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->fix_type), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->satellites_visible), 33);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->dgps_numch), 34);
    return 35;
}

int MavLinkGps2Raw::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->alt), 16);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->dgps_age), 20);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->eph), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->epv), 26);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->vel), 28);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->cog), 30);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->fix_type), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->satellites_visible), 33);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->dgps_numch), 34);
    return 35;
}

int MavLinkPowerStatus::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->Vcc), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->Vservo), 2);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->flags), 4);
    return 6;
}

int MavLinkPowerStatus::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->Vcc), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->Vservo), 2);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->flags), 4);
    return 6;
}

int MavLinkSerialControl::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->baudrate), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->timeout), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->device), 6);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->flags), 7);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->count), 8);
    pack_uint8_t_array(70, buffer, reinterpret_cast<const uint8_t*>(&this->data[0]), 9);
    return 79;
}

int MavLinkSerialControl::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->baudrate), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->timeout), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->device), 6);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->flags), 7);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->count), 8);
    unpack_uint8_t_array(70, buffer, reinterpret_cast<uint8_t*>(&this->data[0]), 9);
    return 79;
}

int MavLinkGpsRtk::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_last_baseline_ms), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->tow), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->baseline_a_mm), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->baseline_b_mm), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->baseline_c_mm), 16);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->accuracy), 20);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->iar_num_hypotheses), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->wn), 28);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rtk_receiver_id), 30);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rtk_health), 31);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rtk_rate), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->nsats), 33);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->baseline_coords_type), 34);
    return 35;
}

int MavLinkGpsRtk::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_last_baseline_ms), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->tow), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->baseline_a_mm), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->baseline_b_mm), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->baseline_c_mm), 16);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->accuracy), 20);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->iar_num_hypotheses), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->wn), 28);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rtk_receiver_id), 30);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rtk_health), 31);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rtk_rate), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->nsats), 33);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->baseline_coords_type), 34);
    return 35;
}

int MavLinkGps2Rtk::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_last_baseline_ms), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->tow), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->baseline_a_mm), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->baseline_b_mm), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->baseline_c_mm), 16);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->accuracy), 20);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->iar_num_hypotheses), 24);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->wn), 28);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rtk_receiver_id), 30);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rtk_health), 31);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->rtk_rate), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->nsats), 33);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->baseline_coords_type), 34);
    return 35;
}

int MavLinkGps2Rtk::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_last_baseline_ms), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->tow), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->baseline_a_mm), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->baseline_b_mm), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->baseline_c_mm), 16);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->accuracy), 20);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->iar_num_hypotheses), 24);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->wn), 28);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rtk_receiver_id), 30);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rtk_health), 31);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->rtk_rate), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->nsats), 33);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->baseline_coords_type), 34);
    return 35;
}

int MavLinkScaledImu3::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xacc), 4);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->yacc), 6);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zacc), 8);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xgyro), 10);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ygyro), 12);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zgyro), 14);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->xmag), 16);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ymag), 18);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->zmag), 20);
    return 22;
}

int MavLinkScaledImu3::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xacc), 4);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->yacc), 6);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zacc), 8);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xgyro), 10);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ygyro), 12);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zgyro), 14);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->xmag), 16);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ymag), 18);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->zmag), 20);
    return 22;
}

int MavLinkDataTransmissionHandshake::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->size), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->width), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->height), 6);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->packets), 8);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->type), 10);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->payload), 11);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->jpg_quality), 12);
    return 13;
}

int MavLinkDataTransmissionHandshake::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->size), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->width), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->height), 6);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->packets), 8);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->type), 10);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->payload), 11);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->jpg_quality), 12);
    return 13;
}

int MavLinkEncapsulatedData::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seqnr), 0);
    pack_uint8_t_array(253, buffer, reinterpret_cast<const uint8_t*>(&this->data[0]), 2);
    return 255;
}

int MavLinkEncapsulatedData::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seqnr), 0);
    unpack_uint8_t_array(253, buffer, reinterpret_cast<uint8_t*>(&this->data[0]), 2);
    return 255;
}

int MavLinkDistanceSensor::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->min_distance), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->max_distance), 6);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->current_distance), 8);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->type), 10);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->id), 11);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->orientation), 12);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->covariance), 13);
    return 14;
}

int MavLinkDistanceSensor::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->min_distance), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->max_distance), 6);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->current_distance), 8);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->type), 10);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->id), 11);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->orientation), 12);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->covariance), 13);
    return 14;
}

int MavLinkTerrainRequest::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->mask), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 12);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->grid_spacing), 16);
    return 18;
}

int MavLinkTerrainRequest::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->mask), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 12);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->grid_spacing), 16);
    return 18;
}

int MavLinkTerrainData::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 4);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->grid_spacing), 8);
    pack_int16_t_array(16, buffer, reinterpret_cast<const int16_t*>(&this->data[0]), 10);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->gridbit), 42);
    return 43;
}

int MavLinkTerrainData::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 4);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->grid_spacing), 8);
    unpack_int16_t_array(16, buffer, reinterpret_cast<int16_t*>(&this->data[0]), 10);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->gridbit), 42);
    return 43;
}

int MavLinkTerrainCheck::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 4);
    return 8;
}

int MavLinkTerrainCheck::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 4);
    return 8;
}

int MavLinkTerrainReport::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->terrain_height), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->current_height), 12);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->spacing), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->pending), 18);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->loaded), 20);
    return 22;
}

int MavLinkTerrainReport::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->terrain_height), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->current_height), 12);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->spacing), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->pending), 18);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->loaded), 20);
    return 22;
}

int MavLinkScaledPressure2::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->press_abs), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->press_diff), 8);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->temperature), 12);
    return 14;
}

int MavLinkScaledPressure2::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->press_abs), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->press_diff), 8);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->temperature), 12);
    return 14;
}

int MavLinkAttPosMocap::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->q[0]), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 32);
    return 36;
}

int MavLinkAttPosMocap::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->q[0]), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 32);
    return 36;
}

int MavLinkSetActuatorControlTarget::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float_array(8, buffer, reinterpret_cast<const float*>(&this->controls[0]), 8);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->group_mlx), 40);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 41);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 42);
    return 43;
}

int MavLinkSetActuatorControlTarget::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float_array(8, buffer, reinterpret_cast<float*>(&this->controls[0]), 8);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->group_mlx), 40);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 41);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 42);
    return 43;
}

int MavLinkActuatorControlTarget::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float_array(8, buffer, reinterpret_cast<const float*>(&this->controls[0]), 8);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->group_mlx), 40);
    return 41;
}

int MavLinkActuatorControlTarget::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float_array(8, buffer, reinterpret_cast<float*>(&this->controls[0]), 8);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->group_mlx), 40);
    return 41;
}

int MavLinkAltitude::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->altitude_monotonic), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->altitude_amsl), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->altitude_local), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->altitude_relative), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->altitude_terrain), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->bottom_clearance), 28);
    return 32;
}

int MavLinkAltitude::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->altitude_monotonic), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->altitude_amsl), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->altitude_local), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->altitude_relative), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->altitude_terrain), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->bottom_clearance), 28);
    return 32;
}

int MavLinkResourceRequest::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->request_id), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->uri_type), 1);
    pack_uint8_t_array(120, buffer, reinterpret_cast<const uint8_t*>(&this->uri[0]), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->transfer_type), 122);
    pack_uint8_t_array(120, buffer, reinterpret_cast<const uint8_t*>(&this->storage[0]), 123);
    return 243;
}

int MavLinkResourceRequest::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->request_id), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->uri_type), 1);
    unpack_uint8_t_array(120, buffer, reinterpret_cast<uint8_t*>(&this->uri[0]), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->transfer_type), 122);
    unpack_uint8_t_array(120, buffer, reinterpret_cast<uint8_t*>(&this->storage[0]), 123);
    return 243;
}

int MavLinkScaledPressure3::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->press_abs), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->press_diff), 8);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->temperature), 12);
    return 14;
}

int MavLinkScaledPressure3::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->press_abs), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->press_diff), 8);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->temperature), 12);
    return 14;
}

int MavLinkFollowTarget::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->timestamp), 0);
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->custom_state), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 16);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->alt), 24);
    pack_float_array(3, buffer, reinterpret_cast<const float*>(&this->vel[0]), 28);
    pack_float_array(3, buffer, reinterpret_cast<const float*>(&this->acc[0]), 40);
    pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->attitude_q[0]), 52);
    pack_float_array(3, buffer, reinterpret_cast<const float*>(&this->rates[0]), 68);
    pack_float_array(3, buffer, reinterpret_cast<const float*>(&this->position_cov[0]), 80);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->est_capabilities), 92);
    return 93;
}

int MavLinkFollowTarget::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->timestamp), 0);
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->custom_state), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 16);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->alt), 24);
    unpack_float_array(3, buffer, reinterpret_cast<float*>(&this->vel[0]), 28);
    unpack_float_array(3, buffer, reinterpret_cast<float*>(&this->acc[0]), 40);
    unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->attitude_q[0]), 52);
    unpack_float_array(3, buffer, reinterpret_cast<float*>(&this->rates[0]), 68);
    unpack_float_array(3, buffer, reinterpret_cast<float*>(&this->position_cov[0]), 80);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->est_capabilities), 92);
    return 93;
}

int MavLinkControlSystemState::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x_acc), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y_acc), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z_acc), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x_vel), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y_vel), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z_vel), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x_pos), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y_pos), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z_pos), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->airspeed), 44);
    pack_float_array(3, buffer, reinterpret_cast<const float*>(&this->vel_variance[0]), 48);
    pack_float_array(3, buffer, reinterpret_cast<const float*>(&this->pos_variance[0]), 60);
    pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->q[0]), 72);
    pack_float(buffer, reinterpret_cast<const float*>(&this->roll_rate), 88);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pitch_rate), 92);
    pack_float(buffer, reinterpret_cast<const float*>(&this->yaw_rate), 96);
    return 100;
}

int MavLinkControlSystemState::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x_acc), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y_acc), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z_acc), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x_vel), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y_vel), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z_vel), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x_pos), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y_pos), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z_pos), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->airspeed), 44);
    unpack_float_array(3, buffer, reinterpret_cast<float*>(&this->vel_variance[0]), 48);
    unpack_float_array(3, buffer, reinterpret_cast<float*>(&this->pos_variance[0]), 60);
    unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->q[0]), 72);
    unpack_float(buffer, reinterpret_cast<float*>(&this->roll_rate), 88);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pitch_rate), 92);
    unpack_float(buffer, reinterpret_cast<float*>(&this->yaw_rate), 96);
    return 100;
}

int MavLinkBatteryStatus::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->current_consumed), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->energy_consumed), 4);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->temperature), 8);
    pack_uint16_t_array(10, buffer, reinterpret_cast<const uint16_t*>(&this->voltages[0]), 10);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->current_battery), 30);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->id), 32);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->battery_function), 33);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->type), 34);
    pack_int8_t(buffer, reinterpret_cast<const int8_t*>(&this->battery_remaining), 35);
    return 36;
}

int MavLinkBatteryStatus::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->current_consumed), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->energy_consumed), 4);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->temperature), 8);
    unpack_uint16_t_array(10, buffer, reinterpret_cast<uint16_t*>(&this->voltages[0]), 10);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->current_battery), 30);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->id), 32);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->battery_function), 33);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->type), 34);
    unpack_int8_t(buffer, reinterpret_cast<int8_t*>(&this->battery_remaining), 35);
    return 36;
}

int MavLinkAutopilotVersion::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->capabilities), 0);
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->uid), 8);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->flight_sw_version), 16);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->middleware_sw_version), 20);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->os_sw_version), 24);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->board_version), 28);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->vendor_id), 32);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->product_id), 34);
    pack_uint8_t_array(8, buffer, reinterpret_cast<const uint8_t*>(&this->flight_custom_version[0]), 36);
    pack_uint8_t_array(8, buffer, reinterpret_cast<const uint8_t*>(&this->middleware_custom_version[0]), 44);
    pack_uint8_t_array(8, buffer, reinterpret_cast<const uint8_t*>(&this->os_custom_version[0]), 52);
    return 60;
}

int MavLinkAutopilotVersion::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->capabilities), 0);
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->uid), 8);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->flight_sw_version), 16);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->middleware_sw_version), 20);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->os_sw_version), 24);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->board_version), 28);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->vendor_id), 32);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->product_id), 34);
    unpack_uint8_t_array(8, buffer, reinterpret_cast<uint8_t*>(&this->flight_custom_version[0]), 36);
    unpack_uint8_t_array(8, buffer, reinterpret_cast<uint8_t*>(&this->middleware_custom_version[0]), 44);
    unpack_uint8_t_array(8, buffer, reinterpret_cast<uint8_t*>(&this->os_custom_version[0]), 52);
    return 60;
}

int MavLinkLandingTarget::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->angle_x), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->angle_y), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->distance), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->size_x), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->size_y), 24);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_num), 28);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->frame), 29);
    return 30;
}

int MavLinkLandingTarget::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->angle_x), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->angle_y), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->distance), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->size_x), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->size_y), 24);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_num), 28);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->frame), 29);
    return 30;
}

int MavLinkEstimatorStatus::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vel_ratio), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pos_horiz_ratio), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pos_vert_ratio), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->mag_ratio), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->hagl_ratio), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->tas_ratio), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pos_horiz_accuracy), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->pos_vert_accuracy), 36);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->flags), 40);
    return 42;
}

int MavLinkEstimatorStatus::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vel_ratio), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pos_horiz_ratio), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pos_vert_ratio), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->mag_ratio), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->hagl_ratio), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->tas_ratio), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pos_horiz_accuracy), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->pos_vert_accuracy), 36);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->flags), 40);
    return 42;
}

int MavLinkWindCov::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->wind_x), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->wind_y), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->wind_z), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->var_horiz), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->var_vert), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->wind_alt), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->horiz_accuracy), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vert_accuracy), 36);
    return 40;
}

int MavLinkWindCov::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->wind_x), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->wind_y), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->wind_z), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->var_horiz), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->var_vert), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->wind_alt), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->horiz_accuracy), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vert_accuracy), 36);
    return 40;
}

int MavLinkGpsInput::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_week_ms), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->alt), 20);
    pack_float(buffer, reinterpret_cast<const float*>(&this->hdop), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vdop), 28);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vn), 32);
    pack_float(buffer, reinterpret_cast<const float*>(&this->ve), 36);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vd), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->speed_accuracy), 44);
    pack_float(buffer, reinterpret_cast<const float*>(&this->horiz_accuracy), 48);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vert_accuracy), 52);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->ignore_flags), 56);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->time_week), 58);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->gps_id), 60);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->fix_type), 61);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->satellites_visible), 62);
    return 63;
}

int MavLinkGpsInput::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_week_ms), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->alt), 20);
    unpack_float(buffer, reinterpret_cast<float*>(&this->hdop), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vdop), 28);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vn), 32);
    unpack_float(buffer, reinterpret_cast<float*>(&this->ve), 36);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vd), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->speed_accuracy), 44);
    unpack_float(buffer, reinterpret_cast<float*>(&this->horiz_accuracy), 48);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vert_accuracy), 52);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->ignore_flags), 56);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->time_week), 58);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->gps_id), 60);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->fix_type), 61);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->satellites_visible), 62);
    return 63;
}

int MavLinkGpsRtcmData::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->flags), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->len), 1);
    pack_uint8_t_array(180, buffer, reinterpret_cast<const uint8_t*>(&this->data[0]), 2);
    return 182;
}

int MavLinkGpsRtcmData::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->flags), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->len), 1);
    unpack_uint8_t_array(180, buffer, reinterpret_cast<uint8_t*>(&this->data[0]), 2);
    return 182;
}

int MavLinkVibration::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vibration_x), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vibration_y), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->vibration_z), 16);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->clipping_0), 20);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->clipping_1), 24);
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->clipping_2), 28);
    return 32;
}

int MavLinkVibration::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vibration_x), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vibration_y), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->vibration_z), 16);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->clipping_0), 20);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->clipping_1), 24);
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->clipping_2), 28);
    return 32;
}

int MavLinkHomePosition::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->latitude), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->longitude), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->altitude), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 20);
    pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->q[0]), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->approach_x), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->approach_y), 44);
    pack_float(buffer, reinterpret_cast<const float*>(&this->approach_z), 48);
    return 52;
}

int MavLinkHomePosition::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->latitude), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->longitude), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->altitude), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 20);
    unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->q[0]), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->approach_x), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->approach_y), 44);
    unpack_float(buffer, reinterpret_cast<float*>(&this->approach_z), 48);
    return 52;
}

int MavLinkSetHomePosition::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->latitude), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->longitude), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->altitude), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 16);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 20);
    pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->q[0]), 24);
    pack_float(buffer, reinterpret_cast<const float*>(&this->approach_x), 40);
    pack_float(buffer, reinterpret_cast<const float*>(&this->approach_y), 44);
    pack_float(buffer, reinterpret_cast<const float*>(&this->approach_z), 48);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 52);
    return 53;
}

int MavLinkSetHomePosition::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->latitude), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->longitude), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->altitude), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 16);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 20);
    unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->q[0]), 24);
    unpack_float(buffer, reinterpret_cast<float*>(&this->approach_x), 40);
    unpack_float(buffer, reinterpret_cast<float*>(&this->approach_y), 44);
    unpack_float(buffer, reinterpret_cast<float*>(&this->approach_z), 48);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 52);
    return 53;
}

int MavLinkMessageInterval::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->interval_us), 0);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->message_id), 4);
    return 6;
}

int MavLinkMessageInterval::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->interval_us), 0);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->message_id), 4);
    return 6;
}

int MavLinkExtendedSysState::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->vtol_state), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->landed_state), 1);
    return 2;
}

int MavLinkExtendedSysState::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->vtol_state), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->landed_state), 1);
    return 2;
}

int MavLinkAdsbVehicle::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->ICAO_address), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->altitude), 12);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->heading), 16);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->hor_velocity), 18);
    pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->ver_velocity), 20);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->flags), 22);
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->squawk), 24);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->altitude_type), 26);
    pack_char_array(9, buffer, reinterpret_cast<const char*>(&this->callsign[0]), 27);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->emitter_type), 36);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->tslc), 37);
    return 38;
}

int MavLinkAdsbVehicle::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->ICAO_address), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->altitude), 12);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->heading), 16);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->hor_velocity), 18);
    unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->ver_velocity), 20);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->flags), 22);
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->squawk), 24);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->altitude_type), 26);
    unpack_char_array(9, buffer, reinterpret_cast<char*>(&this->callsign[0]), 27);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->emitter_type), 36);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->tslc), 37);
    return 38;
}

int MavLinkCollision::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->id), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->time_to_minimum_delta), 4);
    pack_float(buffer, reinterpret_cast<const float*>(&this->altitude_minimum_delta), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->horizontal_minimum_delta), 12);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->src), 16);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->action), 17);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->threat_level), 18);
    return 19;
}

int MavLinkCollision::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->id), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->time_to_minimum_delta), 4);
    unpack_float(buffer, reinterpret_cast<float*>(&this->altitude_minimum_delta), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->horizontal_minimum_delta), 12);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->src), 16);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->action), 17);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->threat_level), 18);
    return 19;
}

int MavLinkV2Extension::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->message_type), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_network), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_system), 3);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->target_component), 4);
    pack_uint8_t_array(249, buffer, reinterpret_cast<const uint8_t*>(&this->payload[0]), 5);
    return 254;
}

int MavLinkV2Extension::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->message_type), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_network), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_system), 3);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->target_component), 4);
    unpack_uint8_t_array(249, buffer, reinterpret_cast<uint8_t*>(&this->payload[0]), 5);
    return 254;
}

int MavLinkMemoryVect::pack(char* buffer) const {
    pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->address), 0);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->ver), 2);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->type), 3);
    pack_int8_t_array(32, buffer, reinterpret_cast<const int8_t*>(&this->value[0]), 4);
    return 36;
}

int MavLinkMemoryVect::unpack(const char* buffer) {
    unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->address), 0);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->ver), 2);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->type), 3);
    unpack_int8_t_array(32, buffer, reinterpret_cast<int8_t*>(&this->value[0]), 4);
    return 36;
}

int MavLinkDebugVect::pack(char* buffer) const {
    pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->x), 8);
    pack_float(buffer, reinterpret_cast<const float*>(&this->y), 12);
    pack_float(buffer, reinterpret_cast<const float*>(&this->z), 16);
    pack_char_array(10, buffer, reinterpret_cast<const char*>(&this->name[0]), 20);
    return 30;
}

int MavLinkDebugVect::unpack(const char* buffer) {
    unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->x), 8);
    unpack_float(buffer, reinterpret_cast<float*>(&this->y), 12);
    unpack_float(buffer, reinterpret_cast<float*>(&this->z), 16);
    unpack_char_array(10, buffer, reinterpret_cast<char*>(&this->name[0]), 20);
    return 30;
}

int MavLinkNamedValueFloat::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->value), 4);
    pack_char_array(10, buffer, reinterpret_cast<const char*>(&this->name[0]), 8);
    return 18;
}

int MavLinkNamedValueFloat::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->value), 4);
    unpack_char_array(10, buffer, reinterpret_cast<char*>(&this->name[0]), 8);
    return 18;
}

int MavLinkNamedValueInt::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->value), 4);
    pack_char_array(10, buffer, reinterpret_cast<const char*>(&this->name[0]), 8);
    return 18;
}

int MavLinkNamedValueInt::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->value), 4);
    unpack_char_array(10, buffer, reinterpret_cast<char*>(&this->name[0]), 8);
    return 18;
}

int MavLinkStatustext::pack(char* buffer) const {
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->severity), 0);
    pack_char_array(50, buffer, reinterpret_cast<const char*>(&this->text[0]), 1);
    return 51;
}

int MavLinkStatustext::unpack(const char* buffer) {
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->severity), 0);
    unpack_char_array(50, buffer, reinterpret_cast<char*>(&this->text[0]), 1);
    return 51;
}

int MavLinkDebug::pack(char* buffer) const {
    pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->time_boot_ms), 0);
    pack_float(buffer, reinterpret_cast<const float*>(&this->value), 4);
    pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->ind), 8);
    return 9;
}

int MavLinkDebug::unpack(const char* buffer) {
    unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->time_boot_ms), 0);
    unpack_float(buffer, reinterpret_cast<float*>(&this->value), 4);
    unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->ind), 8);
    return 9;
}

void MavCmdNavWaypoint::pack() {
    param1 = HoldTimeDecimal;
    param2 = AcceptanceRadiusMeters;
    param3 = p0ToPass;
    param4 = DesiredYawAngle;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavWaypoint::unpack() {
    HoldTimeDecimal = param1;
    AcceptanceRadiusMeters = param2;
    p0ToPass = param3;
    DesiredYawAngle = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdNavLoiterUnlim::pack() {
    param3 = RadiusAroundMission;
    param4 = DesiredYawAngle;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavLoiterUnlim::unpack() {
    RadiusAroundMission = param3;
    DesiredYawAngle = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdNavLoiterTurns::pack() {
    param1 = Turns;
    param3 = RadiusAroundMission;
    param4 = ExitXtrackLocation;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavLoiterTurns::unpack() {
    Turns = param1;
    RadiusAroundMission = param3;
    ExitXtrackLocation = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdNavLoiterTime::pack() {
    param1 = Seconds;
    param3 = RadiusAroundMission;
    param4 = ExitXtrackLocation;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavLoiterTime::unpack() {
    Seconds = param1;
    RadiusAroundMission = param3;
    ExitXtrackLocation = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdNavReturnToLaunch::pack() {
}
void MavCmdNavReturnToLaunch::unpack() {
}
void MavCmdNavLand::pack() {
    param1 = AbortAlt;
    param4 = DesiredYawAngle;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavLand::unpack() {
    AbortAlt = param1;
    DesiredYawAngle = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdNavTakeoff::pack() {
    param1 = MinimumPitch;
    param4 = YawAngle;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavTakeoff::unpack() {
    MinimumPitch = param1;
    YawAngle = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdNavLandLocal::pack() {
    param1 = LandingTargetNumber;
    param2 = MaximumAcceptedOffset;
    param3 = LandingDescendRate;
    param4 = DesiredYawAngle;
    param5 = Y;
    param6 = X;
    param7 = Z;
}
void MavCmdNavLandLocal::unpack() {
    LandingTargetNumber = param1;
    MaximumAcceptedOffset = param2;
    LandingDescendRate = param3;
    DesiredYawAngle = param4;
    Y = param5;
    X = param6;
    Z = param7;
}
void MavCmdNavTakeoffLocal::pack() {
    param1 = MinimumPitch;
    param3 = TakeoffAscendRate;
    param4 = YawAngle;
    param5 = Y;
    param6 = X;
    param7 = Z;
}
void MavCmdNavTakeoffLocal::unpack() {
    MinimumPitch = param1;
    TakeoffAscendRate = param3;
    YawAngle = param4;
    Y = param5;
    X = param6;
    Z = param7;
}
void MavCmdNavFollow::pack() {
    param1 = FollowingLogicTo;
    param2 = GroundSpeedOf;
    param3 = RadiusAroundMission;
    param4 = DesiredYawAngle;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavFollow::unpack() {
    FollowingLogicTo = param1;
    GroundSpeedOf = param2;
    RadiusAroundMission = param3;
    DesiredYawAngle = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdNavContinueAndChangeAlt::pack() {
    param1 = ClimbOrDescend;
    param7 = DesiredAltitudeMeters;
}
void MavCmdNavContinueAndChangeAlt::unpack() {
    ClimbOrDescend = param1;
    DesiredAltitudeMeters = param7;
}
void MavCmdNavLoiterToAlt::pack() {
    param1 = HeadingRequired;
    param2 = RadiusMeters;
    param4 = ExitXtrackLocation;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavLoiterToAlt::unpack() {
    HeadingRequired = param1;
    RadiusMeters = param2;
    ExitXtrackLocation = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdDoFollow::pack() {
    param1 = SystemId;
    param4 = AltitudeFlag;
    param5 = Altitude;
    param7 = TtlSecondsWhich;
}
void MavCmdDoFollow::unpack() {
    SystemId = param1;
    AltitudeFlag = param4;
    Altitude = param5;
    TtlSecondsWhich = param7;
}
void MavCmdDoFollowReposition::pack() {
    param1 = CameraQ1;
    param2 = CameraQ2;
    param3 = CameraQ3;
    param4 = CameraQ4;
    param5 = AltitudeOffsetTarget;
    param6 = XOffsetTarget;
    param7 = YOffsetTarget;
}
void MavCmdDoFollowReposition::unpack() {
    CameraQ1 = param1;
    CameraQ2 = param2;
    CameraQ3 = param3;
    CameraQ4 = param4;
    AltitudeOffsetTarget = param5;
    XOffsetTarget = param6;
    YOffsetTarget = param7;
}
void MavCmdNavRoi::pack() {
    param1 = RegionOfIntereset;
    param2 = MissionIndexTarget;
    param3 = RoiIndex;
    param5 = XLocationOf;
    param6 = Y;
    param7 = Z;
}
void MavCmdNavRoi::unpack() {
    RegionOfIntereset = param1;
    MissionIndexTarget = param2;
    RoiIndex = param3;
    XLocationOf = param5;
    Y = param6;
    Z = param7;
}
void MavCmdNavPathplanning::pack() {
    param1 = p0;
    param2 = p02;
    param4 = YawAngleAt;
    param5 = LatitudexOfGoal;
    param6 = LongitudeyOfGoal;
    param7 = AltitudezOfGoal;
}
void MavCmdNavPathplanning::unpack() {
    p0 = param1;
    p02 = param2;
    YawAngleAt = param4;
    LatitudexOfGoal = param5;
    LongitudeyOfGoal = param6;
    AltitudezOfGoal = param7;
}
void MavCmdNavSplineWaypoint::pack() {
    param1 = HoldTimeDecimal;
    param5 = LatitudexOfGoal;
    param6 = LongitudeyOfGoal;
    param7 = AltitudezOfGoal;
}
void MavCmdNavSplineWaypoint::unpack() {
    HoldTimeDecimal = param1;
    LatitudexOfGoal = param5;
    LongitudeyOfGoal = param6;
    AltitudezOfGoal = param7;
}
void MavCmdNavVtolTakeoff::pack() {
    param4 = YawAngleDegrees;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavVtolTakeoff::unpack() {
    YawAngleDegrees = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdNavVtolLand::pack() {
    param4 = YawAngleDegrees;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdNavVtolLand::unpack() {
    YawAngleDegrees = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdNavGuidedEnable::pack() {
    param1 = OnOff;
}
void MavCmdNavGuidedEnable::unpack() {
    OnOff = param1;
}
void MavCmdNavDelay::pack() {
    param1 = DelaySeconds;
    param2 = Hour;
    param3 = Minute;
    param4 = Second;
}
void MavCmdNavDelay::unpack() {
    DelaySeconds = param1;
    Hour = param2;
    Minute = param3;
    Second = param4;
}
void MavCmdNavLast::pack() {
}
void MavCmdNavLast::unpack() {
}
void MavCmdConditionDelay::pack() {
    param1 = DelaySeconds;
}
void MavCmdConditionDelay::unpack() {
    DelaySeconds = param1;
}
void MavCmdConditionChangeAlt::pack() {
    param1 = DescentAscend;
    param7 = FinishAltitude;
}
void MavCmdConditionChangeAlt::unpack() {
    DescentAscend = param1;
    FinishAltitude = param7;
}
void MavCmdConditionDistance::pack() {
    param1 = Distance;
}
void MavCmdConditionDistance::unpack() {
    Distance = param1;
}
void MavCmdConditionYaw::pack() {
    param1 = TargetAngle;
    param2 = SpeedDuringYaw;
    param3 = Direction;
    param4 = RelativeOffsetOr;
}
void MavCmdConditionYaw::unpack() {
    TargetAngle = param1;
    SpeedDuringYaw = param2;
    Direction = param3;
    RelativeOffsetOr = param4;
}
void MavCmdConditionLast::pack() {
}
void MavCmdConditionLast::unpack() {
}
void MavCmdDoSetMode::pack() {
    param1 = Mode;
    param2 = CustomMode;
    param3 = CustomSubMode;
}
void MavCmdDoSetMode::unpack() {
    Mode = param1;
    CustomMode = param2;
    CustomSubMode = param3;
}
void MavCmdDoJump::pack() {
    param1 = SequenceNumber;
    param2 = RepeatCount;
}
void MavCmdDoJump::unpack() {
    SequenceNumber = param1;
    RepeatCount = param2;
}
void MavCmdDoChangeSpeed::pack() {
    param1 = SpeedType;
    param2 = Speed;
    param3 = Throttle;
    param4 = AbsoluteOrRelative;
}
void MavCmdDoChangeSpeed::unpack() {
    SpeedType = param1;
    Speed = param2;
    Throttle = param3;
    AbsoluteOrRelative = param4;
}
void MavCmdDoSetHome::pack() {
    param1 = UseCurrent;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdDoSetHome::unpack() {
    UseCurrent = param1;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdDoSetParameter::pack() {
    param1 = ParameterNumber;
    param2 = ParameterValue;
}
void MavCmdDoSetParameter::unpack() {
    ParameterNumber = param1;
    ParameterValue = param2;
}
void MavCmdDoSetRelay::pack() {
    param1 = RelayNumber;
    param2 = Setting;
}
void MavCmdDoSetRelay::unpack() {
    RelayNumber = param1;
    Setting = param2;
}
void MavCmdDoRepeatRelay::pack() {
    param1 = RelayNumber;
    param2 = CycleCount;
    param3 = CycleTime;
}
void MavCmdDoRepeatRelay::unpack() {
    RelayNumber = param1;
    CycleCount = param2;
    CycleTime = param3;
}
void MavCmdDoSetServo::pack() {
    param1 = ServoNumber;
    param2 = Pwm;
}
void MavCmdDoSetServo::unpack() {
    ServoNumber = param1;
    Pwm = param2;
}
void MavCmdDoRepeatServo::pack() {
    param1 = ServoNumber;
    param2 = Pwm;
    param3 = CycleCount;
    param4 = CycleTime;
}
void MavCmdDoRepeatServo::unpack() {
    ServoNumber = param1;
    Pwm = param2;
    CycleCount = param3;
    CycleTime = param4;
}
void MavCmdDoFlighttermination::pack() {
    param1 = FlightTerminationActivated;
}
void MavCmdDoFlighttermination::unpack() {
    FlightTerminationActivated = param1;
}
void MavCmdDoChangeAltitude::pack() {
    param1 = AltitudeMeters;
    param2 = MavFrameOf;
}
void MavCmdDoChangeAltitude::unpack() {
    AltitudeMeters = param1;
    MavFrameOf = param2;
}
void MavCmdDoLandStart::pack() {
    param5 = Latitude;
    param6 = Longitude;
}
void MavCmdDoLandStart::unpack() {
    Latitude = param5;
    Longitude = param6;
}
void MavCmdDoRallyLand::pack() {
    param1 = BreakAltitude;
    param2 = LandingSpeed;
}
void MavCmdDoRallyLand::unpack() {
    BreakAltitude = param1;
    LandingSpeed = param2;
}
void MavCmdDoGoAround::pack() {
    param1 = Altitude;
}
void MavCmdDoGoAround::unpack() {
    Altitude = param1;
}
void MavCmdDoReposition::pack() {
    param1 = GroundSpeed;
    param2 = BitmaskOfOption;
    param4 = YawHeading;
    param5 = Latitude;
    param6 = Longitude;
    param7 = Altitude;
}
void MavCmdDoReposition::unpack() {
    GroundSpeed = param1;
    BitmaskOfOption = param2;
    YawHeading = param4;
    Latitude = param5;
    Longitude = param6;
    Altitude = param7;
}
void MavCmdDoPauseContinue::pack() {
    param1 = p0;
}
void MavCmdDoPauseContinue::unpack() {
    p0 = param1;
}
void MavCmdDoSetReverse::pack() {
    param1 = Direction;
}
void MavCmdDoSetReverse::unpack() {
    Direction = param1;
}
void MavCmdDoControlVideo::pack() {
    param1 = CameraId;
    param2 = Transmission;
    param3 = TransmissionMode;
    param4 = Recording;
}
void MavCmdDoControlVideo::unpack() {
    CameraId = param1;
    Transmission = param2;
    TransmissionMode = param3;
    Recording = param4;
}
void MavCmdDoSetRoi::pack() {
    param1 = RegionOfIntereset;
    param2 = MissionIndexTarget;
    param3 = RoiIndex;
    param5 = XLocationOf;
    param6 = Y;
    param7 = Z;
}
void MavCmdDoSetRoi::unpack() {
    RegionOfIntereset = param1;
    MissionIndexTarget = param2;
    RoiIndex = param3;
    XLocationOf = param5;
    Y = param6;
    Z = param7;
}
void MavCmdDoDigicamConfigure::pack() {
    param1 = Modes;
    param2 = ShutterSpeed;
    param3 = Aperture;
    param4 = IsoNumberE;
    param5 = ExposureTypeEnumerator;
    param6 = CommandIdentity;
    param7 = MainEngineCut;
}
void MavCmdDoDigicamConfigure::unpack() {
    Modes = param1;
    ShutterSpeed = param2;
    Aperture = param3;
    IsoNumberE = param4;
    ExposureTypeEnumerator = param5;
    CommandIdentity = param6;
    MainEngineCut = param7;
}
void MavCmdDoDigicamControl::pack() {
    param1 = SessionControlE;
    param2 = ZoomsAbsolutePosition;
    param3 = ZoomingStepValue;
    param4 = FocusLocking;
    param5 = ShootingCommand;
    param6 = CommandIdentity;
}
void MavCmdDoDigicamControl::unpack() {
    SessionControlE = param1;
    ZoomsAbsolutePosition = param2;
    ZoomingStepValue = param3;
    FocusLocking = param4;
    ShootingCommand = param5;
    CommandIdentity = param6;
}
void MavCmdDoMountConfigure::pack() {
    param1 = MountOperationMode;
    param2 = StabilizeRoll;
    param3 = StabilizePitch;
    param4 = StabilizeYaw;
}
void MavCmdDoMountConfigure::unpack() {
    MountOperationMode = param1;
    StabilizeRoll = param2;
    StabilizePitch = param3;
    StabilizeYaw = param4;
}
void MavCmdDoMountControl::pack() {
    param1 = PitchOrLat;
    param2 = RollOrLon;
    param3 = YawOrAlt;
    param7 = MavMountModeEnumValue;
}
void MavCmdDoMountControl::unpack() {
    PitchOrLat = param1;
    RollOrLon = param2;
    YawOrAlt = param3;
    MavMountModeEnumValue = param7;
}
void MavCmdDoSetCamTriggDist::pack() {
    param1 = CameraTriggerDistance;
}
void MavCmdDoSetCamTriggDist::unpack() {
    CameraTriggerDistance = param1;
}
void MavCmdDoFenceEnable::pack() {
    param1 = Enable;
}
void MavCmdDoFenceEnable::unpack() {
    Enable = param1;
}
void MavCmdDoParachute::pack() {
    param1 = Action;
}
void MavCmdDoParachute::unpack() {
    Action = param1;
}
void MavCmdDoMotorTest::pack() {
    param1 = MotorSequenceNumber;
    param2 = ThrottleType;
    param3 = Throttle;
    param4 = Timeout;
}
void MavCmdDoMotorTest::unpack() {
    MotorSequenceNumber = param1;
    ThrottleType = param2;
    Throttle = param3;
    Timeout = param4;
}
void MavCmdDoInvertedFlight::pack() {
    param1 = Inverted;
}
void MavCmdDoInvertedFlight::unpack() {
    Inverted = param1;
}
void MavCmdDoMountControlQuat::pack() {
    param1 = Q1;
    param2 = Q2;
    param3 = Q3;
    param4 = Q4;
}
void MavCmdDoMountControlQuat::unpack() {
    Q1 = param1;
    Q2 = param2;
    Q3 = param3;
    Q4 = param4;
}
void MavCmdDoGuidedMaster::pack() {
    param1 = SystemId;
    param2 = ComponentId;
}
void MavCmdDoGuidedMaster::unpack() {
    SystemId = param1;
    ComponentId = param2;
}
void MavCmdDoGuidedLimits::pack() {
    param1 = Timeout;
    param2 = AbsoluteAltitudeMin;
    param3 = AbsoluteAltitudeMax;
    param4 = HorizontalMoveLimit;
}
void MavCmdDoGuidedLimits::unpack() {
    Timeout = param1;
    AbsoluteAltitudeMin = param2;
    AbsoluteAltitudeMax = param3;
    HorizontalMoveLimit = param4;
}
void MavCmdDoEngineControl::pack() {
    param1 = p0;
    param2 = p02;
    param3 = HeightDelay;
}
void MavCmdDoEngineControl::unpack() {
    p0 = param1;
    p02 = param2;
    HeightDelay = param3;
}
void MavCmdDoLast::pack() {
}
void MavCmdDoLast::unpack() {
}
void MavCmdPreflightCalibration::pack() {
    param1 = GyroCalibration;
    param2 = MagnetometerCalibration;
    param3 = GroundPressure;
    param4 = RadioCalibration;
    param5 = AccelerometerCalibration;
    param6 = CompassmotorInterferenceCalibration;
}
void MavCmdPreflightCalibration::unpack() {
    GyroCalibration = param1;
    MagnetometerCalibration = param2;
    GroundPressure = param3;
    RadioCalibration = param4;
    AccelerometerCalibration = param5;
    CompassmotorInterferenceCalibration = param6;
}
void MavCmdPreflightSetSensorOffsets::pack() {
    param1 = SensorToAdjust;
    param2 = XAxisOffset;
    param3 = YAxisOffset;
    param4 = ZAxisOffset;
    param5 = GenericDimension4;
    param6 = GenericDimension5;
    param7 = GenericDimension6;
}
void MavCmdPreflightSetSensorOffsets::unpack() {
    SensorToAdjust = param1;
    XAxisOffset = param2;
    YAxisOffset = param3;
    ZAxisOffset = param4;
    GenericDimension4 = param5;
    GenericDimension5 = param6;
    GenericDimension6 = param7;
}
void MavCmdPreflightUavcan::pack() {
    param1 = p1;
}
void MavCmdPreflightUavcan::unpack() {
    p1 = param1;
}
void MavCmdPreflightStorage::pack() {
    param1 = ParameterStorage;
    param2 = MissionStorage;
    param3 = OnboardLogging;
}
void MavCmdPreflightStorage::unpack() {
    ParameterStorage = param1;
    MissionStorage = param2;
    OnboardLogging = param3;
}
void MavCmdPreflightRebootShutdown::pack() {
    param1 = p0;
    param2 = p02;
}
void MavCmdPreflightRebootShutdown::unpack() {
    p0 = param1;
    p02 = param2;
}
void MavCmdOverrideGoto::pack() {
    param1 = MavGotoDoHold;
    param2 = MavGotoHoldAtCurrentPosition;
    param3 = MavFrameCoordinateFrame;
    param4 = DesiredYawAngle;
    param5 = LatitudeX;
    param6 = LongitudeY;
    param7 = AltitudeZ;
}
void MavCmdOverrideGoto::unpack() {
    MavGotoDoHold = param1;
    MavGotoHoldAtCurrentPosition = param2;
    MavFrameCoordinateFrame = param3;
    DesiredYawAngle = param4;
    LatitudeX = param5;
    LongitudeY = param6;
    AltitudeZ = param7;
}
void MavCmdMissionStart::pack() {
    param1 = FirstItem;
    param2 = LastItem;
}
void MavCmdMissionStart::unpack() {
    FirstItem = param1;
    LastItem = param2;
}
void MavCmdComponentArmDisarm::pack() {
    param1 = p1ToArm;
}
void MavCmdComponentArmDisarm::unpack() {
    p1ToArm = param1;
}
void MavCmdGetHomePosition::pack() {
}
void MavCmdGetHomePosition::unpack() {
}
void MavCmdStartRxPair::pack() {
    param1 = p0;
    param2 = p02;
}
void MavCmdStartRxPair::unpack() {
    p0 = param1;
    p02 = param2;
}
void MavCmdGetMessageInterval::pack() {
    param1 = TheMavlinkMessage;
}
void MavCmdGetMessageInterval::unpack() {
    TheMavlinkMessage = param1;
}
void MavCmdSetMessageInterval::pack() {
    param1 = TheMavlinkMessage;
    param2 = TheIntervalBetween;
}
void MavCmdSetMessageInterval::unpack() {
    TheMavlinkMessage = param1;
    TheIntervalBetween = param2;
}
void MavCmdRequestAutopilotCapabilities::pack() {
    param1 = p1;
}
void MavCmdRequestAutopilotCapabilities::unpack() {
    p1 = param1;
}
void MavCmdImageStartCapture::pack() {
    param1 = DurationBetweenTwo;
    param2 = NumberOfImages;
    param3 = ResolutionMegapixels;
}
void MavCmdImageStartCapture::unpack() {
    DurationBetweenTwo = param1;
    NumberOfImages = param2;
    ResolutionMegapixels = param3;
}
void MavCmdImageStopCapture::pack() {
}
void MavCmdImageStopCapture::unpack() {
}
void MavCmdDoTriggerControl::pack() {
    param1 = TriggerEnabledisable;
    param2 = ShutterIntegrationTime;
}
void MavCmdDoTriggerControl::unpack() {
    TriggerEnabledisable = param1;
    ShutterIntegrationTime = param2;
}
void MavCmdVideoStartCapture::pack() {
    param1 = CameraId;
    param2 = FramesPerSecond;
    param3 = ResolutionMegapixels;
}
void MavCmdVideoStartCapture::unpack() {
    CameraId = param1;
    FramesPerSecond = param2;
    ResolutionMegapixels = param3;
}
void MavCmdVideoStopCapture::pack() {
}
void MavCmdVideoStopCapture::unpack() {
}
void MavCmdPanoramaCreate::pack() {
    param1 = ViewingAngleHorizontal;
    param2 = ViewingAngleVertical;
    param3 = SpeedOfHorizontal;
    param4 = SpeedOfVertical;
}
void MavCmdPanoramaCreate::unpack() {
    ViewingAngleHorizontal = param1;
    ViewingAngleVertical = param2;
    SpeedOfHorizontal = param3;
    SpeedOfVertical = param4;
}
void MavCmdDoVtolTransition::pack() {
    param1 = TheTargetVtol;
}
void MavCmdDoVtolTransition::unpack() {
    TheTargetVtol = param1;
}
void MavCmdSetGuidedSubmodeStandard::pack() {
}
void MavCmdSetGuidedSubmodeStandard::unpack() {
}
void MavCmdSetGuidedSubmodeCircle::pack() {
    param1 = RadiusOfDesired;
    param2 = UserDefined;
    param3 = UserDefined2;
    param4 = UserDefined3;
    param5 = UnscaledTargetLatitude;
    param6 = UnscaledTargetLongitude;
}
void MavCmdSetGuidedSubmodeCircle::unpack() {
    RadiusOfDesired = param1;
    UserDefined = param2;
    UserDefined2 = param3;
    UserDefined3 = param4;
    UnscaledTargetLatitude = param5;
    UnscaledTargetLongitude = param6;
}
void MavCmdPayloadPrepareDeploy::pack() {
    param1 = OperationMode;
    param2 = DesiredApproachVector;
    param3 = DesiredGroundSpeed;
    param4 = MinimumAltitudeClearance;
    param5 = LatitudeUnscaledFor;
    param6 = LongitudeUnscaledFor;
    param7 = Altitude;
}
void MavCmdPayloadPrepareDeploy::unpack() {
    OperationMode = param1;
    DesiredApproachVector = param2;
    DesiredGroundSpeed = param3;
    MinimumAltitudeClearance = param4;
    LatitudeUnscaledFor = param5;
    LongitudeUnscaledFor = param6;
    Altitude = param7;
}
void MavCmdPayloadControlDeploy::pack() {
    param1 = OperationMode;
}
void MavCmdPayloadControlDeploy::unpack() {
    OperationMode = param1;
}
void MavCmdWaypointUser1::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdWaypointUser1::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdWaypointUser2::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdWaypointUser2::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdWaypointUser3::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdWaypointUser3::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdWaypointUser4::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdWaypointUser4::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdWaypointUser5::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdWaypointUser5::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdSpatialUser1::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdSpatialUser1::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdSpatialUser2::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdSpatialUser2::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdSpatialUser3::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdSpatialUser3::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdSpatialUser4::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdSpatialUser4::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdSpatialUser5::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = LatitudeUnscaled;
    param6 = LongitudeUnscaled;
    param7 = Altitude;
}
void MavCmdSpatialUser5::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    LatitudeUnscaled = param5;
    LongitudeUnscaled = param6;
    Altitude = param7;
}
void MavCmdUser1::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = UserDefined5;
    param6 = UserDefined6;
    param7 = UserDefined7;
}
void MavCmdUser1::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    UserDefined5 = param5;
    UserDefined6 = param6;
    UserDefined7 = param7;
}
void MavCmdUser2::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = UserDefined5;
    param6 = UserDefined6;
    param7 = UserDefined7;
}
void MavCmdUser2::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    UserDefined5 = param5;
    UserDefined6 = param6;
    UserDefined7 = param7;
}
void MavCmdUser3::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = UserDefined5;
    param6 = UserDefined6;
    param7 = UserDefined7;
}
void MavCmdUser3::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    UserDefined5 = param5;
    UserDefined6 = param6;
    UserDefined7 = param7;
}
void MavCmdUser4::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = UserDefined5;
    param6 = UserDefined6;
    param7 = UserDefined7;
}
void MavCmdUser4::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    UserDefined5 = param5;
    UserDefined6 = param6;
    UserDefined7 = param7;
}
void MavCmdUser5::pack() {
    param1 = UserDefined;
    param2 = UserDefined2;
    param3 = UserDefined3;
    param4 = UserDefined4;
    param5 = UserDefined5;
    param6 = UserDefined6;
    param7 = UserDefined7;
}
void MavCmdUser5::unpack() {
    UserDefined = param1;
    UserDefined2 = param2;
    UserDefined3 = param3;
    UserDefined4 = param4;
    UserDefined5 = param5;
    UserDefined6 = param6;
    UserDefined7 = param7;
}
