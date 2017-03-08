// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "MavLinkMessages.hpp"
#include <sstream>
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

std::string MavLinkHeartbeat::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HEARTBEAT\", \"id\": 0";
	result << ", \"custom_mode\":" << this->custom_mode;
	result << ", \"type\":" << static_cast<unsigned int>(this->type);
	result << ", \"autopilot\":" << static_cast<unsigned int>(this->autopilot);
	result << ", \"base_mode\":" << static_cast<unsigned int>(this->base_mode);
	result << ", \"system_status\":" << static_cast<unsigned int>(this->system_status);
	result << ", \"mavlink_version\":" << static_cast<unsigned int>(this->mavlink_version);
	result << "},";
	return result.str();
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

std::string MavLinkSysStatus::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SYS_STATUS\", \"id\": 1";
	result << ", \"onboard_control_sensors_present\":" << this->onboard_control_sensors_present;
	result << ", \"onboard_control_sensors_enabled\":" << this->onboard_control_sensors_enabled;
	result << ", \"onboard_control_sensors_health\":" << this->onboard_control_sensors_health;
	result << ", \"load\":" << this->load;
	result << ", \"voltage_battery\":" << this->voltage_battery;
	result << ", \"current_battery\":" << this->current_battery;
	result << ", \"drop_rate_comm\":" << this->drop_rate_comm;
	result << ", \"errors_comm\":" << this->errors_comm;
	result << ", \"errors_count1\":" << this->errors_count1;
	result << ", \"errors_count2\":" << this->errors_count2;
	result << ", \"errors_count3\":" << this->errors_count3;
	result << ", \"errors_count4\":" << this->errors_count4;
	result << ", \"battery_remaining\":" << static_cast<int>(this->battery_remaining);
	result << "},";
	return result.str();
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

std::string MavLinkSystemTime::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SYSTEM_TIME\", \"id\": 2";
	result << ", \"time_unix_usec\":" << this->time_unix_usec;
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << "},";
	return result.str();
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

std::string MavLinkPing::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"PING\", \"id\": 4";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"seq\":" << this->seq;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkChangeOperatorControl::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"CHANGE_OPERATOR_CONTROL\", \"id\": 5";
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"control_request\":" << static_cast<unsigned int>(this->control_request);
	result << ", \"version\":" << static_cast<unsigned int>(this->version);
	result << ", \"passkey\":" << "\"" << char_array_tostring(25, reinterpret_cast<char*>(&this->passkey[0])) << "\"";
	result << "},";
	return result.str();
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

std::string MavLinkChangeOperatorControlAck::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"CHANGE_OPERATOR_CONTROL_ACK\", \"id\": 6";
	result << ", \"gcs_system_id\":" << static_cast<unsigned int>(this->gcs_system_id);
	result << ", \"control_request\":" << static_cast<unsigned int>(this->control_request);
	result << ", \"ack\":" << static_cast<unsigned int>(this->ack);
	result << "},";
	return result.str();
}

int MavLinkAuthKey::pack(char* buffer) const {
	pack_char_array(32, buffer, reinterpret_cast<const char*>(&this->key[0]), 0);
	return 32;
}

int MavLinkAuthKey::unpack(const char* buffer) {
	unpack_char_array(32, buffer, reinterpret_cast<char*>(&this->key[0]), 0);
	return 32;
}

std::string MavLinkAuthKey::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"AUTH_KEY\", \"id\": 7";
	result << ", \"key\":" << "\"" << char_array_tostring(32, reinterpret_cast<char*>(&this->key[0])) << "\"";
	result << "},";
	return result.str();
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

std::string MavLinkSetMode::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SET_MODE\", \"id\": 11";
	result << ", \"custom_mode\":" << this->custom_mode;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"base_mode\":" << static_cast<unsigned int>(this->base_mode);
	result << "},";
	return result.str();
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

std::string MavLinkParamRequestRead::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"PARAM_REQUEST_READ\", \"id\": 20";
	result << ", \"param_index\":" << this->param_index;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"param_id\":" << "\"" << char_array_tostring(16, reinterpret_cast<char*>(&this->param_id[0])) << "\"";
	result << "},";
	return result.str();
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

std::string MavLinkParamRequestList::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"PARAM_REQUEST_LIST\", \"id\": 21";
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkParamValue::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"PARAM_VALUE\", \"id\": 22";
	result << ", \"param_value\":" << float_tostring(this->param_value);
	result << ", \"param_count\":" << this->param_count;
	result << ", \"param_index\":" << this->param_index;
	result << ", \"param_id\":" << "\"" << char_array_tostring(16, reinterpret_cast<char*>(&this->param_id[0])) << "\"";
	result << ", \"param_type\":" << static_cast<unsigned int>(this->param_type);
	result << "},";
	return result.str();
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

std::string MavLinkParamSet::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"PARAM_SET\", \"id\": 23";
	result << ", \"param_value\":" << float_tostring(this->param_value);
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"param_id\":" << "\"" << char_array_tostring(16, reinterpret_cast<char*>(&this->param_id[0])) << "\"";
	result << ", \"param_type\":" << static_cast<unsigned int>(this->param_type);
	result << "},";
	return result.str();
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

std::string MavLinkGpsRawInt::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GPS_RAW_INT\", \"id\": 24";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"alt\":" << this->alt;
	result << ", \"eph\":" << this->eph;
	result << ", \"epv\":" << this->epv;
	result << ", \"vel\":" << this->vel;
	result << ", \"cog\":" << this->cog;
	result << ", \"fix_type\":" << static_cast<unsigned int>(this->fix_type);
	result << ", \"satellites_visible\":" << static_cast<unsigned int>(this->satellites_visible);
	result << "},";
	return result.str();
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

std::string MavLinkGpsStatus::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GPS_STATUS\", \"id\": 25";
	result << ", \"satellites_visible\":" << static_cast<unsigned int>(this->satellites_visible);
	result << ", \"satellite_prn\":" << "[" << uint8_t_array_tostring(20, reinterpret_cast<uint8_t*>(&this->satellite_prn[0])) << "]";
	result << ", \"satellite_used\":" << "[" << uint8_t_array_tostring(20, reinterpret_cast<uint8_t*>(&this->satellite_used[0])) << "]";
	result << ", \"satellite_elevation\":" << "[" << uint8_t_array_tostring(20, reinterpret_cast<uint8_t*>(&this->satellite_elevation[0])) << "]";
	result << ", \"satellite_azimuth\":" << "[" << uint8_t_array_tostring(20, reinterpret_cast<uint8_t*>(&this->satellite_azimuth[0])) << "]";
	result << ", \"satellite_snr\":" << "[" << uint8_t_array_tostring(20, reinterpret_cast<uint8_t*>(&this->satellite_snr[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkScaledImu::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SCALED_IMU\", \"id\": 26";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"xacc\":" << this->xacc;
	result << ", \"yacc\":" << this->yacc;
	result << ", \"zacc\":" << this->zacc;
	result << ", \"xgyro\":" << this->xgyro;
	result << ", \"ygyro\":" << this->ygyro;
	result << ", \"zgyro\":" << this->zgyro;
	result << ", \"xmag\":" << this->xmag;
	result << ", \"ymag\":" << this->ymag;
	result << ", \"zmag\":" << this->zmag;
	result << "},";
	return result.str();
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

std::string MavLinkRawImu::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"RAW_IMU\", \"id\": 27";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"xacc\":" << this->xacc;
	result << ", \"yacc\":" << this->yacc;
	result << ", \"zacc\":" << this->zacc;
	result << ", \"xgyro\":" << this->xgyro;
	result << ", \"ygyro\":" << this->ygyro;
	result << ", \"zgyro\":" << this->zgyro;
	result << ", \"xmag\":" << this->xmag;
	result << ", \"ymag\":" << this->ymag;
	result << ", \"zmag\":" << this->zmag;
	result << "},";
	return result.str();
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

std::string MavLinkRawPressure::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"RAW_PRESSURE\", \"id\": 28";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"press_abs\":" << this->press_abs;
	result << ", \"press_diff1\":" << this->press_diff1;
	result << ", \"press_diff2\":" << this->press_diff2;
	result << ", \"temperature\":" << this->temperature;
	result << "},";
	return result.str();
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

std::string MavLinkScaledPressure::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SCALED_PRESSURE\", \"id\": 29";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"press_abs\":" << float_tostring(this->press_abs);
	result << ", \"press_diff\":" << float_tostring(this->press_diff);
	result << ", \"temperature\":" << this->temperature;
	result << "},";
	return result.str();
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

std::string MavLinkAttitude::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ATTITUDE\", \"id\": 30";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"roll\":" << float_tostring(this->roll);
	result << ", \"pitch\":" << float_tostring(this->pitch);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << ", \"rollspeed\":" << float_tostring(this->rollspeed);
	result << ", \"pitchspeed\":" << float_tostring(this->pitchspeed);
	result << ", \"yawspeed\":" << float_tostring(this->yawspeed);
	result << "},";
	return result.str();
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

std::string MavLinkAttitudeQuaternion::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ATTITUDE_QUATERNION\", \"id\": 31";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"q1\":" << float_tostring(this->q1);
	result << ", \"q2\":" << float_tostring(this->q2);
	result << ", \"q3\":" << float_tostring(this->q3);
	result << ", \"q4\":" << float_tostring(this->q4);
	result << ", \"rollspeed\":" << float_tostring(this->rollspeed);
	result << ", \"pitchspeed\":" << float_tostring(this->pitchspeed);
	result << ", \"yawspeed\":" << float_tostring(this->yawspeed);
	result << "},";
	return result.str();
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

std::string MavLinkLocalPositionNed::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LOCAL_POSITION_NED\", \"id\": 32";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"vx\":" << float_tostring(this->vx);
	result << ", \"vy\":" << float_tostring(this->vy);
	result << ", \"vz\":" << float_tostring(this->vz);
	result << "},";
	return result.str();
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

std::string MavLinkGlobalPositionInt::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GLOBAL_POSITION_INT\", \"id\": 33";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"alt\":" << this->alt;
	result << ", \"relative_alt\":" << this->relative_alt;
	result << ", \"vx\":" << this->vx;
	result << ", \"vy\":" << this->vy;
	result << ", \"vz\":" << this->vz;
	result << ", \"hdg\":" << this->hdg;
	result << "},";
	return result.str();
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

std::string MavLinkRcChannelsScaled::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"RC_CHANNELS_SCALED\", \"id\": 34";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"chan1_scaled\":" << this->chan1_scaled;
	result << ", \"chan2_scaled\":" << this->chan2_scaled;
	result << ", \"chan3_scaled\":" << this->chan3_scaled;
	result << ", \"chan4_scaled\":" << this->chan4_scaled;
	result << ", \"chan5_scaled\":" << this->chan5_scaled;
	result << ", \"chan6_scaled\":" << this->chan6_scaled;
	result << ", \"chan7_scaled\":" << this->chan7_scaled;
	result << ", \"chan8_scaled\":" << this->chan8_scaled;
	result << ", \"port\":" << static_cast<unsigned int>(this->port);
	result << ", \"rssi\":" << static_cast<unsigned int>(this->rssi);
	result << "},";
	return result.str();
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

std::string MavLinkRcChannelsRaw::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"RC_CHANNELS_RAW\", \"id\": 35";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"chan1_raw\":" << this->chan1_raw;
	result << ", \"chan2_raw\":" << this->chan2_raw;
	result << ", \"chan3_raw\":" << this->chan3_raw;
	result << ", \"chan4_raw\":" << this->chan4_raw;
	result << ", \"chan5_raw\":" << this->chan5_raw;
	result << ", \"chan6_raw\":" << this->chan6_raw;
	result << ", \"chan7_raw\":" << this->chan7_raw;
	result << ", \"chan8_raw\":" << this->chan8_raw;
	result << ", \"port\":" << static_cast<unsigned int>(this->port);
	result << ", \"rssi\":" << static_cast<unsigned int>(this->rssi);
	result << "},";
	return result.str();
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

std::string MavLinkServoOutputRaw::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SERVO_OUTPUT_RAW\", \"id\": 36";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"servo1_raw\":" << this->servo1_raw;
	result << ", \"servo2_raw\":" << this->servo2_raw;
	result << ", \"servo3_raw\":" << this->servo3_raw;
	result << ", \"servo4_raw\":" << this->servo4_raw;
	result << ", \"servo5_raw\":" << this->servo5_raw;
	result << ", \"servo6_raw\":" << this->servo6_raw;
	result << ", \"servo7_raw\":" << this->servo7_raw;
	result << ", \"servo8_raw\":" << this->servo8_raw;
	result << ", \"servo9_raw\":" << this->servo9_raw;
	result << ", \"servo10_raw\":" << this->servo10_raw;
	result << ", \"servo11_raw\":" << this->servo11_raw;
	result << ", \"servo12_raw\":" << this->servo12_raw;
	result << ", \"servo13_raw\":" << this->servo13_raw;
	result << ", \"servo14_raw\":" << this->servo14_raw;
	result << ", \"servo15_raw\":" << this->servo15_raw;
	result << ", \"servo16_raw\":" << this->servo16_raw;
	result << ", \"port\":" << static_cast<unsigned int>(this->port);
	result << "},";
	return result.str();
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

std::string MavLinkMissionRequestPartialList::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_REQUEST_PARTIAL_LIST\", \"id\": 37";
	result << ", \"start_index\":" << this->start_index;
	result << ", \"end_index\":" << this->end_index;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkMissionWritePartialList::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_WRITE_PARTIAL_LIST\", \"id\": 38";
	result << ", \"start_index\":" << this->start_index;
	result << ", \"end_index\":" << this->end_index;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkMissionItem::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_ITEM\", \"id\": 39";
	result << ", \"param1\":" << float_tostring(this->param1);
	result << ", \"param2\":" << float_tostring(this->param2);
	result << ", \"param3\":" << float_tostring(this->param3);
	result << ", \"param4\":" << float_tostring(this->param4);
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"seq\":" << this->seq;
	result << ", \"command\":" << this->command;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"frame\":" << static_cast<unsigned int>(this->frame);
	result << ", \"current\":" << static_cast<unsigned int>(this->current);
	result << ", \"autocontinue\":" << static_cast<unsigned int>(this->autocontinue);
	result << "},";
	return result.str();
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

std::string MavLinkMissionRequest::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_REQUEST\", \"id\": 40";
	result << ", \"seq\":" << this->seq;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkMissionSetCurrent::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_SET_CURRENT\", \"id\": 41";
	result << ", \"seq\":" << this->seq;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
}

int MavLinkMissionCurrent::pack(char* buffer) const {
	pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seq), 0);
	return 2;
}

int MavLinkMissionCurrent::unpack(const char* buffer) {
	unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seq), 0);
	return 2;
}

std::string MavLinkMissionCurrent::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_CURRENT\", \"id\": 42";
	result << ", \"seq\":" << this->seq;
	result << "},";
	return result.str();
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

std::string MavLinkMissionRequestList::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_REQUEST_LIST\", \"id\": 43";
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkMissionCount::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_COUNT\", \"id\": 44";
	result << ", \"count\":" << this->count;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkMissionClearAll::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_CLEAR_ALL\", \"id\": 45";
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
}

int MavLinkMissionItemReached::pack(char* buffer) const {
	pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->seq), 0);
	return 2;
}

int MavLinkMissionItemReached::unpack(const char* buffer) {
	unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->seq), 0);
	return 2;
}

std::string MavLinkMissionItemReached::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_ITEM_REACHED\", \"id\": 46";
	result << ", \"seq\":" << this->seq;
	result << "},";
	return result.str();
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

std::string MavLinkMissionAck::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_ACK\", \"id\": 47";
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"type\":" << static_cast<unsigned int>(this->type);
	result << "},";
	return result.str();
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

std::string MavLinkSetGpsGlobalOrigin::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SET_GPS_GLOBAL_ORIGIN\", \"id\": 48";
	result << ", \"latitude\":" << this->latitude;
	result << ", \"longitude\":" << this->longitude;
	result << ", \"altitude\":" << this->altitude;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << "},";
	return result.str();
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

std::string MavLinkGpsGlobalOrigin::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GPS_GLOBAL_ORIGIN\", \"id\": 49";
	result << ", \"latitude\":" << this->latitude;
	result << ", \"longitude\":" << this->longitude;
	result << ", \"altitude\":" << this->altitude;
	result << "},";
	return result.str();
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

std::string MavLinkParamMapRc::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"PARAM_MAP_RC\", \"id\": 50";
	result << ", \"param_value0\":" << float_tostring(this->param_value0);
	result << ", \"scale\":" << float_tostring(this->scale);
	result << ", \"param_value_min\":" << float_tostring(this->param_value_min);
	result << ", \"param_value_max\":" << float_tostring(this->param_value_max);
	result << ", \"param_index\":" << this->param_index;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"param_id\":" << "\"" << char_array_tostring(16, reinterpret_cast<char*>(&this->param_id[0])) << "\"";
	result << ", \"parameter_rc_channel_index\":" << static_cast<unsigned int>(this->parameter_rc_channel_index);
	result << "},";
	return result.str();
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

std::string MavLinkMissionRequestInt::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_REQUEST_INT\", \"id\": 51";
	result << ", \"seq\":" << this->seq;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkSafetySetAllowedArea::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SAFETY_SET_ALLOWED_AREA\", \"id\": 54";
	result << ", \"p1x\":" << float_tostring(this->p1x);
	result << ", \"p1y\":" << float_tostring(this->p1y);
	result << ", \"p1z\":" << float_tostring(this->p1z);
	result << ", \"p2x\":" << float_tostring(this->p2x);
	result << ", \"p2y\":" << float_tostring(this->p2y);
	result << ", \"p2z\":" << float_tostring(this->p2z);
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"frame\":" << static_cast<unsigned int>(this->frame);
	result << "},";
	return result.str();
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

std::string MavLinkSafetyAllowedArea::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SAFETY_ALLOWED_AREA\", \"id\": 55";
	result << ", \"p1x\":" << float_tostring(this->p1x);
	result << ", \"p1y\":" << float_tostring(this->p1y);
	result << ", \"p1z\":" << float_tostring(this->p1z);
	result << ", \"p2x\":" << float_tostring(this->p2x);
	result << ", \"p2y\":" << float_tostring(this->p2y);
	result << ", \"p2z\":" << float_tostring(this->p2z);
	result << ", \"frame\":" << static_cast<unsigned int>(this->frame);
	result << "},";
	return result.str();
}

int MavLinkAttitudeQuaternionCov::pack(char* buffer) const {
	pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
	pack_float_array(4, buffer, reinterpret_cast<const float*>(&this->q[0]), 8);
	pack_float(buffer, reinterpret_cast<const float*>(&this->rollspeed), 24);
	pack_float(buffer, reinterpret_cast<const float*>(&this->pitchspeed), 28);
	pack_float(buffer, reinterpret_cast<const float*>(&this->yawspeed), 32);
	pack_float_array(9, buffer, reinterpret_cast<const float*>(&this->covariance[0]), 36);
	return 72;
}

int MavLinkAttitudeQuaternionCov::unpack(const char* buffer) {
	unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
	unpack_float_array(4, buffer, reinterpret_cast<float*>(&this->q[0]), 8);
	unpack_float(buffer, reinterpret_cast<float*>(&this->rollspeed), 24);
	unpack_float(buffer, reinterpret_cast<float*>(&this->pitchspeed), 28);
	unpack_float(buffer, reinterpret_cast<float*>(&this->yawspeed), 32);
	unpack_float_array(9, buffer, reinterpret_cast<float*>(&this->covariance[0]), 36);
	return 72;
}

std::string MavLinkAttitudeQuaternionCov::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ATTITUDE_QUATERNION_COV\", \"id\": 61";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"q\":" << "[" << float_array_tostring(4, reinterpret_cast<float*>(&this->q[0])) << "]";
	result << ", \"rollspeed\":" << float_tostring(this->rollspeed);
	result << ", \"pitchspeed\":" << float_tostring(this->pitchspeed);
	result << ", \"yawspeed\":" << float_tostring(this->yawspeed);
	result << ", \"covariance\":" << "[" << float_array_tostring(9, reinterpret_cast<float*>(&this->covariance[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkNavControllerOutput::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"NAV_CONTROLLER_OUTPUT\", \"id\": 62";
	result << ", \"nav_roll\":" << float_tostring(this->nav_roll);
	result << ", \"nav_pitch\":" << float_tostring(this->nav_pitch);
	result << ", \"alt_error\":" << float_tostring(this->alt_error);
	result << ", \"aspd_error\":" << float_tostring(this->aspd_error);
	result << ", \"xtrack_error\":" << float_tostring(this->xtrack_error);
	result << ", \"nav_bearing\":" << this->nav_bearing;
	result << ", \"target_bearing\":" << this->target_bearing;
	result << ", \"wp_dist\":" << this->wp_dist;
	result << "},";
	return result.str();
}

int MavLinkGlobalPositionIntCov::pack(char* buffer) const {
	pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
	pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lat), 8);
	pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->lon), 12);
	pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->alt), 16);
	pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->relative_alt), 20);
	pack_float(buffer, reinterpret_cast<const float*>(&this->vx), 24);
	pack_float(buffer, reinterpret_cast<const float*>(&this->vy), 28);
	pack_float(buffer, reinterpret_cast<const float*>(&this->vz), 32);
	pack_float_array(36, buffer, reinterpret_cast<const float*>(&this->covariance[0]), 36);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->estimator_type), 180);
	return 181;
}

int MavLinkGlobalPositionIntCov::unpack(const char* buffer) {
	unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
	unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lat), 8);
	unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->lon), 12);
	unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->alt), 16);
	unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->relative_alt), 20);
	unpack_float(buffer, reinterpret_cast<float*>(&this->vx), 24);
	unpack_float(buffer, reinterpret_cast<float*>(&this->vy), 28);
	unpack_float(buffer, reinterpret_cast<float*>(&this->vz), 32);
	unpack_float_array(36, buffer, reinterpret_cast<float*>(&this->covariance[0]), 36);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->estimator_type), 180);
	return 181;
}

std::string MavLinkGlobalPositionIntCov::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GLOBAL_POSITION_INT_COV\", \"id\": 63";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"alt\":" << this->alt;
	result << ", \"relative_alt\":" << this->relative_alt;
	result << ", \"vx\":" << float_tostring(this->vx);
	result << ", \"vy\":" << float_tostring(this->vy);
	result << ", \"vz\":" << float_tostring(this->vz);
	result << ", \"covariance\":" << "[" << float_array_tostring(36, reinterpret_cast<float*>(&this->covariance[0])) << "]";
	result << ", \"estimator_type\":" << static_cast<unsigned int>(this->estimator_type);
	result << "},";
	return result.str();
}

int MavLinkLocalPositionNedCov::pack(char* buffer) const {
	pack_uint64_t(buffer, reinterpret_cast<const uint64_t*>(&this->time_usec), 0);
	pack_float(buffer, reinterpret_cast<const float*>(&this->x), 8);
	pack_float(buffer, reinterpret_cast<const float*>(&this->y), 12);
	pack_float(buffer, reinterpret_cast<const float*>(&this->z), 16);
	pack_float(buffer, reinterpret_cast<const float*>(&this->vx), 20);
	pack_float(buffer, reinterpret_cast<const float*>(&this->vy), 24);
	pack_float(buffer, reinterpret_cast<const float*>(&this->vz), 28);
	pack_float(buffer, reinterpret_cast<const float*>(&this->ax), 32);
	pack_float(buffer, reinterpret_cast<const float*>(&this->ay), 36);
	pack_float(buffer, reinterpret_cast<const float*>(&this->az), 40);
	pack_float_array(45, buffer, reinterpret_cast<const float*>(&this->covariance[0]), 44);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->estimator_type), 224);
	return 225;
}

int MavLinkLocalPositionNedCov::unpack(const char* buffer) {
	unpack_uint64_t(buffer, reinterpret_cast<uint64_t*>(&this->time_usec), 0);
	unpack_float(buffer, reinterpret_cast<float*>(&this->x), 8);
	unpack_float(buffer, reinterpret_cast<float*>(&this->y), 12);
	unpack_float(buffer, reinterpret_cast<float*>(&this->z), 16);
	unpack_float(buffer, reinterpret_cast<float*>(&this->vx), 20);
	unpack_float(buffer, reinterpret_cast<float*>(&this->vy), 24);
	unpack_float(buffer, reinterpret_cast<float*>(&this->vz), 28);
	unpack_float(buffer, reinterpret_cast<float*>(&this->ax), 32);
	unpack_float(buffer, reinterpret_cast<float*>(&this->ay), 36);
	unpack_float(buffer, reinterpret_cast<float*>(&this->az), 40);
	unpack_float_array(45, buffer, reinterpret_cast<float*>(&this->covariance[0]), 44);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->estimator_type), 224);
	return 225;
}

std::string MavLinkLocalPositionNedCov::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LOCAL_POSITION_NED_COV\", \"id\": 64";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"vx\":" << float_tostring(this->vx);
	result << ", \"vy\":" << float_tostring(this->vy);
	result << ", \"vz\":" << float_tostring(this->vz);
	result << ", \"ax\":" << float_tostring(this->ax);
	result << ", \"ay\":" << float_tostring(this->ay);
	result << ", \"az\":" << float_tostring(this->az);
	result << ", \"covariance\":" << "[" << float_array_tostring(45, reinterpret_cast<float*>(&this->covariance[0])) << "]";
	result << ", \"estimator_type\":" << static_cast<unsigned int>(this->estimator_type);
	result << "},";
	return result.str();
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

std::string MavLinkRcChannels::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"RC_CHANNELS\", \"id\": 65";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"chan1_raw\":" << this->chan1_raw;
	result << ", \"chan2_raw\":" << this->chan2_raw;
	result << ", \"chan3_raw\":" << this->chan3_raw;
	result << ", \"chan4_raw\":" << this->chan4_raw;
	result << ", \"chan5_raw\":" << this->chan5_raw;
	result << ", \"chan6_raw\":" << this->chan6_raw;
	result << ", \"chan7_raw\":" << this->chan7_raw;
	result << ", \"chan8_raw\":" << this->chan8_raw;
	result << ", \"chan9_raw\":" << this->chan9_raw;
	result << ", \"chan10_raw\":" << this->chan10_raw;
	result << ", \"chan11_raw\":" << this->chan11_raw;
	result << ", \"chan12_raw\":" << this->chan12_raw;
	result << ", \"chan13_raw\":" << this->chan13_raw;
	result << ", \"chan14_raw\":" << this->chan14_raw;
	result << ", \"chan15_raw\":" << this->chan15_raw;
	result << ", \"chan16_raw\":" << this->chan16_raw;
	result << ", \"chan17_raw\":" << this->chan17_raw;
	result << ", \"chan18_raw\":" << this->chan18_raw;
	result << ", \"chancount\":" << static_cast<unsigned int>(this->chancount);
	result << ", \"rssi\":" << static_cast<unsigned int>(this->rssi);
	result << "},";
	return result.str();
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

std::string MavLinkRequestDataStream::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"REQUEST_DATA_STREAM\", \"id\": 66";
	result << ", \"req_message_rate\":" << this->req_message_rate;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"req_stream_id\":" << static_cast<unsigned int>(this->req_stream_id);
	result << ", \"start_stop\":" << static_cast<unsigned int>(this->start_stop);
	result << "},";
	return result.str();
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

std::string MavLinkDataStream::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"DATA_STREAM\", \"id\": 67";
	result << ", \"message_rate\":" << this->message_rate;
	result << ", \"stream_id\":" << static_cast<unsigned int>(this->stream_id);
	result << ", \"on_off\":" << static_cast<unsigned int>(this->on_off);
	result << "},";
	return result.str();
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

std::string MavLinkManualControl::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MANUAL_CONTROL\", \"id\": 69";
	result << ", \"x\":" << this->x;
	result << ", \"y\":" << this->y;
	result << ", \"z\":" << this->z;
	result << ", \"r\":" << this->r;
	result << ", \"buttons\":" << this->buttons;
	result << ", \"target\":" << static_cast<unsigned int>(this->target);
	result << "},";
	return result.str();
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

std::string MavLinkRcChannelsOverride::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"RC_CHANNELS_OVERRIDE\", \"id\": 70";
	result << ", \"chan1_raw\":" << this->chan1_raw;
	result << ", \"chan2_raw\":" << this->chan2_raw;
	result << ", \"chan3_raw\":" << this->chan3_raw;
	result << ", \"chan4_raw\":" << this->chan4_raw;
	result << ", \"chan5_raw\":" << this->chan5_raw;
	result << ", \"chan6_raw\":" << this->chan6_raw;
	result << ", \"chan7_raw\":" << this->chan7_raw;
	result << ", \"chan8_raw\":" << this->chan8_raw;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkMissionItemInt::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MISSION_ITEM_INT\", \"id\": 73";
	result << ", \"param1\":" << float_tostring(this->param1);
	result << ", \"param2\":" << float_tostring(this->param2);
	result << ", \"param3\":" << float_tostring(this->param3);
	result << ", \"param4\":" << float_tostring(this->param4);
	result << ", \"x\":" << this->x;
	result << ", \"y\":" << this->y;
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"seq\":" << this->seq;
	result << ", \"command\":" << this->command;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"frame\":" << static_cast<unsigned int>(this->frame);
	result << ", \"current\":" << static_cast<unsigned int>(this->current);
	result << ", \"autocontinue\":" << static_cast<unsigned int>(this->autocontinue);
	result << "},";
	return result.str();
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

std::string MavLinkVfrHud::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"VFR_HUD\", \"id\": 74";
	result << ", \"airspeed\":" << float_tostring(this->airspeed);
	result << ", \"groundspeed\":" << float_tostring(this->groundspeed);
	result << ", \"alt\":" << float_tostring(this->alt);
	result << ", \"climb\":" << float_tostring(this->climb);
	result << ", \"heading\":" << this->heading;
	result << ", \"throttle\":" << this->throttle;
	result << "},";
	return result.str();
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

std::string MavLinkCommandInt::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"COMMAND_INT\", \"id\": 75";
	result << ", \"param1\":" << float_tostring(this->param1);
	result << ", \"param2\":" << float_tostring(this->param2);
	result << ", \"param3\":" << float_tostring(this->param3);
	result << ", \"param4\":" << float_tostring(this->param4);
	result << ", \"x\":" << this->x;
	result << ", \"y\":" << this->y;
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"command\":" << this->command;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"frame\":" << static_cast<unsigned int>(this->frame);
	result << ", \"current\":" << static_cast<unsigned int>(this->current);
	result << ", \"autocontinue\":" << static_cast<unsigned int>(this->autocontinue);
	result << "},";
	return result.str();
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

std::string MavLinkCommandLong::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"COMMAND_LONG\", \"id\": 76";
	result << ", \"param1\":" << float_tostring(this->param1);
	result << ", \"param2\":" << float_tostring(this->param2);
	result << ", \"param3\":" << float_tostring(this->param3);
	result << ", \"param4\":" << float_tostring(this->param4);
	result << ", \"param5\":" << float_tostring(this->param5);
	result << ", \"param6\":" << float_tostring(this->param6);
	result << ", \"param7\":" << float_tostring(this->param7);
	result << ", \"command\":" << this->command;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"confirmation\":" << static_cast<unsigned int>(this->confirmation);
	result << "},";
	return result.str();
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

std::string MavLinkCommandAck::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"COMMAND_ACK\", \"id\": 77";
	result << ", \"command\":" << this->command;
	result << ", \"result\":" << static_cast<unsigned int>(this->result);
	result << "},";
	return result.str();
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

std::string MavLinkManualSetpoint::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MANUAL_SETPOINT\", \"id\": 81";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"roll\":" << float_tostring(this->roll);
	result << ", \"pitch\":" << float_tostring(this->pitch);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << ", \"thrust\":" << float_tostring(this->thrust);
	result << ", \"mode_switch\":" << static_cast<unsigned int>(this->mode_switch);
	result << ", \"manual_override_switch\":" << static_cast<unsigned int>(this->manual_override_switch);
	result << "},";
	return result.str();
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

std::string MavLinkSetAttitudeTarget::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SET_ATTITUDE_TARGET\", \"id\": 82";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"q\":" << "[" << float_array_tostring(4, reinterpret_cast<float*>(&this->q[0])) << "]";
	result << ", \"body_roll_rate\":" << float_tostring(this->body_roll_rate);
	result << ", \"body_pitch_rate\":" << float_tostring(this->body_pitch_rate);
	result << ", \"body_yaw_rate\":" << float_tostring(this->body_yaw_rate);
	result << ", \"thrust\":" << float_tostring(this->thrust);
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"type_mask\":" << static_cast<unsigned int>(this->type_mask);
	result << "},";
	return result.str();
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

std::string MavLinkAttitudeTarget::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ATTITUDE_TARGET\", \"id\": 83";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"q\":" << "[" << float_array_tostring(4, reinterpret_cast<float*>(&this->q[0])) << "]";
	result << ", \"body_roll_rate\":" << float_tostring(this->body_roll_rate);
	result << ", \"body_pitch_rate\":" << float_tostring(this->body_pitch_rate);
	result << ", \"body_yaw_rate\":" << float_tostring(this->body_yaw_rate);
	result << ", \"thrust\":" << float_tostring(this->thrust);
	result << ", \"type_mask\":" << static_cast<unsigned int>(this->type_mask);
	result << "},";
	return result.str();
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

std::string MavLinkSetPositionTargetLocalNed::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SET_POSITION_TARGET_LOCAL_NED\", \"id\": 84";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"vx\":" << float_tostring(this->vx);
	result << ", \"vy\":" << float_tostring(this->vy);
	result << ", \"vz\":" << float_tostring(this->vz);
	result << ", \"afx\":" << float_tostring(this->afx);
	result << ", \"afy\":" << float_tostring(this->afy);
	result << ", \"afz\":" << float_tostring(this->afz);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << ", \"yaw_rate\":" << float_tostring(this->yaw_rate);
	result << ", \"type_mask\":" << this->type_mask;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"coordinate_frame\":" << static_cast<unsigned int>(this->coordinate_frame);
	result << "},";
	return result.str();
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

std::string MavLinkPositionTargetLocalNed::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"POSITION_TARGET_LOCAL_NED\", \"id\": 85";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"vx\":" << float_tostring(this->vx);
	result << ", \"vy\":" << float_tostring(this->vy);
	result << ", \"vz\":" << float_tostring(this->vz);
	result << ", \"afx\":" << float_tostring(this->afx);
	result << ", \"afy\":" << float_tostring(this->afy);
	result << ", \"afz\":" << float_tostring(this->afz);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << ", \"yaw_rate\":" << float_tostring(this->yaw_rate);
	result << ", \"type_mask\":" << this->type_mask;
	result << ", \"coordinate_frame\":" << static_cast<unsigned int>(this->coordinate_frame);
	result << "},";
	return result.str();
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

std::string MavLinkSetPositionTargetGlobalInt::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SET_POSITION_TARGET_GLOBAL_INT\", \"id\": 86";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"lat_int\":" << this->lat_int;
	result << ", \"lon_int\":" << this->lon_int;
	result << ", \"alt\":" << float_tostring(this->alt);
	result << ", \"vx\":" << float_tostring(this->vx);
	result << ", \"vy\":" << float_tostring(this->vy);
	result << ", \"vz\":" << float_tostring(this->vz);
	result << ", \"afx\":" << float_tostring(this->afx);
	result << ", \"afy\":" << float_tostring(this->afy);
	result << ", \"afz\":" << float_tostring(this->afz);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << ", \"yaw_rate\":" << float_tostring(this->yaw_rate);
	result << ", \"type_mask\":" << this->type_mask;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"coordinate_frame\":" << static_cast<unsigned int>(this->coordinate_frame);
	result << "},";
	return result.str();
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

std::string MavLinkPositionTargetGlobalInt::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"POSITION_TARGET_GLOBAL_INT\", \"id\": 87";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"lat_int\":" << this->lat_int;
	result << ", \"lon_int\":" << this->lon_int;
	result << ", \"alt\":" << float_tostring(this->alt);
	result << ", \"vx\":" << float_tostring(this->vx);
	result << ", \"vy\":" << float_tostring(this->vy);
	result << ", \"vz\":" << float_tostring(this->vz);
	result << ", \"afx\":" << float_tostring(this->afx);
	result << ", \"afy\":" << float_tostring(this->afy);
	result << ", \"afz\":" << float_tostring(this->afz);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << ", \"yaw_rate\":" << float_tostring(this->yaw_rate);
	result << ", \"type_mask\":" << this->type_mask;
	result << ", \"coordinate_frame\":" << static_cast<unsigned int>(this->coordinate_frame);
	result << "},";
	return result.str();
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

std::string MavLinkLocalPositionNedSystemGlobalOffset::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET\", \"id\": 89";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"roll\":" << float_tostring(this->roll);
	result << ", \"pitch\":" << float_tostring(this->pitch);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << "},";
	return result.str();
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

std::string MavLinkHilState::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIL_STATE\", \"id\": 90";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"roll\":" << float_tostring(this->roll);
	result << ", \"pitch\":" << float_tostring(this->pitch);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << ", \"rollspeed\":" << float_tostring(this->rollspeed);
	result << ", \"pitchspeed\":" << float_tostring(this->pitchspeed);
	result << ", \"yawspeed\":" << float_tostring(this->yawspeed);
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"alt\":" << this->alt;
	result << ", \"vx\":" << this->vx;
	result << ", \"vy\":" << this->vy;
	result << ", \"vz\":" << this->vz;
	result << ", \"xacc\":" << this->xacc;
	result << ", \"yacc\":" << this->yacc;
	result << ", \"zacc\":" << this->zacc;
	result << "},";
	return result.str();
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

std::string MavLinkHilControls::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIL_CONTROLS\", \"id\": 91";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"roll_ailerons\":" << float_tostring(this->roll_ailerons);
	result << ", \"pitch_elevator\":" << float_tostring(this->pitch_elevator);
	result << ", \"yaw_rudder\":" << float_tostring(this->yaw_rudder);
	result << ", \"throttle\":" << float_tostring(this->throttle);
	result << ", \"aux1\":" << float_tostring(this->aux1);
	result << ", \"aux2\":" << float_tostring(this->aux2);
	result << ", \"aux3\":" << float_tostring(this->aux3);
	result << ", \"aux4\":" << float_tostring(this->aux4);
	result << ", \"mode\":" << static_cast<unsigned int>(this->mode);
	result << ", \"nav_mode\":" << static_cast<unsigned int>(this->nav_mode);
	result << "},";
	return result.str();
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

std::string MavLinkHilRcInputsRaw::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIL_RC_INPUTS_RAW\", \"id\": 92";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"chan1_raw\":" << this->chan1_raw;
	result << ", \"chan2_raw\":" << this->chan2_raw;
	result << ", \"chan3_raw\":" << this->chan3_raw;
	result << ", \"chan4_raw\":" << this->chan4_raw;
	result << ", \"chan5_raw\":" << this->chan5_raw;
	result << ", \"chan6_raw\":" << this->chan6_raw;
	result << ", \"chan7_raw\":" << this->chan7_raw;
	result << ", \"chan8_raw\":" << this->chan8_raw;
	result << ", \"chan9_raw\":" << this->chan9_raw;
	result << ", \"chan10_raw\":" << this->chan10_raw;
	result << ", \"chan11_raw\":" << this->chan11_raw;
	result << ", \"chan12_raw\":" << this->chan12_raw;
	result << ", \"rssi\":" << static_cast<unsigned int>(this->rssi);
	result << "},";
	return result.str();
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

std::string MavLinkHilActuatorControls::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIL_ACTUATOR_CONTROLS\", \"id\": 93";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"flags\":" << this->flags;
	result << ", \"controls\":" << "[" << float_array_tostring(16, reinterpret_cast<float*>(&this->controls[0])) << "]";
	result << ", \"mode\":" << static_cast<unsigned int>(this->mode);
	result << "},";
	return result.str();
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

std::string MavLinkOpticalFlow::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"OPTICAL_FLOW\", \"id\": 100";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"flow_comp_m_x\":" << float_tostring(this->flow_comp_m_x);
	result << ", \"flow_comp_m_y\":" << float_tostring(this->flow_comp_m_y);
	result << ", \"ground_distance\":" << float_tostring(this->ground_distance);
	result << ", \"flow_x\":" << this->flow_x;
	result << ", \"flow_y\":" << this->flow_y;
	result << ", \"sensor_id\":" << static_cast<unsigned int>(this->sensor_id);
	result << ", \"quality\":" << static_cast<unsigned int>(this->quality);
	result << "},";
	return result.str();
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

std::string MavLinkGlobalVisionPositionEstimate::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GLOBAL_VISION_POSITION_ESTIMATE\", \"id\": 101";
	result << ", \"usec\":" << this->usec;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"roll\":" << float_tostring(this->roll);
	result << ", \"pitch\":" << float_tostring(this->pitch);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << "},";
	return result.str();
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

std::string MavLinkVisionPositionEstimate::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"VISION_POSITION_ESTIMATE\", \"id\": 102";
	result << ", \"usec\":" << this->usec;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"roll\":" << float_tostring(this->roll);
	result << ", \"pitch\":" << float_tostring(this->pitch);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << "},";
	return result.str();
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

std::string MavLinkVisionSpeedEstimate::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"VISION_SPEED_ESTIMATE\", \"id\": 103";
	result << ", \"usec\":" << this->usec;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << "},";
	return result.str();
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

std::string MavLinkViconPositionEstimate::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"VICON_POSITION_ESTIMATE\", \"id\": 104";
	result << ", \"usec\":" << this->usec;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"roll\":" << float_tostring(this->roll);
	result << ", \"pitch\":" << float_tostring(this->pitch);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << "},";
	return result.str();
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

std::string MavLinkHighresImu::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIGHRES_IMU\", \"id\": 105";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"xacc\":" << float_tostring(this->xacc);
	result << ", \"yacc\":" << float_tostring(this->yacc);
	result << ", \"zacc\":" << float_tostring(this->zacc);
	result << ", \"xgyro\":" << float_tostring(this->xgyro);
	result << ", \"ygyro\":" << float_tostring(this->ygyro);
	result << ", \"zgyro\":" << float_tostring(this->zgyro);
	result << ", \"xmag\":" << float_tostring(this->xmag);
	result << ", \"ymag\":" << float_tostring(this->ymag);
	result << ", \"zmag\":" << float_tostring(this->zmag);
	result << ", \"abs_pressure\":" << float_tostring(this->abs_pressure);
	result << ", \"diff_pressure\":" << float_tostring(this->diff_pressure);
	result << ", \"pressure_alt\":" << float_tostring(this->pressure_alt);
	result << ", \"temperature\":" << float_tostring(this->temperature);
	result << ", \"fields_updated\":" << this->fields_updated;
	result << "},";
	return result.str();
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

std::string MavLinkOpticalFlowRad::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"OPTICAL_FLOW_RAD\", \"id\": 106";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"integration_time_us\":" << this->integration_time_us;
	result << ", \"integrated_x\":" << float_tostring(this->integrated_x);
	result << ", \"integrated_y\":" << float_tostring(this->integrated_y);
	result << ", \"integrated_xgyro\":" << float_tostring(this->integrated_xgyro);
	result << ", \"integrated_ygyro\":" << float_tostring(this->integrated_ygyro);
	result << ", \"integrated_zgyro\":" << float_tostring(this->integrated_zgyro);
	result << ", \"time_delta_distance_us\":" << this->time_delta_distance_us;
	result << ", \"distance\":" << float_tostring(this->distance);
	result << ", \"temperature\":" << this->temperature;
	result << ", \"sensor_id\":" << static_cast<unsigned int>(this->sensor_id);
	result << ", \"quality\":" << static_cast<unsigned int>(this->quality);
	result << "},";
	return result.str();
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

std::string MavLinkHilSensor::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIL_SENSOR\", \"id\": 107";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"xacc\":" << float_tostring(this->xacc);
	result << ", \"yacc\":" << float_tostring(this->yacc);
	result << ", \"zacc\":" << float_tostring(this->zacc);
	result << ", \"xgyro\":" << float_tostring(this->xgyro);
	result << ", \"ygyro\":" << float_tostring(this->ygyro);
	result << ", \"zgyro\":" << float_tostring(this->zgyro);
	result << ", \"xmag\":" << float_tostring(this->xmag);
	result << ", \"ymag\":" << float_tostring(this->ymag);
	result << ", \"zmag\":" << float_tostring(this->zmag);
	result << ", \"abs_pressure\":" << float_tostring(this->abs_pressure);
	result << ", \"diff_pressure\":" << float_tostring(this->diff_pressure);
	result << ", \"pressure_alt\":" << float_tostring(this->pressure_alt);
	result << ", \"temperature\":" << float_tostring(this->temperature);
	result << ", \"fields_updated\":" << this->fields_updated;
	result << "},";
	return result.str();
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

std::string MavLinkSimState::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SIM_STATE\", \"id\": 108";
	result << ", \"q1\":" << float_tostring(this->q1);
	result << ", \"q2\":" << float_tostring(this->q2);
	result << ", \"q3\":" << float_tostring(this->q3);
	result << ", \"q4\":" << float_tostring(this->q4);
	result << ", \"roll\":" << float_tostring(this->roll);
	result << ", \"pitch\":" << float_tostring(this->pitch);
	result << ", \"yaw\":" << float_tostring(this->yaw);
	result << ", \"xacc\":" << float_tostring(this->xacc);
	result << ", \"yacc\":" << float_tostring(this->yacc);
	result << ", \"zacc\":" << float_tostring(this->zacc);
	result << ", \"xgyro\":" << float_tostring(this->xgyro);
	result << ", \"ygyro\":" << float_tostring(this->ygyro);
	result << ", \"zgyro\":" << float_tostring(this->zgyro);
	result << ", \"lat\":" << float_tostring(this->lat);
	result << ", \"lon\":" << float_tostring(this->lon);
	result << ", \"alt\":" << float_tostring(this->alt);
	result << ", \"std_dev_horz\":" << float_tostring(this->std_dev_horz);
	result << ", \"std_dev_vert\":" << float_tostring(this->std_dev_vert);
	result << ", \"vn\":" << float_tostring(this->vn);
	result << ", \"ve\":" << float_tostring(this->ve);
	result << ", \"vd\":" << float_tostring(this->vd);
	result << "},";
	return result.str();
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

std::string MavLinkRadioStatus::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"RADIO_STATUS\", \"id\": 109";
	result << ", \"rxerrors\":" << this->rxerrors;
	result << ", \"fixed\":" << this->fixed;
	result << ", \"rssi\":" << static_cast<unsigned int>(this->rssi);
	result << ", \"remrssi\":" << static_cast<unsigned int>(this->remrssi);
	result << ", \"txbuf\":" << static_cast<unsigned int>(this->txbuf);
	result << ", \"noise\":" << static_cast<unsigned int>(this->noise);
	result << ", \"remnoise\":" << static_cast<unsigned int>(this->remnoise);
	result << "},";
	return result.str();
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

std::string MavLinkFileTransferProtocol::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"FILE_TRANSFER_PROTOCOL\", \"id\": 110";
	result << ", \"target_network\":" << static_cast<unsigned int>(this->target_network);
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"payload\":" << "[" << uint8_t_array_tostring(251, reinterpret_cast<uint8_t*>(&this->payload[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkTimesync::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"TIMESYNC\", \"id\": 111";
	result << ", \"tc1\":" << this->tc1;
	result << ", \"ts1\":" << this->ts1;
	result << "},";
	return result.str();
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

std::string MavLinkCameraTrigger::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"CAMERA_TRIGGER\", \"id\": 112";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"seq\":" << this->seq;
	result << "},";
	return result.str();
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

std::string MavLinkHilGps::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIL_GPS\", \"id\": 113";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"alt\":" << this->alt;
	result << ", \"eph\":" << this->eph;
	result << ", \"epv\":" << this->epv;
	result << ", \"vel\":" << this->vel;
	result << ", \"vn\":" << this->vn;
	result << ", \"ve\":" << this->ve;
	result << ", \"vd\":" << this->vd;
	result << ", \"cog\":" << this->cog;
	result << ", \"fix_type\":" << static_cast<unsigned int>(this->fix_type);
	result << ", \"satellites_visible\":" << static_cast<unsigned int>(this->satellites_visible);
	result << "},";
	return result.str();
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

std::string MavLinkHilOpticalFlow::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIL_OPTICAL_FLOW\", \"id\": 114";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"integration_time_us\":" << this->integration_time_us;
	result << ", \"integrated_x\":" << float_tostring(this->integrated_x);
	result << ", \"integrated_y\":" << float_tostring(this->integrated_y);
	result << ", \"integrated_xgyro\":" << float_tostring(this->integrated_xgyro);
	result << ", \"integrated_ygyro\":" << float_tostring(this->integrated_ygyro);
	result << ", \"integrated_zgyro\":" << float_tostring(this->integrated_zgyro);
	result << ", \"time_delta_distance_us\":" << this->time_delta_distance_us;
	result << ", \"distance\":" << float_tostring(this->distance);
	result << ", \"temperature\":" << this->temperature;
	result << ", \"sensor_id\":" << static_cast<unsigned int>(this->sensor_id);
	result << ", \"quality\":" << static_cast<unsigned int>(this->quality);
	result << "},";
	return result.str();
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

std::string MavLinkHilStateQuaternion::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIL_STATE_QUATERNION\", \"id\": 115";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"attitude_quaternion\":" << "[" << float_array_tostring(4, reinterpret_cast<float*>(&this->attitude_quaternion[0])) << "]";
	result << ", \"rollspeed\":" << float_tostring(this->rollspeed);
	result << ", \"pitchspeed\":" << float_tostring(this->pitchspeed);
	result << ", \"yawspeed\":" << float_tostring(this->yawspeed);
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"alt\":" << this->alt;
	result << ", \"vx\":" << this->vx;
	result << ", \"vy\":" << this->vy;
	result << ", \"vz\":" << this->vz;
	result << ", \"ind_airspeed\":" << this->ind_airspeed;
	result << ", \"true_airspeed\":" << this->true_airspeed;
	result << ", \"xacc\":" << this->xacc;
	result << ", \"yacc\":" << this->yacc;
	result << ", \"zacc\":" << this->zacc;
	result << "},";
	return result.str();
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

std::string MavLinkScaledImu2::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SCALED_IMU2\", \"id\": 116";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"xacc\":" << this->xacc;
	result << ", \"yacc\":" << this->yacc;
	result << ", \"zacc\":" << this->zacc;
	result << ", \"xgyro\":" << this->xgyro;
	result << ", \"ygyro\":" << this->ygyro;
	result << ", \"zgyro\":" << this->zgyro;
	result << ", \"xmag\":" << this->xmag;
	result << ", \"ymag\":" << this->ymag;
	result << ", \"zmag\":" << this->zmag;
	result << "},";
	return result.str();
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

std::string MavLinkLogRequestList::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LOG_REQUEST_LIST\", \"id\": 117";
	result << ", \"start\":" << this->start;
	result << ", \"end\":" << this->end;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkLogEntry::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LOG_ENTRY\", \"id\": 118";
	result << ", \"time_utc\":" << this->time_utc;
	result << ", \"size\":" << this->size;
	result << ", \"id\":" << this->id;
	result << ", \"num_logs\":" << this->num_logs;
	result << ", \"last_log_num\":" << this->last_log_num;
	result << "},";
	return result.str();
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

std::string MavLinkLogRequestData::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LOG_REQUEST_DATA\", \"id\": 119";
	result << ", \"ofs\":" << this->ofs;
	result << ", \"count\":" << this->count;
	result << ", \"id\":" << this->id;
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkLogData::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LOG_DATA\", \"id\": 120";
	result << ", \"ofs\":" << this->ofs;
	result << ", \"id\":" << this->id;
	result << ", \"count\":" << static_cast<unsigned int>(this->count);
	result << ", \"data\":" << "[" << uint8_t_array_tostring(90, reinterpret_cast<uint8_t*>(&this->data[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkLogErase::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LOG_ERASE\", \"id\": 121";
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkLogRequestEnd::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LOG_REQUEST_END\", \"id\": 122";
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkGpsInjectData::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GPS_INJECT_DATA\", \"id\": 123";
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"len\":" << static_cast<unsigned int>(this->len);
	result << ", \"data\":" << "[" << uint8_t_array_tostring(110, reinterpret_cast<uint8_t*>(&this->data[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkGps2Raw::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GPS2_RAW\", \"id\": 124";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"alt\":" << this->alt;
	result << ", \"dgps_age\":" << this->dgps_age;
	result << ", \"eph\":" << this->eph;
	result << ", \"epv\":" << this->epv;
	result << ", \"vel\":" << this->vel;
	result << ", \"cog\":" << this->cog;
	result << ", \"fix_type\":" << static_cast<unsigned int>(this->fix_type);
	result << ", \"satellites_visible\":" << static_cast<unsigned int>(this->satellites_visible);
	result << ", \"dgps_numch\":" << static_cast<unsigned int>(this->dgps_numch);
	result << "},";
	return result.str();
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

std::string MavLinkPowerStatus::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"POWER_STATUS\", \"id\": 125";
	result << ", \"Vcc\":" << this->Vcc;
	result << ", \"Vservo\":" << this->Vservo;
	result << ", \"flags\":" << this->flags;
	result << "},";
	return result.str();
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

std::string MavLinkSerialControl::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SERIAL_CONTROL\", \"id\": 126";
	result << ", \"baudrate\":" << this->baudrate;
	result << ", \"timeout\":" << this->timeout;
	result << ", \"device\":" << static_cast<unsigned int>(this->device);
	result << ", \"flags\":" << static_cast<unsigned int>(this->flags);
	result << ", \"count\":" << static_cast<unsigned int>(this->count);
	result << ", \"data\":" << "[" << uint8_t_array_tostring(70, reinterpret_cast<uint8_t*>(&this->data[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkGpsRtk::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GPS_RTK\", \"id\": 127";
	result << ", \"time_last_baseline_ms\":" << this->time_last_baseline_ms;
	result << ", \"tow\":" << this->tow;
	result << ", \"baseline_a_mm\":" << this->baseline_a_mm;
	result << ", \"baseline_b_mm\":" << this->baseline_b_mm;
	result << ", \"baseline_c_mm\":" << this->baseline_c_mm;
	result << ", \"accuracy\":" << this->accuracy;
	result << ", \"iar_num_hypotheses\":" << this->iar_num_hypotheses;
	result << ", \"wn\":" << this->wn;
	result << ", \"rtk_receiver_id\":" << static_cast<unsigned int>(this->rtk_receiver_id);
	result << ", \"rtk_health\":" << static_cast<unsigned int>(this->rtk_health);
	result << ", \"rtk_rate\":" << static_cast<unsigned int>(this->rtk_rate);
	result << ", \"nsats\":" << static_cast<unsigned int>(this->nsats);
	result << ", \"baseline_coords_type\":" << static_cast<unsigned int>(this->baseline_coords_type);
	result << "},";
	return result.str();
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

std::string MavLinkGps2Rtk::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GPS2_RTK\", \"id\": 128";
	result << ", \"time_last_baseline_ms\":" << this->time_last_baseline_ms;
	result << ", \"tow\":" << this->tow;
	result << ", \"baseline_a_mm\":" << this->baseline_a_mm;
	result << ", \"baseline_b_mm\":" << this->baseline_b_mm;
	result << ", \"baseline_c_mm\":" << this->baseline_c_mm;
	result << ", \"accuracy\":" << this->accuracy;
	result << ", \"iar_num_hypotheses\":" << this->iar_num_hypotheses;
	result << ", \"wn\":" << this->wn;
	result << ", \"rtk_receiver_id\":" << static_cast<unsigned int>(this->rtk_receiver_id);
	result << ", \"rtk_health\":" << static_cast<unsigned int>(this->rtk_health);
	result << ", \"rtk_rate\":" << static_cast<unsigned int>(this->rtk_rate);
	result << ", \"nsats\":" << static_cast<unsigned int>(this->nsats);
	result << ", \"baseline_coords_type\":" << static_cast<unsigned int>(this->baseline_coords_type);
	result << "},";
	return result.str();
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

std::string MavLinkScaledImu3::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SCALED_IMU3\", \"id\": 129";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"xacc\":" << this->xacc;
	result << ", \"yacc\":" << this->yacc;
	result << ", \"zacc\":" << this->zacc;
	result << ", \"xgyro\":" << this->xgyro;
	result << ", \"ygyro\":" << this->ygyro;
	result << ", \"zgyro\":" << this->zgyro;
	result << ", \"xmag\":" << this->xmag;
	result << ", \"ymag\":" << this->ymag;
	result << ", \"zmag\":" << this->zmag;
	result << "},";
	return result.str();
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

std::string MavLinkDataTransmissionHandshake::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"DATA_TRANSMISSION_HANDSHAKE\", \"id\": 130";
	result << ", \"size\":" << this->size;
	result << ", \"width\":" << this->width;
	result << ", \"height\":" << this->height;
	result << ", \"packets\":" << this->packets;
	result << ", \"type\":" << static_cast<unsigned int>(this->type);
	result << ", \"payload\":" << static_cast<unsigned int>(this->payload);
	result << ", \"jpg_quality\":" << static_cast<unsigned int>(this->jpg_quality);
	result << "},";
	return result.str();
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

std::string MavLinkEncapsulatedData::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ENCAPSULATED_DATA\", \"id\": 131";
	result << ", \"seqnr\":" << this->seqnr;
	result << ", \"data\":" << "[" << uint8_t_array_tostring(253, reinterpret_cast<uint8_t*>(&this->data[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkDistanceSensor::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"DISTANCE_SENSOR\", \"id\": 132";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"min_distance\":" << this->min_distance;
	result << ", \"max_distance\":" << this->max_distance;
	result << ", \"current_distance\":" << this->current_distance;
	result << ", \"type\":" << static_cast<unsigned int>(this->type);
	result << ", \"id\":" << static_cast<unsigned int>(this->id);
	result << ", \"orientation\":" << static_cast<unsigned int>(this->orientation);
	result << ", \"covariance\":" << static_cast<unsigned int>(this->covariance);
	result << "},";
	return result.str();
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

std::string MavLinkTerrainRequest::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"TERRAIN_REQUEST\", \"id\": 133";
	result << ", \"mask\":" << this->mask;
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"grid_spacing\":" << this->grid_spacing;
	result << "},";
	return result.str();
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

std::string MavLinkTerrainData::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"TERRAIN_DATA\", \"id\": 134";
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"grid_spacing\":" << this->grid_spacing;
	result << ", \"data\":" << "[" << int16_t_array_tostring(16, reinterpret_cast<int16_t*>(&this->data[0])) << "]";
	result << ", \"gridbit\":" << static_cast<unsigned int>(this->gridbit);
	result << "},";
	return result.str();
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

std::string MavLinkTerrainCheck::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"TERRAIN_CHECK\", \"id\": 135";
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << "},";
	return result.str();
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

std::string MavLinkTerrainReport::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"TERRAIN_REPORT\", \"id\": 136";
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"terrain_height\":" << float_tostring(this->terrain_height);
	result << ", \"current_height\":" << float_tostring(this->current_height);
	result << ", \"spacing\":" << this->spacing;
	result << ", \"pending\":" << this->pending;
	result << ", \"loaded\":" << this->loaded;
	result << "},";
	return result.str();
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

std::string MavLinkScaledPressure2::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SCALED_PRESSURE2\", \"id\": 137";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"press_abs\":" << float_tostring(this->press_abs);
	result << ", \"press_diff\":" << float_tostring(this->press_diff);
	result << ", \"temperature\":" << this->temperature;
	result << "},";
	return result.str();
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

std::string MavLinkAttPosMocap::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ATT_POS_MOCAP\", \"id\": 138";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"q\":" << "[" << float_array_tostring(4, reinterpret_cast<float*>(&this->q[0])) << "]";
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << "},";
	return result.str();
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

std::string MavLinkSetActuatorControlTarget::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SET_ACTUATOR_CONTROL_TARGET\", \"id\": 139";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"controls\":" << "[" << float_array_tostring(8, reinterpret_cast<float*>(&this->controls[0])) << "]";
	result << ", \"group_mlx\":" << static_cast<unsigned int>(this->group_mlx);
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << "},";
	return result.str();
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

std::string MavLinkActuatorControlTarget::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ACTUATOR_CONTROL_TARGET\", \"id\": 140";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"controls\":" << "[" << float_array_tostring(8, reinterpret_cast<float*>(&this->controls[0])) << "]";
	result << ", \"group_mlx\":" << static_cast<unsigned int>(this->group_mlx);
	result << "},";
	return result.str();
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

std::string MavLinkAltitude::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ALTITUDE\", \"id\": 141";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"altitude_monotonic\":" << float_tostring(this->altitude_monotonic);
	result << ", \"altitude_amsl\":" << float_tostring(this->altitude_amsl);
	result << ", \"altitude_local\":" << float_tostring(this->altitude_local);
	result << ", \"altitude_relative\":" << float_tostring(this->altitude_relative);
	result << ", \"altitude_terrain\":" << float_tostring(this->altitude_terrain);
	result << ", \"bottom_clearance\":" << float_tostring(this->bottom_clearance);
	result << "},";
	return result.str();
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

std::string MavLinkResourceRequest::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"RESOURCE_REQUEST\", \"id\": 142";
	result << ", \"request_id\":" << static_cast<unsigned int>(this->request_id);
	result << ", \"uri_type\":" << static_cast<unsigned int>(this->uri_type);
	result << ", \"uri\":" << "[" << uint8_t_array_tostring(120, reinterpret_cast<uint8_t*>(&this->uri[0])) << "]";
	result << ", \"transfer_type\":" << static_cast<unsigned int>(this->transfer_type);
	result << ", \"storage\":" << "[" << uint8_t_array_tostring(120, reinterpret_cast<uint8_t*>(&this->storage[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkScaledPressure3::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SCALED_PRESSURE3\", \"id\": 143";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"press_abs\":" << float_tostring(this->press_abs);
	result << ", \"press_diff\":" << float_tostring(this->press_diff);
	result << ", \"temperature\":" << this->temperature;
	result << "},";
	return result.str();
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

std::string MavLinkFollowTarget::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"FOLLOW_TARGET\", \"id\": 144";
	result << ", \"timestamp\":" << this->timestamp;
	result << ", \"custom_state\":" << this->custom_state;
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"alt\":" << float_tostring(this->alt);
	result << ", \"vel\":" << "[" << float_array_tostring(3, reinterpret_cast<float*>(&this->vel[0])) << "]";
	result << ", \"acc\":" << "[" << float_array_tostring(3, reinterpret_cast<float*>(&this->acc[0])) << "]";
	result << ", \"attitude_q\":" << "[" << float_array_tostring(4, reinterpret_cast<float*>(&this->attitude_q[0])) << "]";
	result << ", \"rates\":" << "[" << float_array_tostring(3, reinterpret_cast<float*>(&this->rates[0])) << "]";
	result << ", \"position_cov\":" << "[" << float_array_tostring(3, reinterpret_cast<float*>(&this->position_cov[0])) << "]";
	result << ", \"est_capabilities\":" << static_cast<unsigned int>(this->est_capabilities);
	result << "},";
	return result.str();
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

std::string MavLinkControlSystemState::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"CONTROL_SYSTEM_STATE\", \"id\": 146";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"x_acc\":" << float_tostring(this->x_acc);
	result << ", \"y_acc\":" << float_tostring(this->y_acc);
	result << ", \"z_acc\":" << float_tostring(this->z_acc);
	result << ", \"x_vel\":" << float_tostring(this->x_vel);
	result << ", \"y_vel\":" << float_tostring(this->y_vel);
	result << ", \"z_vel\":" << float_tostring(this->z_vel);
	result << ", \"x_pos\":" << float_tostring(this->x_pos);
	result << ", \"y_pos\":" << float_tostring(this->y_pos);
	result << ", \"z_pos\":" << float_tostring(this->z_pos);
	result << ", \"airspeed\":" << float_tostring(this->airspeed);
	result << ", \"vel_variance\":" << "[" << float_array_tostring(3, reinterpret_cast<float*>(&this->vel_variance[0])) << "]";
	result << ", \"pos_variance\":" << "[" << float_array_tostring(3, reinterpret_cast<float*>(&this->pos_variance[0])) << "]";
	result << ", \"q\":" << "[" << float_array_tostring(4, reinterpret_cast<float*>(&this->q[0])) << "]";
	result << ", \"roll_rate\":" << float_tostring(this->roll_rate);
	result << ", \"pitch_rate\":" << float_tostring(this->pitch_rate);
	result << ", \"yaw_rate\":" << float_tostring(this->yaw_rate);
	result << "},";
	return result.str();
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

std::string MavLinkBatteryStatus::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"BATTERY_STATUS\", \"id\": 147";
	result << ", \"current_consumed\":" << this->current_consumed;
	result << ", \"energy_consumed\":" << this->energy_consumed;
	result << ", \"temperature\":" << this->temperature;
	result << ", \"voltages\":" << "[" << uint16_t_array_tostring(10, reinterpret_cast<uint16_t*>(&this->voltages[0])) << "]";
	result << ", \"current_battery\":" << this->current_battery;
	result << ", \"id\":" << static_cast<unsigned int>(this->id);
	result << ", \"battery_function\":" << static_cast<unsigned int>(this->battery_function);
	result << ", \"type\":" << static_cast<unsigned int>(this->type);
	result << ", \"battery_remaining\":" << static_cast<int>(this->battery_remaining);
	result << "},";
	return result.str();
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

std::string MavLinkAutopilotVersion::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"AUTOPILOT_VERSION\", \"id\": 148";
	result << ", \"capabilities\":" << this->capabilities;
	result << ", \"uid\":" << this->uid;
	result << ", \"flight_sw_version\":" << this->flight_sw_version;
	result << ", \"middleware_sw_version\":" << this->middleware_sw_version;
	result << ", \"os_sw_version\":" << this->os_sw_version;
	result << ", \"board_version\":" << this->board_version;
	result << ", \"vendor_id\":" << this->vendor_id;
	result << ", \"product_id\":" << this->product_id;
	result << ", \"flight_custom_version\":" << "[" << uint8_t_array_tostring(8, reinterpret_cast<uint8_t*>(&this->flight_custom_version[0])) << "]";
	result << ", \"middleware_custom_version\":" << "[" << uint8_t_array_tostring(8, reinterpret_cast<uint8_t*>(&this->middleware_custom_version[0])) << "]";
	result << ", \"os_custom_version\":" << "[" << uint8_t_array_tostring(8, reinterpret_cast<uint8_t*>(&this->os_custom_version[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkLandingTarget::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"LANDING_TARGET\", \"id\": 149";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"angle_x\":" << float_tostring(this->angle_x);
	result << ", \"angle_y\":" << float_tostring(this->angle_y);
	result << ", \"distance\":" << float_tostring(this->distance);
	result << ", \"size_x\":" << float_tostring(this->size_x);
	result << ", \"size_y\":" << float_tostring(this->size_y);
	result << ", \"target_num\":" << static_cast<unsigned int>(this->target_num);
	result << ", \"frame\":" << static_cast<unsigned int>(this->frame);
	result << "},";
	return result.str();
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

std::string MavLinkEstimatorStatus::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ESTIMATOR_STATUS\", \"id\": 230";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"vel_ratio\":" << float_tostring(this->vel_ratio);
	result << ", \"pos_horiz_ratio\":" << float_tostring(this->pos_horiz_ratio);
	result << ", \"pos_vert_ratio\":" << float_tostring(this->pos_vert_ratio);
	result << ", \"mag_ratio\":" << float_tostring(this->mag_ratio);
	result << ", \"hagl_ratio\":" << float_tostring(this->hagl_ratio);
	result << ", \"tas_ratio\":" << float_tostring(this->tas_ratio);
	result << ", \"pos_horiz_accuracy\":" << float_tostring(this->pos_horiz_accuracy);
	result << ", \"pos_vert_accuracy\":" << float_tostring(this->pos_vert_accuracy);
	result << ", \"flags\":" << this->flags;
	result << "},";
	return result.str();
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

std::string MavLinkWindCov::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"WIND_COV\", \"id\": 231";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"wind_x\":" << float_tostring(this->wind_x);
	result << ", \"wind_y\":" << float_tostring(this->wind_y);
	result << ", \"wind_z\":" << float_tostring(this->wind_z);
	result << ", \"var_horiz\":" << float_tostring(this->var_horiz);
	result << ", \"var_vert\":" << float_tostring(this->var_vert);
	result << ", \"wind_alt\":" << float_tostring(this->wind_alt);
	result << ", \"horiz_accuracy\":" << float_tostring(this->horiz_accuracy);
	result << ", \"vert_accuracy\":" << float_tostring(this->vert_accuracy);
	result << "},";
	return result.str();
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

std::string MavLinkGpsInput::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GPS_INPUT\", \"id\": 232";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"time_week_ms\":" << this->time_week_ms;
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"alt\":" << float_tostring(this->alt);
	result << ", \"hdop\":" << float_tostring(this->hdop);
	result << ", \"vdop\":" << float_tostring(this->vdop);
	result << ", \"vn\":" << float_tostring(this->vn);
	result << ", \"ve\":" << float_tostring(this->ve);
	result << ", \"vd\":" << float_tostring(this->vd);
	result << ", \"speed_accuracy\":" << float_tostring(this->speed_accuracy);
	result << ", \"horiz_accuracy\":" << float_tostring(this->horiz_accuracy);
	result << ", \"vert_accuracy\":" << float_tostring(this->vert_accuracy);
	result << ", \"ignore_flags\":" << this->ignore_flags;
	result << ", \"time_week\":" << this->time_week;
	result << ", \"gps_id\":" << static_cast<unsigned int>(this->gps_id);
	result << ", \"fix_type\":" << static_cast<unsigned int>(this->fix_type);
	result << ", \"satellites_visible\":" << static_cast<unsigned int>(this->satellites_visible);
	result << "},";
	return result.str();
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

std::string MavLinkGpsRtcmData::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"GPS_RTCM_DATA\", \"id\": 233";
	result << ", \"flags\":" << static_cast<unsigned int>(this->flags);
	result << ", \"len\":" << static_cast<unsigned int>(this->len);
	result << ", \"data\":" << "[" << uint8_t_array_tostring(180, reinterpret_cast<uint8_t*>(&this->data[0])) << "]";
	result << "},";
	return result.str();
}

int MavLinkHighLatency::pack(char* buffer) const {
	pack_uint32_t(buffer, reinterpret_cast<const uint32_t*>(&this->custom_mode), 0);
	pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->latitude), 4);
	pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->longitude), 8);
	pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->roll), 12);
	pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->pitch), 14);
	pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->heading), 16);
	pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->heading_sp), 18);
	pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->altitude_amsl), 20);
	pack_int16_t(buffer, reinterpret_cast<const int16_t*>(&this->altitude_sp), 22);
	pack_uint16_t(buffer, reinterpret_cast<const uint16_t*>(&this->wp_distance), 24);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->base_mode), 26);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->landed_state), 27);
	pack_int8_t(buffer, reinterpret_cast<const int8_t*>(&this->throttle), 28);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->airspeed), 29);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->airspeed_sp), 30);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->groundspeed), 31);
	pack_int8_t(buffer, reinterpret_cast<const int8_t*>(&this->climb_rate), 32);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->gps_nsat), 33);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->gps_fix_type), 34);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->battery_remaining), 35);
	pack_int8_t(buffer, reinterpret_cast<const int8_t*>(&this->temperature), 36);
	pack_int8_t(buffer, reinterpret_cast<const int8_t*>(&this->temperature_air), 37);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->failsafe), 38);
	pack_uint8_t(buffer, reinterpret_cast<const uint8_t*>(&this->wp_num), 39);
	return 40;
}

int MavLinkHighLatency::unpack(const char* buffer) {
	unpack_uint32_t(buffer, reinterpret_cast<uint32_t*>(&this->custom_mode), 0);
	unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->latitude), 4);
	unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->longitude), 8);
	unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->roll), 12);
	unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->pitch), 14);
	unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->heading), 16);
	unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->heading_sp), 18);
	unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->altitude_amsl), 20);
	unpack_int16_t(buffer, reinterpret_cast<int16_t*>(&this->altitude_sp), 22);
	unpack_uint16_t(buffer, reinterpret_cast<uint16_t*>(&this->wp_distance), 24);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->base_mode), 26);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->landed_state), 27);
	unpack_int8_t(buffer, reinterpret_cast<int8_t*>(&this->throttle), 28);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->airspeed), 29);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->airspeed_sp), 30);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->groundspeed), 31);
	unpack_int8_t(buffer, reinterpret_cast<int8_t*>(&this->climb_rate), 32);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->gps_nsat), 33);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->gps_fix_type), 34);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->battery_remaining), 35);
	unpack_int8_t(buffer, reinterpret_cast<int8_t*>(&this->temperature), 36);
	unpack_int8_t(buffer, reinterpret_cast<int8_t*>(&this->temperature_air), 37);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->failsafe), 38);
	unpack_uint8_t(buffer, reinterpret_cast<uint8_t*>(&this->wp_num), 39);
	return 40;
}

std::string MavLinkHighLatency::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HIGH_LATENCY\", \"id\": 234";
	result << ", \"custom_mode\":" << this->custom_mode;
	result << ", \"latitude\":" << this->latitude;
	result << ", \"longitude\":" << this->longitude;
	result << ", \"roll\":" << this->roll;
	result << ", \"pitch\":" << this->pitch;
	result << ", \"heading\":" << this->heading;
	result << ", \"heading_sp\":" << this->heading_sp;
	result << ", \"altitude_amsl\":" << this->altitude_amsl;
	result << ", \"altitude_sp\":" << this->altitude_sp;
	result << ", \"wp_distance\":" << this->wp_distance;
	result << ", \"base_mode\":" << static_cast<unsigned int>(this->base_mode);
	result << ", \"landed_state\":" << static_cast<unsigned int>(this->landed_state);
	result << ", \"throttle\":" << static_cast<int>(this->throttle);
	result << ", \"airspeed\":" << static_cast<unsigned int>(this->airspeed);
	result << ", \"airspeed_sp\":" << static_cast<unsigned int>(this->airspeed_sp);
	result << ", \"groundspeed\":" << static_cast<unsigned int>(this->groundspeed);
	result << ", \"climb_rate\":" << static_cast<int>(this->climb_rate);
	result << ", \"gps_nsat\":" << static_cast<unsigned int>(this->gps_nsat);
	result << ", \"gps_fix_type\":" << static_cast<unsigned int>(this->gps_fix_type);
	result << ", \"battery_remaining\":" << static_cast<unsigned int>(this->battery_remaining);
	result << ", \"temperature\":" << static_cast<int>(this->temperature);
	result << ", \"temperature_air\":" << static_cast<int>(this->temperature_air);
	result << ", \"failsafe\":" << static_cast<unsigned int>(this->failsafe);
	result << ", \"wp_num\":" << static_cast<unsigned int>(this->wp_num);
	result << "},";
	return result.str();
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

std::string MavLinkVibration::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"VIBRATION\", \"id\": 241";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"vibration_x\":" << float_tostring(this->vibration_x);
	result << ", \"vibration_y\":" << float_tostring(this->vibration_y);
	result << ", \"vibration_z\":" << float_tostring(this->vibration_z);
	result << ", \"clipping_0\":" << this->clipping_0;
	result << ", \"clipping_1\":" << this->clipping_1;
	result << ", \"clipping_2\":" << this->clipping_2;
	result << "},";
	return result.str();
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

std::string MavLinkHomePosition::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"HOME_POSITION\", \"id\": 242";
	result << ", \"latitude\":" << this->latitude;
	result << ", \"longitude\":" << this->longitude;
	result << ", \"altitude\":" << this->altitude;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"q\":" << "[" << float_array_tostring(4, reinterpret_cast<float*>(&this->q[0])) << "]";
	result << ", \"approach_x\":" << float_tostring(this->approach_x);
	result << ", \"approach_y\":" << float_tostring(this->approach_y);
	result << ", \"approach_z\":" << float_tostring(this->approach_z);
	result << "},";
	return result.str();
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

std::string MavLinkSetHomePosition::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"SET_HOME_POSITION\", \"id\": 243";
	result << ", \"latitude\":" << this->latitude;
	result << ", \"longitude\":" << this->longitude;
	result << ", \"altitude\":" << this->altitude;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"q\":" << "[" << float_array_tostring(4, reinterpret_cast<float*>(&this->q[0])) << "]";
	result << ", \"approach_x\":" << float_tostring(this->approach_x);
	result << ", \"approach_y\":" << float_tostring(this->approach_y);
	result << ", \"approach_z\":" << float_tostring(this->approach_z);
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << "},";
	return result.str();
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

std::string MavLinkMessageInterval::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MESSAGE_INTERVAL\", \"id\": 244";
	result << ", \"interval_us\":" << this->interval_us;
	result << ", \"message_id\":" << this->message_id;
	result << "},";
	return result.str();
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

std::string MavLinkExtendedSysState::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"EXTENDED_SYS_STATE\", \"id\": 245";
	result << ", \"vtol_state\":" << static_cast<unsigned int>(this->vtol_state);
	result << ", \"landed_state\":" << static_cast<unsigned int>(this->landed_state);
	result << "},";
	return result.str();
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

std::string MavLinkAdsbVehicle::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"ADSB_VEHICLE\", \"id\": 246";
	result << ", \"ICAO_address\":" << this->ICAO_address;
	result << ", \"lat\":" << this->lat;
	result << ", \"lon\":" << this->lon;
	result << ", \"altitude\":" << this->altitude;
	result << ", \"heading\":" << this->heading;
	result << ", \"hor_velocity\":" << this->hor_velocity;
	result << ", \"ver_velocity\":" << this->ver_velocity;
	result << ", \"flags\":" << this->flags;
	result << ", \"squawk\":" << this->squawk;
	result << ", \"altitude_type\":" << static_cast<unsigned int>(this->altitude_type);
	result << ", \"callsign\":" << "\"" << char_array_tostring(9, reinterpret_cast<char*>(&this->callsign[0])) << "\"";
	result << ", \"emitter_type\":" << static_cast<unsigned int>(this->emitter_type);
	result << ", \"tslc\":" << static_cast<unsigned int>(this->tslc);
	result << "},";
	return result.str();
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

std::string MavLinkCollision::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"COLLISION\", \"id\": 247";
	result << ", \"id\":" << this->id;
	result << ", \"time_to_minimum_delta\":" << float_tostring(this->time_to_minimum_delta);
	result << ", \"altitude_minimum_delta\":" << float_tostring(this->altitude_minimum_delta);
	result << ", \"horizontal_minimum_delta\":" << float_tostring(this->horizontal_minimum_delta);
	result << ", \"src\":" << static_cast<unsigned int>(this->src);
	result << ", \"action\":" << static_cast<unsigned int>(this->action);
	result << ", \"threat_level\":" << static_cast<unsigned int>(this->threat_level);
	result << "},";
	return result.str();
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

std::string MavLinkV2Extension::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"V2_EXTENSION\", \"id\": 248";
	result << ", \"message_type\":" << this->message_type;
	result << ", \"target_network\":" << static_cast<unsigned int>(this->target_network);
	result << ", \"target_system\":" << static_cast<unsigned int>(this->target_system);
	result << ", \"target_component\":" << static_cast<unsigned int>(this->target_component);
	result << ", \"payload\":" << "[" << uint8_t_array_tostring(249, reinterpret_cast<uint8_t*>(&this->payload[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkMemoryVect::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"MEMORY_VECT\", \"id\": 249";
	result << ", \"address\":" << this->address;
	result << ", \"ver\":" << static_cast<unsigned int>(this->ver);
	result << ", \"type\":" << static_cast<unsigned int>(this->type);
	result << ", \"value\":" << "[" << int8_t_array_tostring(32, reinterpret_cast<int8_t*>(&this->value[0])) << "]";
	result << "},";
	return result.str();
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

std::string MavLinkDebugVect::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"DEBUG_VECT\", \"id\": 250";
	result << ", \"time_usec\":" << this->time_usec;
	result << ", \"x\":" << float_tostring(this->x);
	result << ", \"y\":" << float_tostring(this->y);
	result << ", \"z\":" << float_tostring(this->z);
	result << ", \"name\":" << "\"" << char_array_tostring(10, reinterpret_cast<char*>(&this->name[0])) << "\"";
	result << "},";
	return result.str();
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

std::string MavLinkNamedValueFloat::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"NAMED_VALUE_FLOAT\", \"id\": 251";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"value\":" << float_tostring(this->value);
	result << ", \"name\":" << "\"" << char_array_tostring(10, reinterpret_cast<char*>(&this->name[0])) << "\"";
	result << "},";
	return result.str();
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

std::string MavLinkNamedValueInt::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"NAMED_VALUE_INT\", \"id\": 252";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"value\":" << this->value;
	result << ", \"name\":" << "\"" << char_array_tostring(10, reinterpret_cast<char*>(&this->name[0])) << "\"";
	result << "},";
	return result.str();
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

std::string MavLinkStatustext::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"STATUSTEXT\", \"id\": 253";
	result << ", \"severity\":" << static_cast<unsigned int>(this->severity);
	result << ", \"text\":" << "\"" << char_array_tostring(50, reinterpret_cast<char*>(&this->text[0])) << "\"";
	result << "},";
	return result.str();
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

std::string MavLinkDebug::toJSon() {
	std::ostringstream result;
	result << "{ \"name\": \"DEBUG\", \"id\": 254";
	result << ", \"time_boot_ms\":" << this->time_boot_ms;
	result << ", \"value\":" << float_tostring(this->value);
	result << ", \"ind\":" << static_cast<unsigned int>(this->ind);
	result << "},";
	return result.str();
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
	param1 = Pitch;
	param2 = Roll;
	param3 = Yaw;
	param4 = Wip;
	param5 = Wip2;
	param6 = Wip3;
	param7 = MavMountModeEnumValue;
}
void MavCmdDoMountControl::unpack() {
	Pitch = param1;
	Roll = param2;
	Yaw = param3;
	Wip = param4;
	Wip2 = param5;
	Wip3 = param6;
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
void MavCmdNavSetYawSpeed::pack() {
	param1 = YawAngleTo;
	param2 = Speed;
}
void MavCmdNavSetYawSpeed::unpack() {
	YawAngleTo = param1;
	Speed = param2;
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
	param3 = Wip;
	param4 = Wip2;
	param7 = Wip3;
}
void MavCmdPreflightRebootShutdown::unpack() {
	p0 = param1;
	p02 = param2;
	Wip = param3;
	Wip2 = param4;
	Wip3 = param7;
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
void MavCmdRequestCameraInformation::pack() {
	param1 = p1;
	param2 = CameraId;
}
void MavCmdRequestCameraInformation::unpack() {
	p1 = param1;
	CameraId = param2;
}
void MavCmdRequestCameraSettings::pack() {
	param1 = p1;
	param2 = CameraId;
}
void MavCmdRequestCameraSettings::unpack() {
	p1 = param1;
	CameraId = param2;
}
void MavCmdSetCameraSettings1::pack() {
	param1 = CameraId;
	param2 = Aperture;
	param3 = ApertureLocked;
	param4 = ShutterSpeedS;
	param5 = ShutterSpeedLocked;
	param6 = IsoSensitivity;
	param7 = IsoSensitivityLocked;
}
void MavCmdSetCameraSettings1::unpack() {
	CameraId = param1;
	Aperture = param2;
	ApertureLocked = param3;
	ShutterSpeedS = param4;
	ShutterSpeedLocked = param5;
	IsoSensitivity = param6;
	IsoSensitivityLocked = param7;
}
void MavCmdSetCameraSettings2::pack() {
	param1 = CameraId;
	param2 = WhiteBalanceLocked;
	param3 = WhiteBalance;
	param4 = ReservedForCamera;
	param5 = ReservedForColor;
	param6 = ReservedForImage;
}
void MavCmdSetCameraSettings2::unpack() {
	CameraId = param1;
	WhiteBalanceLocked = param2;
	WhiteBalance = param3;
	ReservedForCamera = param4;
	ReservedForColor = param5;
	ReservedForImage = param6;
}
void MavCmdRequestStorageInformation::pack() {
	param1 = p1;
	param2 = StorageId;
}
void MavCmdRequestStorageInformation::unpack() {
	p1 = param1;
	StorageId = param2;
}
void MavCmdStorageFormat::pack() {
	param1 = p1;
	param2 = StorageId;
}
void MavCmdStorageFormat::unpack() {
	p1 = param1;
	StorageId = param2;
}
void MavCmdRequestCameraCaptureStatus::pack() {
	param1 = p1;
	param2 = CameraId;
}
void MavCmdRequestCameraCaptureStatus::unpack() {
	p1 = param1;
	CameraId = param2;
}
void MavCmdRequestFlightInformation::pack() {
	param1 = p1;
}
void MavCmdRequestFlightInformation::unpack() {
	p1 = param1;
}
void MavCmdImageStartCapture::pack() {
	param1 = DurationBetweenTwo;
	param2 = NumberOfImages;
	param3 = ResolutionMegapixels;
	param4 = Wip;
	param5 = Wip2;
	param6 = Wip3;
}
void MavCmdImageStartCapture::unpack() {
	DurationBetweenTwo = param1;
	NumberOfImages = param2;
	ResolutionMegapixels = param3;
	Wip = param4;
	Wip2 = param5;
	Wip3 = param6;
}
void MavCmdImageStopCapture::pack() {
	param1 = CameraId;
}
void MavCmdImageStopCapture::unpack() {
	CameraId = param1;
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
	param4 = Wip;
	param5 = Wip2;
}
void MavCmdVideoStartCapture::unpack() {
	CameraId = param1;
	FramesPerSecond = param2;
	ResolutionMegapixels = param3;
	Wip = param4;
	Wip2 = param5;
}
void MavCmdVideoStopCapture::pack() {
	param1 = Wip;
}
void MavCmdVideoStopCapture::unpack() {
	Wip = param1;
}
void MavCmdLoggingStart::pack() {
	param1 = Format;
}
void MavCmdLoggingStart::unpack() {
	Format = param1;
}
void MavCmdLoggingStop::pack() {
}
void MavCmdLoggingStop::unpack() {
}
void MavCmdAirframeConfiguration::pack() {
	param1 = LandingGearId;
	param2 = LandingGearPosition;
}
void MavCmdAirframeConfiguration::unpack() {
	LandingGearId = param1;
	LandingGearPosition = param2;
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
MavLinkMessageBase* MavLinkMessageBase::lookup(const MavLinkMessage& msg) {
	MavLinkMessageBase* result = nullptr;
	switch (static_cast<MavLinkMessageIds>(msg.msgid)) {
	case MavLinkMessageIds::MAVLINK_MSG_ID_HEARTBEAT:
		result = new MavLinkHeartbeat();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SYS_STATUS:
		result = new MavLinkSysStatus();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SYSTEM_TIME:
		result = new MavLinkSystemTime();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_PING:
		result = new MavLinkPing();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
		result = new MavLinkChangeOperatorControl();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:
		result = new MavLinkChangeOperatorControlAck();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_AUTH_KEY:
		result = new MavLinkAuthKey();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SET_MODE:
		result = new MavLinkSetMode();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		result = new MavLinkParamRequestRead();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		result = new MavLinkParamRequestList();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_PARAM_VALUE:
		result = new MavLinkParamValue();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_PARAM_SET:
		result = new MavLinkParamSet();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GPS_RAW_INT:
		result = new MavLinkGpsRawInt();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GPS_STATUS:
		result = new MavLinkGpsStatus();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SCALED_IMU:
		result = new MavLinkScaledImu();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_RAW_IMU:
		result = new MavLinkRawImu();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_RAW_PRESSURE:
		result = new MavLinkRawPressure();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SCALED_PRESSURE:
		result = new MavLinkScaledPressure();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ATTITUDE:
		result = new MavLinkAttitude();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
		result = new MavLinkAttitudeQuaternion();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		result = new MavLinkLocalPositionNed();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		result = new MavLinkGlobalPositionInt();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
		result = new MavLinkRcChannelsScaled();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_RC_CHANNELS_RAW:
		result = new MavLinkRcChannelsRaw();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
		result = new MavLinkServoOutputRaw();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
		result = new MavLinkMissionRequestPartialList();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
		result = new MavLinkMissionWritePartialList();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_ITEM:
		result = new MavLinkMissionItem();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_REQUEST:
		result = new MavLinkMissionRequest();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		result = new MavLinkMissionSetCurrent();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_CURRENT:
		result = new MavLinkMissionCurrent();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		result = new MavLinkMissionRequestList();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_COUNT:
		result = new MavLinkMissionCount();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
		result = new MavLinkMissionClearAll();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
		result = new MavLinkMissionItemReached();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_ACK:
		result = new MavLinkMissionAck();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
		result = new MavLinkSetGpsGlobalOrigin();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
		result = new MavLinkGpsGlobalOrigin();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_PARAM_MAP_RC:
		result = new MavLinkParamMapRc();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_REQUEST_INT:
		result = new MavLinkMissionRequestInt();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA:
		result = new MavLinkSafetySetAllowedArea();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA:
		result = new MavLinkSafetyAllowedArea();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV:
		result = new MavLinkAttitudeQuaternionCov();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
		result = new MavLinkNavControllerOutput();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV:
		result = new MavLinkGlobalPositionIntCov();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV:
		result = new MavLinkLocalPositionNedCov();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_RC_CHANNELS:
		result = new MavLinkRcChannels();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
		result = new MavLinkRequestDataStream();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_DATA_STREAM:
		result = new MavLinkDataStream();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MANUAL_CONTROL:
		result = new MavLinkManualControl();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		result = new MavLinkRcChannelsOverride();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MISSION_ITEM_INT:
		result = new MavLinkMissionItemInt();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_VFR_HUD:
		result = new MavLinkVfrHud();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_COMMAND_INT:
		result = new MavLinkCommandInt();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_COMMAND_LONG:
		result = new MavLinkCommandLong();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_COMMAND_ACK:
		result = new MavLinkCommandAck();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MANUAL_SETPOINT:
		result = new MavLinkManualSetpoint();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
		result = new MavLinkSetAttitudeTarget();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ATTITUDE_TARGET:
		result = new MavLinkAttitudeTarget();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		result = new MavLinkSetPositionTargetLocalNed();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		result = new MavLinkPositionTargetLocalNed();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
		result = new MavLinkSetPositionTargetGlobalInt();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
		result = new MavLinkPositionTargetGlobalInt();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:
		result = new MavLinkLocalPositionNedSystemGlobalOffset();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIL_STATE:
		result = new MavLinkHilState();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIL_CONTROLS:
		result = new MavLinkHilControls();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW:
		result = new MavLinkHilRcInputsRaw();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
		result = new MavLinkHilActuatorControls();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_OPTICAL_FLOW:
		result = new MavLinkOpticalFlow();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
		result = new MavLinkGlobalVisionPositionEstimate();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		result = new MavLinkVisionPositionEstimate();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
		result = new MavLinkVisionSpeedEstimate();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
		result = new MavLinkViconPositionEstimate();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIGHRES_IMU:
		result = new MavLinkHighresImu();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
		result = new MavLinkOpticalFlowRad();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIL_SENSOR:
		result = new MavLinkHilSensor();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SIM_STATE:
		result = new MavLinkSimState();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_RADIO_STATUS:
		result = new MavLinkRadioStatus();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
		result = new MavLinkFileTransferProtocol();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_TIMESYNC:
		result = new MavLinkTimesync();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_CAMERA_TRIGGER:
		result = new MavLinkCameraTrigger();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIL_GPS:
		result = new MavLinkHilGps();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
		result = new MavLinkHilOpticalFlow();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
		result = new MavLinkHilStateQuaternion();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SCALED_IMU2:
		result = new MavLinkScaledImu2();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LOG_REQUEST_LIST:
		result = new MavLinkLogRequestList();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LOG_ENTRY:
		result = new MavLinkLogEntry();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LOG_REQUEST_DATA:
		result = new MavLinkLogRequestData();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LOG_DATA:
		result = new MavLinkLogData();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LOG_ERASE:
		result = new MavLinkLogErase();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LOG_REQUEST_END:
		result = new MavLinkLogRequestEnd();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GPS_INJECT_DATA:
		result = new MavLinkGpsInjectData();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GPS2_RAW:
		result = new MavLinkGps2Raw();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_POWER_STATUS:
		result = new MavLinkPowerStatus();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SERIAL_CONTROL:
		result = new MavLinkSerialControl();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GPS_RTK:
		result = new MavLinkGpsRtk();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GPS2_RTK:
		result = new MavLinkGps2Rtk();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SCALED_IMU3:
		result = new MavLinkScaledImu3();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
		result = new MavLinkDataTransmissionHandshake();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ENCAPSULATED_DATA:
		result = new MavLinkEncapsulatedData();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_DISTANCE_SENSOR:
		result = new MavLinkDistanceSensor();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_TERRAIN_REQUEST:
		result = new MavLinkTerrainRequest();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_TERRAIN_DATA:
		result = new MavLinkTerrainData();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_TERRAIN_CHECK:
		result = new MavLinkTerrainCheck();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_TERRAIN_REPORT:
		result = new MavLinkTerrainReport();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SCALED_PRESSURE2:
		result = new MavLinkScaledPressure2();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ATT_POS_MOCAP:
		result = new MavLinkAttPosMocap();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
		result = new MavLinkSetActuatorControlTarget();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
		result = new MavLinkActuatorControlTarget();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ALTITUDE:
		result = new MavLinkAltitude();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_RESOURCE_REQUEST:
		result = new MavLinkResourceRequest();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SCALED_PRESSURE3:
		result = new MavLinkScaledPressure3();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_FOLLOW_TARGET:
		result = new MavLinkFollowTarget();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE:
		result = new MavLinkControlSystemState();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_BATTERY_STATUS:
		result = new MavLinkBatteryStatus();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_AUTOPILOT_VERSION:
		result = new MavLinkAutopilotVersion();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_LANDING_TARGET:
		result = new MavLinkLandingTarget();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ESTIMATOR_STATUS:
		result = new MavLinkEstimatorStatus();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_WIND_COV:
		result = new MavLinkWindCov();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GPS_INPUT:
		result = new MavLinkGpsInput();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_GPS_RTCM_DATA:
		result = new MavLinkGpsRtcmData();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HIGH_LATENCY:
		result = new MavLinkHighLatency();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_VIBRATION:
		result = new MavLinkVibration();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_HOME_POSITION:
		result = new MavLinkHomePosition();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_SET_HOME_POSITION:
		result = new MavLinkSetHomePosition();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MESSAGE_INTERVAL:
		result = new MavLinkMessageInterval();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
		result = new MavLinkExtendedSysState();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_ADSB_VEHICLE:
		result = new MavLinkAdsbVehicle();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_COLLISION:
		result = new MavLinkCollision();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_V2_EXTENSION:
		result = new MavLinkV2Extension();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_MEMORY_VECT:
		result = new MavLinkMemoryVect();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_DEBUG_VECT:
		result = new MavLinkDebugVect();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
		result = new MavLinkNamedValueFloat();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_NAMED_VALUE_INT:
		result = new MavLinkNamedValueInt();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_STATUSTEXT:
		result = new MavLinkStatustext();
		break;
	case MavLinkMessageIds::MAVLINK_MSG_ID_DEBUG:
		result = new MavLinkDebug();
		break;
	}
	if (result != nullptr) {
		result->decode(msg);
	}
	return result;
}
