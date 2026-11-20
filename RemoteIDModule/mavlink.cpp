/*
  mavlink class for handling OpenDroneID messages
 */
#include <Arduino.h>
#include "mavlink.h"
#include "board_config.h"
#include "version.h"
#include "parameters.h"
#if defined(BOARD_TDM2421)
#include "tdm2421_c2.h"
#include "tdm2421_mqtt.h"
#endif

#define SERIAL_BAUD 115200

// TDM2421/Pixhawk position-source policy:
//   1. Native OPEN_DRONE_ID_LOCATION (original ArduRemoteID behaviour)
//   2. GLOBAL_POSITION_INT converted to an OpenDroneID location
//   3. GPS_RAW_INT converted to an OpenDroneID location
// Native OpenDroneID data always wins while it is fresh. GLOBAL_POSITION_INT
// wins over GPS_RAW_INT while it is fresh. EG800K fallback remains in the
// main loop and is used only when no MAVLink location has been received for 5 s.
static uint32_t last_native_odid_location_ms;
static uint32_t last_global_position_ms;
#include "tdm2421_debug.h"
static uint32_t last_gps_raw_ms;
static uint32_t last_mav_rx_report_ms;
static uint32_t mav_rx_frames;
static uint32_t mav_rx_bytes;

// Thống kê kết nối Flight Controller trên Serial1 (UART MAVLink).
static constexpr uint32_t FC_CONNECT_TIMEOUT_MS = 3500;
static uint32_t fc_last_heartbeat_ms = 0;
static uint32_t fc_last_rx_frame_ms = 0;
static uint8_t fc_sysid = 0;
static uint8_t fc_compid = 0;
static uint8_t fc_autopilot = 0;
static uint8_t fc_mav_type = 0;
static uint32_t fc_rx_frames_total = 0;
static uint32_t fc_rx_bytes_total = 0;

enum class MAVRIDLocationSource : uint8_t {
    NONE = 0,
    OPEN_DRONE_ID_LOCATION,
    GLOBAL_POSITION_INT,
    GPS_RAW_INT
};

static MAVRIDLocationSource mav_rid_location_source = MAVRIDLocationSource::NONE;

const char *mavlink_location_source_name(void)
{
    switch (mav_rid_location_source) {
    case MAVRIDLocationSource::OPEN_DRONE_ID_LOCATION: return "OPEN_DRONE_ID_LOCATION";
    case MAVRIDLocationSource::GLOBAL_POSITION_INT:   return "GLOBAL_POSITION_INT";
    case MAVRIDLocationSource::GPS_RAW_INT:          return "GPS_RAW_INT";
    default:                                          return "NONE";
    }
}

bool mavlink_fc_connected(void)
{
    const uint32_t now_ms = millis();
    if (fc_last_heartbeat_ms != 0 && (uint32_t)(now_ms - fc_last_heartbeat_ms) <= FC_CONNECT_TIMEOUT_MS) {
        return true;
    }
    return fc_last_rx_frame_ms != 0 && (uint32_t)(now_ms - fc_last_rx_frame_ms) <= FC_CONNECT_TIMEOUT_MS;
}

String mavlink_fc_status_json(void)
{
    const uint32_t now_ms = millis();
    const bool hb_connected = fc_last_heartbeat_ms != 0 &&
                                (uint32_t)(now_ms - fc_last_heartbeat_ms) <= FC_CONNECT_TIMEOUT_MS;
    const bool connected = mavlink_fc_connected();
    const uint32_t hb_age_ms = fc_last_heartbeat_ms == 0 ? 0 : (uint32_t)(now_ms - fc_last_heartbeat_ms);
    const uint32_t rx_age_ms = fc_last_rx_frame_ms == 0 ? 0 : (uint32_t)(now_ms - fc_last_rx_frame_ms);

    String s = "{";
    s += "\"connected\":" + String(connected ? "true" : "false") + ",";
    s += "\"heartbeatConnected\":" + String(hb_connected ? "true" : "false") + ",";
    s += "\"sysid\":" + String((unsigned)fc_sysid) + ",";
    s += "\"compid\":" + String((unsigned)fc_compid) + ",";
    s += "\"autopilot\":" + String((unsigned)fc_autopilot) + ",";
    s += "\"mavType\":" + String((unsigned)fc_mav_type) + ",";
    s += "\"heartbeatAgeMs\":" + String((unsigned long)hb_age_ms) + ",";
    s += "\"rxAgeMs\":" + String((unsigned long)rx_age_ms) + ",";
    s += "\"locationSource\":\"" + String(mavlink_location_source_name()) + "\",";
    s += "\"rxFrames\":" + String((unsigned long)fc_rx_frames_total) + ",";
    s += "\"rxBytes\":" + String((unsigned long)fc_rx_bytes_total) + ",";
    s += "\"baudrate\":" + String((unsigned long)g.baudrate) + ",";
    s += "\"rxPin\":" + String((int)PIN_UART_RX) + ",";
    s += "\"txPin\":" + String((int)PIN_UART_TX);
    s += "}";
    return s;
}

static float mav_location_timestamp(uint32_t now_ms)
{
    // OPEN_DRONE_ID_LOCATION requires seconds after the full hour.  Standard
    // position messages do not carry UTC seconds-after-hour, so use a monotonic
    // seconds-after-hour value. Keep it non-zero because set_data() uses zero as
    // the invalid/uninitialised sentinel.
    float ts = (now_ms % 3600000UL) * 0.001f;
    return ts > 0.0f ? ts : 0.001f;
}

static uint16_t mav_direction_from_cdeg(uint16_t hdg)
{
    // GLOBAL_POSITION_INT uses UINT16_MAX when heading is unknown.
    return hdg == UINT16_MAX ? 36100U : hdg;
}

static uint16_t mav_speed_horizontal_cms(int16_t vx, int16_t vy)
{
    const float speed = sqrtf(float(vx) * float(vx) + float(vy) * float(vy));
    return (uint16_t)constrain(speed, 0.0f, 25425.0f);
}

// Reject an unset/obviously invalid operator location before it can overwrite
// a previously valid OPEN_DRONE_ID_SYSTEM.  This specifically protects against
// GCS placeholder SYSTEM packets such as lat=0, lon=0, alt=-1000.
static bool valid_operator_location(int32_t lat_e7, int32_t lon_e7, float alt_m)
{
    if (lat_e7 < -900000000L || lat_e7 > 900000000L ||
        lon_e7 < -1800000000L || lon_e7 > 1800000000L) {
        return false;
    }
    if (lat_e7 == 0 && lon_e7 == 0) {
        return false;
    }
    if (!isfinite(alt_m) || alt_m <= -999.0f) {
        return false;
    }
    return true;
}

static HardwareSerial *serial_ports[MAVLINK_COMM_NUM_BUFFERS];

#include <generated/mavlink_helpers.h>

mavlink_system_t mavlink_system = {0, MAV_COMP_ID_ODID_TXRX_1};

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    if (chan >= MAVLINK_COMM_NUM_BUFFERS || serial_ports[uint8_t(chan)] == nullptr) {
        return;
    }
    auto &serial = *serial_ports[uint8_t(chan)];
    serial.write(buf, len);
}

/*
  abstraction for MAVLink on a serial port
 */
MAVLinkSerial::MAVLinkSerial(HardwareSerial &_serial, mavlink_channel_t _chan) :
    serial(_serial),
    chan(_chan)
{
    serial_ports[uint8_t(_chan - MAVLINK_COMM_0)] = &serial;
}

void MAVLinkSerial::init(void)
{
    // print banner at startup
    serial.printf("ArduRemoteID version %u.%u %08x\n",
                  FW_VERSION_MAJOR, FW_VERSION_MINOR, GIT_VERSION);
    mavlink_system.sysid = g.mavlink_sysid;
}

void MAVLinkSerial::update(void)
{
    const uint32_t now_ms = millis();

#if defined(BOARD_TDM2421)
    // Đảm bảo chỉ duy nhất loop() (thông qua mavlink1 trên MAVLINK_COMM_0) ghi ra Serial1.
    // Xả tối đa 4 frame lệnh C2 từ MQTT xuống FC mỗi vòng lặp để tránh đan xen byte với HEARTBEAT/ARM_STATUS.
    if (chan == MAVLINK_COMM_0 && tdm2421_c2.isEnabled()) {
        uint8_t c2_frame[TDM2421_C2_MAX_FRAME_LEN] {};
        uint16_t c2_len = 0;
        for (uint8_t i = 0; i < 4; i++) {
            if (!tdm2421_c2.popUplinkFrame(c2_frame, c2_len)) {
                break;
            }
            serial.write(c2_frame, c2_len);
        }
    }
#endif

    if (mavlink_system.sysid != 0) {
        update_send();
    } else if (g.mavlink_sysid != 0) {
        mavlink_system.sysid = g.mavlink_sysid;
    } else if (now_ms - last_hb_warn_ms >= 2000) {
        last_hb_warn_ms = millis();
        serial.printf("Waiting for heartbeat\n");
    }
    update_receive();

    if (param_request_last_ms != 0 && now_ms - param_request_last_ms > 50) {
        param_request_last_ms = now_ms;
        float value;
        if (param_next->get_as_float(value)) {
            mavlink_msg_param_value_send(chan,
                                         param_next->name, value,
                                         MAV_PARAM_TYPE_REAL32,
                                         g.param_count_float(),
                                         g.param_index_float(param_next));
        }
        param_next++;
        if (param_next->ptype == Parameters::ParamType::NONE) {
            param_next = nullptr;
            param_request_last_ms = 0;
        }
    }
}

void MAVLinkSerial::update_send(void)
{
    uint32_t now_ms = millis();
    if (now_ms - last_hb_ms >= 1000) {
        last_hb_ms = now_ms;
        mavlink_msg_heartbeat_send(
            chan,
            MAV_TYPE_ODID,
            MAV_AUTOPILOT_INVALID,
            0,
            0,
            0);

        // send arming status
        arm_status_send();
    }
}

void MAVLinkSerial::update_receive(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;

    const uint16_t nbytes = serial.available();
    mav_rx_bytes += nbytes;
    fc_rx_bytes_total += nbytes;
    for (uint16_t i=0; i<nbytes; i++) {
        const uint8_t c = (uint8_t)serial.read();
        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            mav_rx_frames++;
            process_packet(status, msg);
#if defined(BOARD_TDM2421)
            // Chỉ đẩy vào C2 downlink khi đã nối xong MQTT để tránh đếm TXdrop giả lúc khởi động 4G.
            if (chan == MAVLINK_COMM_0 && tdm2421_c2.isEnabled() &&
                tdm2421_mqtt.isMqttConnected() &&
                !(msg.sysid == mavlink_system.sysid && msg.compid == MAV_COMP_ID_ODID_TXRX_1)) {
                uint8_t raw[MAVLINK_MAX_PACKET_LEN] {};
                const uint16_t raw_len = mavlink_msg_to_send_buffer(raw, &msg);
                tdm2421_c2.pushDownlinkFrame(raw, raw_len, millis());
            }
#endif
        }
    }

#if defined(BOARD_TDM2421)
    const uint32_t now_ms = millis();
    if ((mav_rx_bytes > 0 || mav_rx_frames > 0) &&
        now_ms - last_mav_rx_report_ms >= 2000) {
        last_mav_rx_report_ms = now_ms;
        tdmDebugPrintf(2, "[MAV RX] bytes=%lu frames=%lu sysid=%u age_location=%lu ms\r\n",
                      (unsigned long)mav_rx_bytes,
                      (unsigned long)mav_rx_frames,
                      (unsigned)mavlink_system.sysid,
                      last_location_ms == 0 ? 0UL :
                      (unsigned long)(now_ms - last_location_ms));
        mav_rx_bytes = 0;
        mav_rx_frames = 0;
    }
#endif
}

/*
  printf via MAVLink STATUSTEXT for debugging
 */
void MAVLinkSerial::mav_printf(uint8_t severity, const char *fmt, ...)
{
    va_list arg_list;
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};
    va_start(arg_list, fmt);
    vsnprintf(text, sizeof(text), fmt, arg_list);
    va_end(arg_list);
    mavlink_msg_statustext_send(chan,
                                severity,
                                text,
                                0, 0);
}

void MAVLinkSerial::process_packet(mavlink_status_t &status, mavlink_message_t &msg)
{
    const uint32_t now_ms = millis();
    fc_rx_frames_total++;
    fc_last_rx_frame_ms = now_ms;
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        if (msg.sysid > 0 && hb.type != MAV_TYPE_GCS &&
            msg.compid != MAV_COMP_ID_ODID_TXRX_1) {
            fc_last_heartbeat_ms = now_ms;
            fc_sysid = msg.sysid;
            fc_compid = msg.compid;
            fc_autopilot = hb.autopilot;
            fc_mav_type = hb.type;
            if (mavlink_system.sysid == 0) {
                mavlink_system.sysid = msg.sysid;
            }
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION: {
        // Highest priority: preserve the original ArduRemoteID behaviour.
        // When the autopilot sends a native OpenDroneID Location message, use
        // its RID fields directly instead of rebuilding them from generic GPS.
        mavlink_msg_open_drone_id_location_decode(&msg, &location);
        last_native_odid_location_ms = now_ms;
        mav_rid_location_source = MAVRIDLocationSource::OPEN_DRONE_ID_LOCATION;
#if defined(BOARD_TDM2421)
        tdmDebugPrintf(2, "[MAV RX] OPEN_DRONE_ID_LOCATION lat=%.7f lon=%.7f alt=%.1f h=%.1f speed=%.2f\r\n",
                      location.latitude * 1.0e-7,
                      location.longitude * 1.0e-7,
                      location.altitude_geodetic,
                      location.height,
                      location.speed_horizontal * 0.01f);
#else
        if (g.options & OPTIONS_PRINT_RID_MAVLINK) {
            tdmDebugPrintf(2, "MAVLink: got Location\r\n");
        }
#endif
        if (last_location_timestamp != location.timestamp) {
            last_location_timestamp = location.timestamp;
        }
        last_location_ms = now_ms;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        // Generic MAVLink position fallback. Do not overwrite a fresh native
        // OPEN_DRONE_ID_LOCATION message from the autopilot.
        if (last_native_odid_location_ms != 0 &&
            now_ms - last_native_odid_location_ms <= 2000) {
            break;
        }

        mavlink_global_position_int_t p;
        mavlink_msg_global_position_int_decode(&msg, &p);
        if (p.lat == 0 && p.lon == 0) {
            break;
        }

        memset(&location, 0, sizeof(location));
        location.target_system = 0;
        location.target_component = 0;
        location.status = MAV_ODID_STATUS_AIRBORNE;
        location.direction = mav_direction_from_cdeg(p.hdg);
        location.speed_horizontal = mav_speed_horizontal_cms(p.vx, p.vy);
        // GLOBAL_POSITION_INT vz is positive down; OpenDroneID is positive up.
        location.speed_vertical = (int16_t)constrain(-(int32_t)p.vz, -6200L, 6200L);
        location.latitude = p.lat;
        location.longitude = p.lon;
        location.altitude_geodetic = p.alt * 0.001f;
        // GLOBAL_POSITION_INT does not provide a separate barometric altitude.
        // Use the best MAVLink altitude available rather than an invalid sentinel.
        location.altitude_barometric = location.altitude_geodetic;
        location.height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;
        location.height = p.relative_alt * 0.001f;
        location.horizontal_accuracy = MAV_ODID_HOR_ACC_UNKNOWN;
        location.vertical_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
        location.barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
        location.speed_accuracy = MAV_ODID_SPEED_ACC_UNKNOWN;
        location.timestamp = mav_location_timestamp(now_ms);
        location.timestamp_accuracy = MAV_ODID_TIME_ACC_UNKNOWN;

        last_global_position_ms = now_ms;
        mav_rid_location_source = MAVRIDLocationSource::GLOBAL_POSITION_INT;
        last_location_ms = now_ms;
        last_location_timestamp = location.timestamp;

#if defined(BOARD_TDM2421)
        tdmDebugPrintf(2, "[MAV RX] GLOBAL_POSITION_INT lat=%.7f lon=%.7f alt=%.1f rel=%.1f speed=%.2f hdg=%.1f\r\n",
                      location.latitude * 1.0e-7,
                      location.longitude * 1.0e-7,
                      location.altitude_geodetic, location.height,
                      location.speed_horizontal * 0.01f,
                      location.direction <= 35999 ? location.direction * 0.01f : -1.0f);
#endif
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        // Last-resort MAVLink GPS source. Prefer native OpenDroneID and
        // GLOBAL_POSITION_INT whenever either is fresh.
        if ((last_native_odid_location_ms != 0 &&
             now_ms - last_native_odid_location_ms <= 2000) ||
            (last_global_position_ms != 0 &&
             now_ms - last_global_position_ms <= 2000)) {
            break;
        }

        mavlink_gps_raw_int_t p;
        mavlink_msg_gps_raw_int_decode(&msg, &p);
        if (p.fix_type < 2 || (p.lat == 0 && p.lon == 0)) {
            break;
        }

        memset(&location, 0, sizeof(location));
        location.target_system = 0;
        location.target_component = 0;
        location.status = MAV_ODID_STATUS_AIRBORNE;
        location.direction = p.cog == UINT16_MAX ? 36100U : p.cog;
        location.speed_horizontal = p.vel == UINT16_MAX ? 25500U :
                                    (uint16_t)constrain((uint32_t)p.vel, 0UL, 25425UL);
        location.speed_vertical = 6300; // unknown in GPS_RAW_INT
        location.latitude = p.lat;
        location.longitude = p.lon;
        location.altitude_geodetic = p.alt * 0.001f;
        location.altitude_barometric = location.altitude_geodetic;
        location.height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;
        location.height = -1000.0f; // GPS_RAW_INT has no relative/takeoff height
        location.horizontal_accuracy = MAV_ODID_HOR_ACC_UNKNOWN;
        location.vertical_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
        location.barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
        location.speed_accuracy = MAV_ODID_SPEED_ACC_UNKNOWN;
        location.timestamp = mav_location_timestamp(now_ms);
        location.timestamp_accuracy = MAV_ODID_TIME_ACC_UNKNOWN;

        last_gps_raw_ms = now_ms;
        mav_rid_location_source = MAVRIDLocationSource::GPS_RAW_INT;
        last_location_ms = now_ms;
        last_location_timestamp = location.timestamp;

#if defined(BOARD_TDM2421)
        tdmDebugPrintf(2, "[MAV RX] GPS_RAW_INT fix=%u sats=%u lat=%.7f lon=%.7f alt=%.1f speed=%.2f\r\n",
                      (unsigned)p.fix_type, (unsigned)p.satellites_visible,
                      location.latitude * 1.0e-7,
                      location.longitude * 1.0e-7,
                      location.altitude_geodetic,
                      location.speed_horizontal == 25500U ? -1.0f :
                      location.speed_horizontal * 0.01f);
#endif
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID: {
        mavlink_open_drone_id_basic_id_t basic_id_tmp;
        if (g.options & OPTIONS_PRINT_RID_MAVLINK) {
            tdmDebugPrintf(2, "MAVLink: got BasicID\r\n");
        }
        mavlink_msg_open_drone_id_basic_id_decode(&msg, &basic_id_tmp);
#if defined(BOARD_TDM2421)
        char dbg_uas_id[21] = {};
        memcpy(dbg_uas_id, basic_id_tmp.uas_id, 20);
        tdmDebugPrintf(2, "[MAV RID RX] BASIC_ID sys=%u comp=%u id_type=%u ua_type=%u uas_id='%s'\r\n",
                      (unsigned)msg.sysid, (unsigned)msg.compid,
                      (unsigned)basic_id_tmp.id_type, (unsigned)basic_id_tmp.ua_type, dbg_uas_id);
#endif
        if ((strlen((const char*) basic_id_tmp.uas_id) > 0) && (basic_id_tmp.id_type > 0) && (basic_id_tmp.id_type <= MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID)) {
            //only update if we receive valid data
            basic_id = basic_id_tmp;
            last_basic_id_ms = now_ms;
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION: {
        mavlink_msg_open_drone_id_authentication_decode(&msg, &authentication);
        if (g.options & OPTIONS_PRINT_RID_MAVLINK) {
            tdmDebugPrintf(2, "MAVLink: got Auth\r\n");
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID: {
        mavlink_msg_open_drone_id_self_id_decode(&msg, &self_id);
#if defined(BOARD_TDM2421)
        char dbg_self_desc[24] = {};
        memcpy(dbg_self_desc, self_id.description, 23);
        tdmDebugPrintf(2, "[MAV RID RX] SELF_ID sys=%u comp=%u type=%u description='%s'\r\n",
                      (unsigned)msg.sysid, (unsigned)msg.compid,
                      (unsigned)self_id.description_type, dbg_self_desc);
#endif
        if (g.options & OPTIONS_PRINT_RID_MAVLINK) {
            tdmDebugPrintf(2, "MAVLink: got SelfID\r\n");
        }
        last_self_id_ms = now_ms;
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM: {
        mavlink_open_drone_id_system_t system_tmp;
        mavlink_msg_open_drone_id_system_decode(&msg, &system_tmp);
        const bool op_loc_valid = valid_operator_location(system_tmp.operator_latitude,
                                                          system_tmp.operator_longitude,
                                                          system_tmp.operator_altitude_geo);
#if defined(BOARD_TDM2421)
        tdmDebugPrintf(2, "[MAV RID RX] SYSTEM sys=%u comp=%u op_loc_type=%u op_lat=%.7f op_lon=%.7f op_alt=%.1f area_count=%u radius=%u ts=%lu %s\r\n",
                      (unsigned)msg.sysid, (unsigned)msg.compid,
                      (unsigned)system_tmp.operator_location_type,
                      system_tmp.operator_latitude * 1.0e-7, system_tmp.operator_longitude * 1.0e-7,
                      system_tmp.operator_altitude_geo, (unsigned)system_tmp.area_count,
                      (unsigned)system_tmp.area_radius, (unsigned long)system_tmp.timestamp,
                      op_loc_valid ? "ACCEPT" : "REJECT_INVALID_OP_LOC");
#endif
        if (!op_loc_valid) {
            // Do not allow an unset GCS SYSTEM packet to destroy the last good
            // operator location. Other RID messages are still accepted.
            break;
        }
        system = system_tmp;
        if (g.options & OPTIONS_PRINT_RID_MAVLINK) {
            tdmDebugPrintf(2, "MAVLink: got System\r\n");
        }
        if ((last_system_timestamp != system.timestamp) || (system.timestamp == 0)) {
            //only update the timestamp if we receive information with a different timestamp
            last_system_ms = millis();
            last_system_timestamp = system.timestamp;
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE: {
        if (g.options & OPTIONS_PRINT_RID_MAVLINK) {
            tdmDebugPrintf(2, "MAVLink: got System update\r\n");
        }
        mavlink_open_drone_id_system_update_t pkt_system_update;
        mavlink_msg_open_drone_id_system_update_decode(&msg, &pkt_system_update);
#if defined(BOARD_TDM2421)
        tdmDebugPrintf(2, "[MAV RID RX] SYSTEM_UPDATE sys=%u comp=%u op_lat=%.7f op_lon=%.7f op_alt=%.1f ts=%lu\r\n",
                      (unsigned)msg.sysid, (unsigned)msg.compid,
                      pkt_system_update.operator_latitude * 1.0e-7,
                      pkt_system_update.operator_longitude * 1.0e-7,
                      pkt_system_update.operator_altitude_geo,
                      (unsigned long)pkt_system_update.timestamp);
#endif
        const bool op_loc_valid = valid_operator_location(pkt_system_update.operator_latitude,
                                                          pkt_system_update.operator_longitude,
                                                          pkt_system_update.operator_altitude_geo);
        if (!op_loc_valid) {
#if defined(BOARD_TDM2421)
            tdmDebugPrintf(2, "[MAV RID RX] SYSTEM_UPDATE REJECT_INVALID_OP_LOC sys=%u comp=%u\r\n",
                          (unsigned)msg.sysid, (unsigned)msg.compid);
#endif
            break;
        }
        system.operator_latitude = pkt_system_update.operator_latitude;
        system.operator_longitude = pkt_system_update.operator_longitude;
        system.operator_altitude_geo = pkt_system_update.operator_altitude_geo;
        system.timestamp = pkt_system_update.timestamp;
        if (last_system_ms != 0) {
            // we can only mark system as updated if we have the other
            // information already
            if ((last_system_timestamp != system.timestamp) || (pkt_system_update.timestamp == 0)) {
                //only update the timestamp if we receive information with a different timestamp
                last_system_ms = millis();
                last_system_timestamp = pkt_system_update.timestamp;
            }
            last_system_ms = now_ms;
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID: {
        mavlink_msg_open_drone_id_operator_id_decode(&msg, &operator_id);
#if defined(BOARD_TDM2421)
        char dbg_operator_id[21] = {};
        memcpy(dbg_operator_id, operator_id.operator_id, 20);
        tdmDebugPrintf(2, "[MAV RID RX] OPERATOR_ID sys=%u comp=%u type=%u id='%s'\r\n",
                      (unsigned)msg.sysid, (unsigned)msg.compid,
                      (unsigned)operator_id.operator_id_type, dbg_operator_id);
#endif
        if (g.options & OPTIONS_PRINT_RID_MAVLINK) {
            tdmDebugPrintf(2, "MAVLink: got OperatorID\r\n");
        }
        last_operator_id_ms = now_ms;
        break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        param_next = g.find_by_index_float(0);
        param_request_last_ms = millis();
        break;
    };
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        mavlink_param_request_read_t pkt;
        mavlink_msg_param_request_read_decode(&msg, &pkt);
        const Parameters::Param *p;
        if (pkt.param_index < 0) {
            p = g.find(pkt.param_id);
        } else {
            p = g.find_by_index_float(pkt.param_index);
        }
        float value;
        if (!p || !p->get_as_float(value)) {
            return;
        }
        if (p != nullptr && (p->flags & PARAM_FLAG_HIDDEN)) {
            return;
        }
        mavlink_msg_param_value_send(chan,
                                     p->name, value,
                                     MAV_PARAM_TYPE_REAL32,
                                     g.param_count_float(),
                                     g.param_index_float(p));
        break;
    }
    case MAVLINK_MSG_ID_PARAM_SET: {
        mavlink_param_set_t pkt;
        mavlink_msg_param_set_decode(&msg, &pkt);
        auto *p = g.find(pkt.param_id);
        float value;
        if (pkt.param_type != MAV_PARAM_TYPE_REAL32 ||
            !p || !p->get_as_float(value)) {
            return;
        }
        p->get_as_float(value);
        if (g.lock_level > 0 &&
            (strcmp(p->name, "LOCK_LEVEL") != 0 ||
             uint8_t(pkt.param_value) <= uint8_t(value))) {
            // only param set allowed is to increase lock level
            mav_printf(MAV_SEVERITY_ERROR, "Parameters locked");
        } else {
            p->set_as_float(pkt.param_value);
        }
        p->get_as_float(value);
        mavlink_msg_param_value_send(chan,
                                     p->name, value,
                                     MAV_PARAM_TYPE_REAL32,
                                     g.param_count_float(),
                                     g.param_index_float(p));
        break;
    }
    case MAVLINK_MSG_ID_SECURE_COMMAND:
    case MAVLINK_MSG_ID_SECURE_COMMAND_REPLY: {
        mavlink_secure_command_t pkt;
        mavlink_msg_secure_command_decode(&msg, &pkt);
        handle_secure_command(pkt);
        break;
    }
    default:
        // we don't care about other packets
        break;
    }
}

void MAVLinkSerial::arm_status_send(void)
{
    const uint8_t status = parse_fail==nullptr?MAV_ODID_ARM_STATUS_GOOD_TO_ARM:MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC;
    const char *reason = parse_fail==nullptr?"":parse_fail;
#if defined(BOARD_TDM2421)
    static uint32_t last_arm_debug_ms;
    const uint32_t now_ms = millis();
    if (now_ms - last_arm_debug_ms >= 1000) {
        last_arm_debug_ms = now_ms;
        tdmDebugPrintf(2, "[MAV RID TX] ARM_STATUS status=%u reason='%s'\r\n",
                      (unsigned)status, reason);
    }
#endif
    mavlink_msg_open_drone_id_arm_status_send(
        chan,
        status,
        reason);
}

#if defined(BOARD_TDM2421)
bool mavlink_parse_c2_uplink_byte(uint8_t c, uint8_t *out_frame, uint16_t &out_len, bool &bad_crc)
{
    out_len = 0;
    bad_crc = false;
    if (out_frame == nullptr) {
        return false;
    }
    // Dùng bộ đệm & trạng thái tĩnh riêng biệt với MAVLINK_COMM_0 của Serial1
    // để parse luồng byte từ MQTT C2 RX mà không xung đột với bộ thu FC.
    static mavlink_message_t c2_rx_msg {};
    static mavlink_status_t c2_rx_status {};
    mavlink_message_t ready_msg {};
    mavlink_status_t ready_status {};

    const uint8_t res = mavlink_frame_char_buffer(
        &c2_rx_msg, &c2_rx_status, c, &ready_msg, &ready_status);
    if (res == MAVLINK_FRAMING_BAD_CRC) {
        bad_crc = true;
        return false;
    }
    if (res != MAVLINK_FRAMING_OK) {
        return false;
    }
    out_len = mavlink_msg_to_send_buffer(out_frame, &ready_msg);
    return out_len > 0 && out_len <= TDM2421_C2_MAX_FRAME_LEN;
}
#endif
