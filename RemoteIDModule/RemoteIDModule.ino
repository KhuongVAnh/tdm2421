/*
  implement OpenDroneID MAVLink and DroneCAN support
 */
/*
  released under GNU GPL v2 or later
 */

#include "options.h"
#include <Arduino.h>
#include "version.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <opendroneid.h>
#include "mavlink.h"
#include "DroneCAN.h"
#include "WiFi_TX.h"
#include "BLE_TX.h"
#include <esp_wifi.h>
#include <WiFi.h>
#include "parameters.h"
#include "webinterface.h"
#include "check_firmware.h"
#include <esp_ota_ops.h>
#include "efuse.h"
#include "led.h"
#include "tdm2421_mqtt.h"


#if AP_DRONECAN_ENABLED
static DroneCAN dronecan;
#endif

#if AP_MAVLINK_ENABLED
static MAVLinkSerial mavlink1{Serial1, MAVLINK_COMM_0};
#if !defined(BOARD_TDM2421)
static MAVLinkSerial mavlink2{Serial,  MAVLINK_COMM_1};
#endif
#endif

static WiFi_TX wifi;
static BLE_TX ble;

#define DEBUG_BAUDRATE 57600

// OpenDroneID output data structure
ODID_UAS_Data UAS_data;
String status_reason;
String status_data_source = "BOOT";
bool status_mavlink_connected = false;
uint32_t status_mavlink_age_ms = 0;
static uint32_t last_location_ms;
static WebInterface webif;

// -----------------------------------------------------------------------------
// GENERIC SAFE FAKE TEST MODE (ESP32-C3 / ESP32-S3)
// Injects known test Remote ID values into UAS_data and prints the encoded
// Location status over the debug Serial port. The normal common WiFi/BLE
// transmit section still runs, so the fake Remote ID can be broadcast without
// a Pixhawk/GPS source. Board-specific services (for example TDM2421 MQTT)
// remain protected by their own BOARD_* guards.
// -----------------------------------------------------------------------------
#if defined(ENABLE_RID_FAKE_TEST)
#define RID_FAKE_TEST_MODE 1
#else
#define RID_FAKE_TEST_MODE 0
#endif

#if RID_FAKE_TEST_MODE
static constexpr int32_t FAKE_LAT = 210677783;     // 21.0677783 deg
static constexpr int32_t FAKE_LON = 1058113251;    // 105.8113251 deg
static constexpr float   FAKE_ALT_M = 50.0f;       // 50 m
static constexpr float   FAKE_SPEED_MPS = 5.0f;    // 5 m/s
static constexpr float   FAKE_HEADING_DEG = 90.0f; // 90 deg

static void fill_fake_test_location(void)
{
    odid_initUasData(&UAS_data);

    // --------------------------------------------------
    // FAKE BASIC ID
    // --------------------------------------------------

    UAS_data.BasicID[0].UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
    UAS_data.BasicID[0].IDType = ODID_IDTYPE_SERIAL_NUMBER;

    // ODID_COPY_STR(
    //     UAS_data.BasicID[0].UASID,
    //     "1866ASDD0000VT1"
    // );
    strncpy(
        UAS_data.BasicID[0].UASID,
        "1866ASDD0000VT4",
        sizeof(UAS_data.BasicID[0].UASID) - 1
    );

    UAS_data.BasicID[0].UASID[
        sizeof(UAS_data.BasicID[0].UASID) - 1
    ] = '\0';    

    UAS_data.BasicIDValid[0] = 1;

    // --------------------------------------------------
    // FAKE LOCATION
    // --------------------------------------------------
    UAS_data.Location.Status = ODID_STATUS_AIRBORNE;
    UAS_data.Location.Direction = FAKE_HEADING_DEG;
    UAS_data.Location.SpeedHorizontal = FAKE_SPEED_MPS;
    UAS_data.Location.SpeedVertical = 0.0f;
    UAS_data.Location.Latitude = FAKE_LAT * 1.0e-7;
    UAS_data.Location.Longitude = FAKE_LON * 1.0e-7;
    UAS_data.Location.AltitudeBaro = FAKE_ALT_M;
    UAS_data.Location.AltitudeGeo = FAKE_ALT_M;
    UAS_data.Location.Height = FAKE_ALT_M;
    UAS_data.Location.TimeStamp = 1.0f;
    UAS_data.LocationValid = 1;
}

static void run_fake_test_mode(void)
{
    // Always refresh fake UAS_data so the normal WiFi/BLE transmit section
    // can broadcast the fake Location on every loop iteration.
    fill_fake_test_location();

    // Keep USB debug output to 1 Hz only. Do not return from loop().
    static uint32_t last_print_ms;
    uint32_t now_ms = millis();
    if (now_ms - last_print_ms < 1000) {
        return;
    }
    last_print_ms = now_ms;

    ODID_Location_encoded encoded {};
    const int enc_result = encodeLocationMessage(&encoded, &UAS_data.Location);

    Serial.printf(
        "[RID TEST] lat=%.7f lon=%.7f alt=%.1f m speed=%.1f m/s hdg=%.1f deg encode=%s\r\n",
        UAS_data.Location.Latitude,
        UAS_data.Location.Longitude,
        UAS_data.Location.AltitudeGeo,
        UAS_data.Location.SpeedHorizontal,
        UAS_data.Location.Direction,
        enc_result == ODID_SUCCESS ? "OK" : "FAIL");
}
#endif


#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

static bool arm_check_ok = false; // goes true for LED arm check status
static bool pfst_check_ok = false;

/*
  setup serial ports
 */
void setup()
{
    // disable brownout checking
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

#if defined(BOARD_TDM2421)
    // IMPORTANT for TDM2421 VIN power-up:
    // Enable the EG800K power rail immediately, before g.init(), WiFi,
    // and the rest of the application are initialised.  This matches the
    // known-good Arduino test sketch, which drives SIM_PWR_EN (GPIO2) HIGH
    // at the very beginning of setup().
    pinMode(PIN_SIM_PWR_EN, OUTPUT);
    digitalWrite(PIN_SIM_PWR_EN, HIGH);
    delay(500);
#endif

    g.init();

    led.set_state(Led::LedState::INIT);
    led.update();

    if (g.webserver_enable) {
        // need WiFi for web server
        wifi.init();
    }

    // Serial for debug printf
    Serial.begin(g.baudrate);
#if defined(BOARD_TDM2421)
    delay(50);
    Serial.printf("[RID] BasicID config: valid=%s UAType=%u IDType=%u UASID='%s'\r\n",
                  g.have_basic_id_info() ? "YES" : "NO",
                  (unsigned)g.ua_type, (unsigned)g.id_type, g.uas_id);
#endif

    // Serial1 for MAVLink
    Serial1.begin(g.baudrate, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

#if defined(BOARD_TDM2421)
    // TDM2421 UART0 is physically wired to Quectel EG800K.
    // Serial remains native USB CDC and is used for debug.
    // SIM_PWR_EN (GPIO2) was already asserted at the very start of setup().
    Serial.printf("[TDM2421] SIM power: GPIO%d = HIGH (enabled early)\r\n",
                  (int)PIN_SIM_PWR_EN);

    pinMode(PIN_SIM_DTR, OUTPUT);
    digitalWrite(PIN_SIM_DTR, HIGH);
    Serial.printf("[TDM2421] SIM DTR: GPIO%d = HIGH\r\n", (int)PIN_SIM_DTR);

    Serial0.begin(SIM_UART_BAUD, SERIAL_8N1,
                  PIN_SIM_UART_RX, PIN_SIM_UART_TX);
    Serial.printf("[TDM2421] EG800K UART started: baud=%u RX=GPIO%d TX=GPIO%d\r\n",
                  (unsigned)SIM_UART_BAUD,
                  (int)PIN_SIM_UART_RX, (int)PIN_SIM_UART_TX);

    // Start LTE/MQTT worker task. It runs independently from the
    // timing-sensitive Remote ID WiFi/BLE loop.
    tdm2421_mqtt.begin();
#endif

    // set all fields to invalid/initial values
    odid_initUasData(&UAS_data);

#if AP_MAVLINK_ENABLED
    mavlink1.init();
#if !defined(BOARD_TDM2421)
    mavlink2.init();
#endif
#endif
#if AP_DRONECAN_ENABLED
    dronecan.init();
#endif

    set_efuses();

    CheckFirmware::check_OTA_running();

#if defined(PIN_CAN_EN)
    // optional CAN enable pin
    pinMode(PIN_CAN_EN, OUTPUT);
    digitalWrite(PIN_CAN_EN, HIGH);
#endif

#if defined(PIN_CAN_nSILENT)
    // disable silent pin
    pinMode(PIN_CAN_nSILENT, OUTPUT);
    digitalWrite(PIN_CAN_nSILENT, HIGH);
#endif

#if defined(PIN_CAN_TERM)
#if !defined(CAN_TERM_EN)
#define CAN_TERM_EN HIGH
#endif

    // optional CAN termination control
    pinMode(PIN_CAN_TERM, OUTPUT);

    if (g.can_term == 1) {
        digitalWrite(PIN_CAN_TERM, CAN_TERM_EN);
    } else {
        digitalWrite(PIN_CAN_TERM, !CAN_TERM_EN);
    }
#endif

#if defined(BUZZER_PIN)
    //set BuZZER OUTPUT ACTIVE, just to show it works
    pinMode(GPIO_NUM_39, OUTPUT);
    digitalWrite(GPIO_NUM_39, HIGH);
#endif

    pfst_check_ok = true;   // note - this will need to be expanded to better capture PFST test status

    // initially set LED for fail
    led.set_state(Led::LedState::ARM_FAIL);

    esp_log_level_set("*", ESP_LOG_DEBUG);

    esp_ota_mark_app_valid_cancel_rollback();
}

#define IMIN(x,y) ((x)<(y)?(x):(y))
#define ODID_COPY_STR(to, from) strncpy(to, (const char*)from, IMIN(sizeof(to), sizeof(from)))

/*
  check parsing of UAS_data, this checks ranges of values to ensure we
  will produce a valid pack
  returns nullptr on no error, or a string error
 */
static const char *check_parse(void)
{
    String ret = "";

    {
        ODID_Location_encoded encoded {};
        if (encodeLocationMessage(&encoded, &UAS_data.Location) != ODID_SUCCESS) {
            ret += "LOC ";
        }
    }
    {
        ODID_System_encoded encoded {};
        if (encodeSystemMessage(&encoded, &UAS_data.System) != ODID_SUCCESS) {
            ret += "SYS ";
        }
    }
    {
        ODID_BasicID_encoded encoded {};
        if (UAS_data.BasicIDValid[0] == 1) {
            if (encodeBasicIDMessage(&encoded, &UAS_data.BasicID[0]) != ODID_SUCCESS) {
                ret += "ID_1 ";
            }
        }
        memset(&encoded, 0, sizeof(encoded));
        if (UAS_data.BasicIDValid[1] == 1) {
            if (encodeBasicIDMessage(&encoded, &UAS_data.BasicID[1]) != ODID_SUCCESS) {
                ret += "ID_2 ";
            }
        }
    }
    {
        ODID_SelfID_encoded encoded {};
        if (encodeSelfIDMessage(&encoded, &UAS_data.SelfID) != ODID_SUCCESS) {
            ret += "SELF_ID ";
        }
    }
    {
        ODID_OperatorID_encoded encoded {};
        if (encodeOperatorIDMessage(&encoded, &UAS_data.OperatorID) != ODID_SUCCESS) {
            ret += "OP_ID ";
        }
    }
    if (ret.length() > 0) {
        // if all errors would occur in this function, it will fit in
        // 50 chars that is also the max for the arm status message
        static char return_string[50];
        memset(return_string, 0, sizeof(return_string));
        snprintf(return_string, sizeof(return_string)-1, "bad %s data", ret.c_str());
        return return_string;
    }
    return nullptr;
}

/*
  fill in UAS_data from MAVLink packets
 */
static void set_data(Transport &t)
{
    const auto &operator_id = t.get_operator_id();
    const auto &basic_id = t.get_basic_id();
    const auto &system = t.get_system();
    const auto &self_id = t.get_self_id();
    const auto &location = t.get_location();

    odid_initUasData(&UAS_data);

    /*
      if we don't have BasicID info from parameters and we have it
      from the DroneCAN or MAVLink transport then copy it to the
      parameters to persist it. This makes it possible to set the
      UAS_ID string via a MAVLink BASIC_ID message and also offers a
      migration path from the old approach of GCS setting these values
      to having them as parameters

      BasicID 2 can be set in parameters, or provided via mavlink We
      don't persist the BasicID2 if provided via mavlink to allow
      users to change BasicID2 on different days
     */
    if (!g.have_basic_id_info() && !(g.options & OPTIONS_DONT_SAVE_BASIC_ID_TO_PARAMETERS)) {
        if (basic_id.ua_type != 0 &&
            basic_id.id_type != 0 &&
            strnlen((const char *)basic_id.uas_id, 20) > 0) {
            g.set_by_name_uint8("UAS_TYPE", basic_id.ua_type);
            g.set_by_name_uint8("UAS_ID_TYPE", basic_id.id_type);
            char uas_id[21] {};
            ODID_COPY_STR(uas_id, basic_id.uas_id);
            g.set_by_name_string("UAS_ID", uas_id);
        }
    }

    // BasicID source priority for RID broadcast:
    //   1) Fresh/available MAVLink OPEN_DRONE_ID_BASIC_ID -> BasicID[0]
    //   2) Existing local/parameter Basic ID -> BasicID[0] fallback
    // A configured second Basic ID is kept as BasicID[1].  Crucially, a
    // different MAVLink Basic ID is NOT demoted to BasicID[1], because WiFi/
    // BLE receivers normally present BasicID[0] as the primary UAS identity.
    const bool mav_basic_id_valid =
        basic_id.ua_type != 0 &&
        basic_id.id_type != 0 &&
        strnlen((const char *)basic_id.uas_id, 20) > 0;

    if (mav_basic_id_valid) {
        UAS_data.BasicID[0].UAType = (ODID_uatype_t)basic_id.ua_type;
        UAS_data.BasicID[0].IDType = (ODID_idtype_t)basic_id.id_type;
        ODID_COPY_STR(UAS_data.BasicID[0].UASID, basic_id.uas_id);
        UAS_data.BasicIDValid[0] = 1;

        // Preserve an explicitly configured Basic ID 2 only when it is
        // different from the MAVLink primary ID.
        if (g.have_basic_id_2_info() &&
            strcmp((const char *)g.uas_id_2, (const char *)basic_id.uas_id) != 0) {
            UAS_data.BasicID[1].UAType = (ODID_uatype_t)g.ua_type_2;
            UAS_data.BasicID[1].IDType = (ODID_idtype_t)g.id_type_2;
            ODID_COPY_STR(UAS_data.BasicID[1].UASID, g.uas_id_2);
            UAS_data.BasicIDValid[1] = 1;
        }
    } else if (g.have_basic_id_info()) {
        // No usable MAVLink Basic ID: fall back to the existing local config.
        UAS_data.BasicID[0].UAType = (ODID_uatype_t)g.ua_type;
        UAS_data.BasicID[0].IDType = (ODID_idtype_t)g.id_type;
        ODID_COPY_STR(UAS_data.BasicID[0].UASID, g.uas_id);
        UAS_data.BasicIDValid[0] = 1;

        if (g.have_basic_id_2_info()) {
            UAS_data.BasicID[1].UAType = (ODID_uatype_t)g.ua_type_2;
            UAS_data.BasicID[1].IDType = (ODID_idtype_t)g.id_type_2;
            ODID_COPY_STR(UAS_data.BasicID[1].UASID, g.uas_id_2);
            UAS_data.BasicIDValid[1] = 1;
        }
    }

    // OperatorID
    if (strlen(operator_id.operator_id) > 0) {
        UAS_data.OperatorID.OperatorIdType = (ODID_operatorIdType_t)operator_id.operator_id_type;
        ODID_COPY_STR(UAS_data.OperatorID.OperatorId, operator_id.operator_id);
        UAS_data.OperatorIDValid = 1;
    }

    // SelfID
    if (strlen(self_id.description) > 0) {
        UAS_data.SelfID.DescType = (ODID_desctype_t)self_id.description_type;
        ODID_COPY_STR(UAS_data.SelfID.Desc, self_id.description);
        UAS_data.SelfIDValid = 1;
    }

    // System
    if (system.timestamp != 0) {
        UAS_data.System.OperatorLocationType = (ODID_operator_location_type_t)system.operator_location_type;
        UAS_data.System.ClassificationType = (ODID_classification_type_t)system.classification_type;
        UAS_data.System.OperatorLatitude = system.operator_latitude * 1.0e-7;
        UAS_data.System.OperatorLongitude = system.operator_longitude * 1.0e-7;
        UAS_data.System.AreaCount = system.area_count;
        UAS_data.System.AreaRadius = system.area_radius;
        UAS_data.System.AreaCeiling = system.area_ceiling;
        UAS_data.System.AreaFloor = system.area_floor;
        UAS_data.System.CategoryEU = (ODID_category_EU_t)system.category_eu;
        UAS_data.System.ClassEU = (ODID_class_EU_t)system.class_eu;
        UAS_data.System.OperatorAltitudeGeo = system.operator_altitude_geo;
        UAS_data.System.Timestamp = system.timestamp;
        UAS_data.SystemValid = 1;
    }

    // Location
    if (location.timestamp != 0) {
        UAS_data.Location.Status = (ODID_status_t)location.status;
        UAS_data.Location.Direction = location.direction*0.01;
        UAS_data.Location.SpeedHorizontal = location.speed_horizontal*0.01;
        UAS_data.Location.SpeedVertical = location.speed_vertical*0.01;
        UAS_data.Location.Latitude = location.latitude*1.0e-7;
        UAS_data.Location.Longitude = location.longitude*1.0e-7;
        UAS_data.Location.AltitudeBaro = location.altitude_barometric;
        UAS_data.Location.AltitudeGeo = location.altitude_geodetic;
        UAS_data.Location.HeightType = (ODID_Height_reference_t)location.height_reference;
        UAS_data.Location.Height = location.height;
        UAS_data.Location.HorizAccuracy = (ODID_Horizontal_accuracy_t)location.horizontal_accuracy;
        UAS_data.Location.VertAccuracy = (ODID_Vertical_accuracy_t)location.vertical_accuracy;
        UAS_data.Location.BaroAccuracy = (ODID_Vertical_accuracy_t)location.barometer_accuracy;
        UAS_data.Location.SpeedAccuracy = (ODID_Speed_accuracy_t)location.speed_accuracy;
        UAS_data.Location.TSAccuracy = (ODID_Timestamp_accuracy_t)location.timestamp_accuracy;
        UAS_data.Location.TimeStamp = location.timestamp;
        UAS_data.LocationValid = 1;        
    }

    // Arm-status evaluation is deliberately deferred until AFTER source
    // arbitration in loop(). set_data() begins with odid_initUasData(), so
    // checking here could emit a transient LOC failure before the latched
    // last-good MAVLink location is restored.

    uint32_t now_ms = millis();
    uint32_t location_age_ms = now_ms - t.get_last_location_ms();
    uint32_t last_location_age_ms = now_ms - last_location_ms;
    if (location_age_ms < last_location_age_ms) {
        last_location_ms = t.get_last_location_ms();
    }
}

static uint8_t loop_counter = 0;

void loop()
{
    uint32_t now_ms = millis();

#if RID_FAKE_TEST_MODE
    // Fake test mode:
    // - fill UAS_data with known fake Location values
    // - do NOT call set_data(), otherwise MAVLink/DroneCAN would reset/overwrite UAS_data
    // - continue to the common WiFi/BLE transmit section below
    run_fake_test_mode();

    led.update();
    status_reason = "RID FAKE TEST";
    status_data_source = "FAKE_TEST";
    status_mavlink_connected = false;
    status_mavlink_age_ms = 0;

    // The local TDM2421 dashboard must also remain responsive in fake-test mode.
    if (g.webserver_enable) {
        webif.update();
    }
#if defined(BOARD_TDM2421)
    // Fake-test coordinates are intentionally valid test data.
    tdm2421_mqtt.submit(UAS_data, g.wifi_channel);
#endif

#else
    // Normal mode: receive OpenDroneID data from MAVLink / DroneCAN.
#if AP_MAVLINK_ENABLED
    mavlink1.update();
#if !defined(BOARD_TDM2421)
    mavlink2.update();
#endif
#endif
#if AP_DRONECAN_ENABLED
    dronecan.update();
#endif

    // IMPORTANT: MAVLink receive handlers timestamp packets with millis().
    // Refresh now_ms AFTER update(); otherwise a packet received a few ms
    // later than the loop-start timestamp makes unsigned age subtraction wrap
    // to ~UINT32_MAX and falsely selects EG800K for one iteration.
    now_ms = millis();

    // the transports have common static data, so we can just use the
    // first for status
#if AP_MAVLINK_ENABLED
    auto &transport = mavlink1;
#elif AP_DRONECAN_ENABLED
    auto &transport = dronecan;
#else
    #error "Must enable DroneCAN or MAVLink"
#endif

    const uint32_t transport_last_location_ms = transport.get_last_location_ms();
    const uint32_t last_system_ms = transport.get_last_system_ms();
    // Source hysteresis: keep MAVLink selected through short gaps between
    // OPEN_DRONE_ID_LOCATION packets.  This prevents a one-loop switch to
    // EG800K and then immediately back to MAVLink.
    static constexpr uint32_t MAV_LOCATION_HOLD_MS  = 6500;
    const uint32_t mav_location_age_ms = transport_last_location_ms == 0 ? UINT32_MAX :
                                         now_ms - transport_last_location_ms;
    const bool mav_location_fresh = (transport_last_location_ms != 0 &&
                                     mav_location_age_ms <= MAV_LOCATION_HOLD_MS);

    status_mavlink_connected = mav_location_fresh;
    status_mavlink_age_ms = transport_last_location_ms == 0 ? 0 :
                            now_ms - transport_last_location_ms;

    led.update();
    status_reason = "";

    // Always collect BasicID/System/SelfID/OperatorID and the most recent
    // MAVLink location first. If MAVLink location is stale/missing, the
    // location fields are overwritten below with a fresh EG800K GNSS fix.
    set_data(transport);

#if defined(BOARD_TDM2421)
    // ------------------------------------------------------------------
    // v7: sticky MAVLink RID identity latch, independent of POSITION.
    //
    // Important difference from v6: only a NEW MAVLink identity packet is
    // allowed to update a latch. Merely reading the Transport cache again
    // cannot replace/clear a previously accepted identity. Therefore, after
    // MAVLink disappears, BasicID/OperatorID/System remain the last-good
    // values while POSITION can fall back to EG800K GNSS.
    // ------------------------------------------------------------------
    static ODID_BasicID_data last_good_mav_basic_id {};
    static ODID_OperatorID_data last_good_mav_operator_id {};
    static ODID_System_data last_good_mav_system {};
    static bool have_last_good_mav_basic_id = false;
    static bool have_last_good_mav_operator_id = false;
    static bool have_last_good_mav_system = false;
    static uint32_t latched_basic_id_ms = 0;
    static uint32_t latched_operator_id_ms = 0;
    static uint32_t latched_system_ms = 0;

    const auto &rx_basic_id = transport.get_basic_id();
    const auto &rx_operator_id = transport.get_operator_id();
    const uint32_t rx_basic_id_ms = transport.get_last_basic_id_ms();
    const uint32_t rx_operator_id_ms = transport.get_last_operator_id_ms();

    // BASIC_ID: update only on a newly received, valid packet.
    if (rx_basic_id_ms != 0 && rx_basic_id_ms != latched_basic_id_ms) {
        latched_basic_id_ms = rx_basic_id_ms;
        const bool valid =
            rx_basic_id.ua_type != 0 &&
            rx_basic_id.id_type != 0 &&
            strnlen((const char *)rx_basic_id.uas_id, 20) > 0;
        if (valid) {
            last_good_mav_basic_id.UAType = (ODID_uatype_t)rx_basic_id.ua_type;
            last_good_mav_basic_id.IDType = (ODID_idtype_t)rx_basic_id.id_type;
            ODID_COPY_STR(last_good_mav_basic_id.UASID, rx_basic_id.uas_id);
            have_last_good_mav_basic_id = true;
            Serial.printf("[RID ID LATCH] BASIC_ID='%s'\r\n",
                          last_good_mav_basic_id.UASID);
        }
    }

    // OPERATOR_ID: an empty/missing later packet never clears a valid latch.
    if (rx_operator_id_ms != 0 && rx_operator_id_ms != latched_operator_id_ms) {
        latched_operator_id_ms = rx_operator_id_ms;
        if (strnlen((const char *)rx_operator_id.operator_id, 20) > 0) {
            last_good_mav_operator_id.OperatorIdType =
                (ODID_operatorIdType_t)rx_operator_id.operator_id_type;
            ODID_COPY_STR(last_good_mav_operator_id.OperatorId,
                          rx_operator_id.operator_id);
            have_last_good_mav_operator_id = true;
            Serial.printf("[RID ID LATCH] OPERATOR_ID='%s'\r\n",
                          last_good_mav_operator_id.OperatorId);
        }
    }

    // SYSTEM: mavlink.cpp rejects invalid 0/0/-1000 operator locations.
    // Latch only when a NEW accepted SYSTEM has reached UAS_data.
    if (last_system_ms != 0 && last_system_ms != latched_system_ms) {
        latched_system_ms = last_system_ms;
        if (UAS_data.SystemValid) {
            last_good_mav_system = UAS_data.System;
            have_last_good_mav_system = true;
            Serial.printf("[RID ID LATCH] SYSTEM pilotLat=%.7f pilotLon=%.7f\r\n",
                          last_good_mav_system.OperatorLatitude,
                          last_good_mav_system.OperatorLongitude);
        }
    }

    // Restore sticky identities after set_data(), which reinitializes UAS_data.
    // Never use onboard EG800K coordinates as operator/pilot coordinates.
    if (have_last_good_mav_basic_id) {
        UAS_data.BasicID[0] = last_good_mav_basic_id;
        UAS_data.BasicIDValid[0] = 1;
    }
    if (have_last_good_mav_operator_id) {
        UAS_data.OperatorID = last_good_mav_operator_id;
        UAS_data.OperatorIDValid = 1;
    }
    if (have_last_good_mav_system) {
        UAS_data.System = last_good_mav_system;
        UAS_data.SystemValid = 1;
    }

    TDM2421GNSSFix eg_fix {};
    const bool eg_gnss_fresh = tdm2421_mqtt.getGNSSFix(eg_fix, 5000);
    bool selected_location_valid = false;

    // Last-good MAVLink location latch.  set_data() clears UAS_data every
    // iteration, so retain a complete decoded ODID location independently.
    // EG800K is not allowed to replace this snapshot until the MAVLink
    // transport location itself has genuinely exceeded MAV_LOCATION_HOLD_MS.
    static ODID_Location_data last_good_mav_location {};
    static bool have_last_good_mav_location = false;
    static uint32_t last_latched_transport_location_ms = 0;

    if (transport_last_location_ms != 0 &&
        transport_last_location_ms != last_latched_transport_location_ms &&
        UAS_data.LocationValid) {
        last_good_mav_location = UAS_data.Location;
        have_last_good_mav_location = true;
        last_latched_transport_location_ms = transport_last_location_ms;
    }

    if (mav_location_fresh && have_last_good_mav_location) {
        // Priority 1: latched Autopilot/MAVLink location.
        UAS_data.Location = last_good_mav_location;
        UAS_data.LocationValid = 1;
        status_data_source = "MAVLink";
        selected_location_valid = true;
    } else if (eg_gnss_fresh) {
        // Priority 2: direct EG800K GNSS fallback.
        UAS_data.Location.Status = ODID_STATUS_AIRBORNE;
        UAS_data.Location.Direction = eg_fix.course_deg;
        UAS_data.Location.SpeedHorizontal = eg_fix.speed_mps;
        UAS_data.Location.SpeedVertical = 0.0f;
        UAS_data.Location.Latitude = eg_fix.latitude;
        UAS_data.Location.Longitude = eg_fix.longitude;
        UAS_data.Location.AltitudeGeo = eg_fix.altitude_m;
        UAS_data.Location.AltitudeBaro = eg_fix.altitude_m;
        UAS_data.Location.HeightType = ODID_HEIGHT_REF_OVER_TAKEOFF;
        UAS_data.Location.Height = eg_fix.altitude_m;
        UAS_data.Location.TimeStamp = 0.0f;
        UAS_data.LocationValid = 1;
        selected_location_valid = true;
        status_data_source = "EG800K_GNSS";
        status_reason = "MAVLink location unavailable; using EG800K GNSS";
    } else {
        status_data_source = "NO_LOCATION";
        UAS_data.Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
        UAS_data.LocationValid = 0;
        status_reason = "No fresh MAVLink or EG800K GNSS location";
    }
#else
    status_data_source = "MAVLink";
    if (!mav_location_fresh) {
        UAS_data.Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
    }
#endif

    // A stale/missing System message must not destroy a valid EG800K fallback
    // position. System validity remains whatever set_data() received.
    if (mav_location_fresh &&
        (last_system_ms == 0 || now_ms - last_system_ms > 5000)) {
        UAS_data.Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
    }

    // Evaluate ARM_STATUS only after the final location source has been
    // selected/restored. This prevents transient LOC/SYS errors caused by
    // odid_initUasData() inside set_data().
    const char *reason = check_parse();
    transport.arm_status_check(reason);
    transport.set_parse_fail(reason);
    arm_check_ok = (reason == nullptr);
    if (g.options & OPTIONS_FORCE_ARM_OK) {
        arm_check_ok = true;
    }
    led.set_state(pfst_check_ok && arm_check_ok ? Led::LedState::ARM_OK : Led::LedState::ARM_FAIL);
    if (reason != nullptr) {
        status_reason = String(reason);
    }

    // web update has to happen after source selection/status update.
    if (g.webserver_enable) {
        webif.update();
    }

#if defined(BOARD_TDM2421)
    // Explicitly report the source that is actually feeding RID/MQTT.
    // This is intentionally after set_data()/fallback selection so the values
    // printed here are the same values that will be broadcast and submitted.
    static uint32_t last_rid_source_report_ms = 0;
    static String last_rid_source_reported = "";
    String rid_source_detail = status_data_source;
    if (mav_location_fresh) {
        rid_source_detail += "/";
        rid_source_detail += mavlink_location_source_name();
    }
    if (rid_source_detail != last_rid_source_reported ||
        now_ms - last_rid_source_report_ms >= 1000) {
        last_rid_source_report_ms = now_ms;
        last_rid_source_reported = rid_source_detail;
        Serial.printf("[RID SOURCE] %s age=%lu ms valid=%u lat=%.7f lon=%.7f alt=%.1f height=%.1f speed=%.2f\r\n",
                      rid_source_detail.c_str(),
                      mav_location_fresh ? (unsigned long)status_mavlink_age_ms : 0UL,
                      (unsigned)UAS_data.LocationValid,
                      UAS_data.Location.Latitude,
                      UAS_data.Location.Longitude,
                      UAS_data.Location.AltitudeGeo,
                      UAS_data.Location.Height,
                      UAS_data.Location.SpeedHorizontal);
    }

    // IMPORTANT: submit to MQTT before bcast_powerup can temporarily force
    // LocationValid=1 for RID power-up broadcasts. MQTT must never publish
    // the OpenDroneID invalid sentinels (0/0, -1000 m, 255 m/s) as a real fix.
    if (selected_location_valid) {
        // Keep source diagnostics readable: report MQTT source only on a
        // source change or once per second, not on every loop/submission.
        static uint32_t last_mqtt_source_report_ms = 0;
        static String last_mqtt_source_reported = "";
        if (rid_source_detail != last_mqtt_source_reported ||
            now_ms - last_mqtt_source_report_ms >= 1000) {
            last_mqtt_source_report_ms = now_ms;
            last_mqtt_source_reported = rid_source_detail;
            Serial.printf("[MQTT SOURCE] %s\r\n", rid_source_detail.c_str());
        }
        // Single source of truth for RID identity: UAS_data.
        // The sticky MAVLink identity latch above is applied to UAS_data BEFORE
        // this point, so WiFi/BLE broadcast and MQTT always see the same
        // BasicID/OperatorID. MQTT must not independently select/latch an ID.
        tdm2421_mqtt.submit(UAS_data, g.wifi_channel);
    }
#endif

    if (g.bcast_powerup) {
        if (!UAS_data.LocationValid) {
            UAS_data.Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
            UAS_data.LocationValid = 1;
        }
    } else {
        // Broadcast only when at least one real location source is fresh.
#if defined(BOARD_TDM2421)
        if (!mav_location_fresh && !eg_gnss_fresh) {
            delay(1);
            return;
        }
#else
        if (!mav_location_fresh) {
            delay(1);
            return;
        }
#endif
    }

#endif // RID_FAKE_TEST_MODE

    // -------------------------------------------------------------------------
    // Common Remote ID transmit section
    // Runs for both normal data and RID fake-test data.
    // -------------------------------------------------------------------------

    static uint32_t last_update_wifi_nan_ms;
    if (g.wifi_nan_rate > 0 &&
        now_ms - last_update_wifi_nan_ms > 1000/g.wifi_nan_rate) {
        last_update_wifi_nan_ms = now_ms;
        wifi.transmit_nan(UAS_data);
    }

    static uint32_t last_update_wifi_beacon_ms;
    if (g.wifi_beacon_rate > 0 &&
        now_ms - last_update_wifi_beacon_ms > 1000/g.wifi_beacon_rate) {
        last_update_wifi_beacon_ms = now_ms;
        wifi.transmit_beacon(UAS_data);
    }

    static uint32_t last_update_bt5_ms;
    if (g.bt5_rate > 0 &&
        now_ms - last_update_bt5_ms > 1000/g.bt5_rate) {
        last_update_bt5_ms = now_ms;
        ble.transmit_longrange(UAS_data);
    }

    static uint32_t last_update_bt4_ms;
    int bt4_states = UAS_data.BasicIDValid[1] ? 7 : 6;
    if (g.bt4_rate > 0 &&
        now_ms - last_update_bt4_ms > (1000.0f/bt4_states)/g.bt4_rate) {
        last_update_bt4_ms = now_ms;
        ble.transmit_legacy(UAS_data);
    }

    // sleep for a bit for power saving
    delay(1);
}
