#pragma once

#include <Arduino.h>
#include <opendroneid.h>

#if defined(BOARD_TDM2421)

#include "tdm2421_modem_rx.h"

struct TDM2421GNSSFix {
    bool valid;
    double latitude;
    double longitude;
    float altitude_m;
    float speed_mps;
    float course_deg;
    float hdop;
    float pdop;
    float vdop;
    uint8_t fix_type;
    uint8_t satellites;
    uint8_t satellites_used;
    bool course_valid;
    bool dop_valid;
    char utc[16];
    char date[8];
    uint32_t age_ms;
};

struct TDM2421Config {
    char apn[33];
    char mqtt_host[65];
    uint16_t mqtt_port;
    char mqtt_user[33];
    char mqtt_pass[65];
    char mqtt_client[65];
    char mqtt_topic[97];
    char device_id[33];
    // Optional physical UAV serial used in MQTT extension.uavSN.
    // Remote ID Basic ID itself is stored in the standard ArduRemoteID UAS_ID parameter.
    char uav_serial[21];
    char version[17];
    char bandwidth[17];
    uint32_t publish_interval_ms;
    uint32_t reconnect_interval_ms;
    bool c2_enable;
    char c2_tx_topic[97];
    char c2_rx_topic[97];
    uint16_t c2_batch_ms;
};

class TDM2421MQTT {
public:
    void begin();
    void submit(const ODID_UAS_Data &uas, uint8_t wifi_channel);
    void setMavLocationFresh(bool fresh) { mav_location_fresh_flag = fresh; }
    bool isMqttConnected() const { return mqtt_ok && raw_socket_ok; }
    bool isC2Subscribed() const { return c2_subscribed; }

    // Runtime configuration stored in ESP32 NVS namespace "tdmnet".
    void getConfig(TDM2421Config &out);
    bool saveConfig(const TDM2421Config &cfg);
    // Cập nhật chu kỳ publish MQTT tạm thời trong RAM (không lưu NVS, không reconnect/reboot).
    bool setRuntimePublishIntervalMs(uint32_t interval_ms);

    // Đọc tọa độ GNSS mới nhất từ modem EG800K. An toàn đa luồng.
    bool getGNSSFix(TDM2421GNSSFix &out, uint32_t max_age_ms = 5000);

    // JSON helpers for the local Web dashboard. Passwords are never returned.
    String configJson();
    String statusJson();

private:
    static void taskEntry(void *arg);
    void task();

    bool modemReady();
    bool ensureNetwork();
    bool ensureMQTT();
    bool rawTcpOpen();
    bool rawTcpSend(const uint8_t *data, size_t len, uint32_t timeout_ms = 1000);
    bool rawMqttConnect();
    bool rawMqttSubscribe(const char *topic, uint16_t packet_id);
    bool rawMqttPublish(const char *topic, const uint8_t *payload, size_t payload_len);
    void pumpModemRX();
    static void onMqttPublishEntry(const char *topic, size_t topic_len,
                                   const uint8_t *payload, size_t payload_len, void *ctx);
    void handleMqttPublish(const char *topic, size_t topic_len,
                           const uint8_t *payload, size_t payload_len);
    static size_t mqttEncodeRemainingLength(uint32_t value, uint8_t *out, size_t out_len);
    static size_t mqttWriteUtf8(uint8_t *out, size_t out_len, const char *s);
    bool syncTime();
    bool publishSnapshot();
    bool publishC2Downlink(const char *topic, const uint8_t *data, size_t len);
    void refreshSignal();
    void handleReconfigure();
    bool ensureGNSS();
    void refreshGNSS(bool skip_dop = false);
    bool parseQGPSLOC(const String &response, TDM2421GNSSFix &fix);
    void refreshGNSSDOP();
    bool parseGSA(const String &response, float &pdop, float &hdop, float &vdop, uint8_t &used);

    String sendAT(const String &cmd, uint32_t timeout_ms = 2000);
    bool waitFor(const String &needle, uint32_t timeout_ms, String *capture = nullptr);

    void loadConfig();
    void setError(const char *msg);
    void setError(const String &msg);
    void setNetworkState(bool value);
    void setMQTTState(bool value);
    static String jsonEscape(const String &s);

    uint64_t nowUnixMs() const;
    uint64_t parseQLTS(const String &s) const;
    static int64_t daysFromCivil(int y, unsigned m, unsigned d);
    static void wifiChannelString(uint8_t channel, char *out, size_t out_len);

    SemaphoreHandle_t snapshot_mutex = nullptr;
    ODID_UAS_Data snapshot {};
    uint8_t snapshot_wifi_channel = 6;
    bool have_snapshot = false;

    SemaphoreHandle_t config_mutex = nullptr;
    TDM2421Config config {};
    volatile bool reconfigure_requested = false;
    volatile bool mav_location_fresh_flag = false;
    volatile bool c2_subscribed = false;
    uint32_t c2_last_batch_ms = 0;

    TDM2421ModemRxDemux rx_demux {};

    SemaphoreHandle_t state_mutex = nullptr;
    bool gnss_started = false;
    uint32_t gnss_last_dop_ms = 0;
    bool sim_ready = false;
    bool registered = false;
    bool network_ok = false;
    bool mqtt_ok = false;
    bool time_ok = false;
    int rssi_dbm = 0;
    int signal_quality_csq = 99;  // AT+CSQ RSSI index: 0..31, 99 = unknown
    char lte_ip[32] {};
    char last_error[96] {};
    uint32_t last_publish_ok_ms = 0;

    uint64_t unix_base_ms = 0;
    uint32_t millis_base = 0;
    uint32_t msg_count = 0;
    uint16_t mqtt_msg_id = 1;
    bool raw_socket_ok = false;
    uint32_t raw_send_ok = 0;
    uint32_t raw_send_error = 0;
    uint32_t mqtt_rate_last_ms = 0;
    uint32_t mqtt_last_ping_ms = 0;
};

extern TDM2421MQTT tdm2421_mqtt;

#endif
