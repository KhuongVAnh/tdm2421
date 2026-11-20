#include "tdm2421_mqtt.h"

#if defined(BOARD_TDM2421)

#include "board_config.h"
#include "tdm2421_c2.h"
#include "tdm2421_gnss.h"
#include "tdm2421_debug.h"
#include "mavlink.h"
#include "parameters.h"
#include <Preferences.h>
#include <inttypes.h>

// -----------------------------------------------------------------------------
// Factory defaults. They are copied to NVS on first use and can then be changed
// from the TDM2421 local Web dashboard without rebuilding the firmware.
// -----------------------------------------------------------------------------
static const char *DEFAULT_APN          = "v-internet";
static const char *DEFAULT_MQTT_HOST    = "171.251.89.98";
static const uint16_t DEFAULT_MQTT_PORT = 8903;
static const char *DEFAULT_MQTT_USER    = "pilot";
static const char *DEFAULT_MQTT_PASS    = "pilot123";
static const char *DEFAULT_MQTT_CLIENT  = "P3000M_TDM2421";
static const char *DEFAULT_MQTT_TOPIC   = "p3000m/data";
static const char *DEFAULT_DEVICE_ID    = "TDM2421";
static const char *DEFAULT_VERSION      = "V1.0";
static const char *DEFAULT_BANDWIDTH    = "20MHz";
static const bool DEFAULT_C2_ENABLE     = true;
static const char *DEFAULT_C2_TX_TOPIC  = "p3000m/c2/1866ASDD0000VT5/tx";
static const char *DEFAULT_C2_RX_TOPIC  = "p3000m/c2/1866ASDD0000VT5/rx";
#if defined(TDM2421_C2_MAX_HZ) && TDM2421_C2_MAX_HZ > 0
static const uint16_t BUILD_C2_BATCH_MS =
    (uint16_t)((1000U + (TDM2421_C2_MAX_HZ / 2U)) / TDM2421_C2_MAX_HZ);
static const uint16_t DEFAULT_C2_BATCH_MS = BUILD_C2_BATCH_MS;
#else
static const uint16_t DEFAULT_C2_BATCH_MS = 33;
#endif
static const uint16_t MIN_C2_BATCH_MS     = 20;
static const uint16_t MAX_C2_BATCH_MS     = 500;
// Giới hạn payload mỗi gói PUBLISH C2 tối đa 1300 B để cộng header + topic vẫn <= 1460 B (giới hạn AT+QISEND=0).
static const size_t C2_BATCH_MAX_BYTES    = 1300;

// Build-time MQTT publish-rate override.
//   TDM2421_MQTT_HZ = 1..30 : force that rate, overriding NVS/Web setting.
//   TDM2421_MQTT_HZ = 0     : use publish_interval_ms from NVS/Web.
#ifndef TDM2421_MQTT_HZ
#define TDM2421_MQTT_HZ 10
#endif
#if TDM2421_MQTT_HZ < 0 || TDM2421_MQTT_HZ > 30
#error "TDM2421_MQTT_HZ must be 0 (NVS/Web) or 1..30 Hz"
#endif

#if TDM2421_MQTT_HZ > 0
static const uint32_t BUILD_PUBLISH_INTERVAL_MS =
    (1000U + (TDM2421_MQTT_HZ / 2U)) / TDM2421_MQTT_HZ;
static const uint32_t DEFAULT_PUBLISH_INTERVAL_MS = BUILD_PUBLISH_INTERVAL_MS;
#else
static const uint32_t DEFAULT_PUBLISH_INTERVAL_MS = 100;
#endif
static const uint32_t MIN_PUBLISH_INTERVAL_MS = 33;
static const uint32_t GNSS_REFRESH_INTERVAL_MS = 5000;
static const uint32_t SIGNAL_REFRESH_INTERVAL_MS = 10000;
static const uint32_t MQTT_PUBLISH_PROMPT_TIMEOUT_MS = 1000;
static const uint32_t MQTT_PUBLISH_ACCEPT_TIMEOUT_MS = 500;
static const uint32_t MQTT_URC_DRAIN_BUDGET_MS = 2;
static const uint32_t DEFAULT_RECONNECT_INTERVAL_MS = 5000;

TDM2421MQTT tdm2421_mqtt;

template <size_t N>
static void copyString(char (&dst)[N], const String &src)
{
    memset(dst, 0, N);
    src.substring(0, N - 1).toCharArray(dst, N);
}

template <size_t N>
static void copyCString(char (&dst)[N], const char *src)
{
    memset(dst, 0, N);
    if (src != nullptr) {
        strncpy(dst, src, N - 1);
    }
}

void TDM2421MQTT::begin()
{
    snapshot_mutex = xSemaphoreCreateMutex();
    config_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();

    loadConfig();
    xTaskCreate(taskEntry, "tdm_mqtt", 8192, this, 1, nullptr);
}

void TDM2421MQTT::loadConfig()
{
    TDM2421Config cfg {};
    copyCString(cfg.apn, DEFAULT_APN);
    copyCString(cfg.mqtt_host, DEFAULT_MQTT_HOST);
    cfg.mqtt_port = DEFAULT_MQTT_PORT;
    copyCString(cfg.mqtt_user, DEFAULT_MQTT_USER);
    copyCString(cfg.mqtt_pass, DEFAULT_MQTT_PASS);
    copyCString(cfg.mqtt_client, DEFAULT_MQTT_CLIENT);
    copyCString(cfg.mqtt_topic, DEFAULT_MQTT_TOPIC);
    copyCString(cfg.device_id, DEFAULT_DEVICE_ID);
    cfg.uav_serial[0] = '\0';
    copyCString(cfg.version, DEFAULT_VERSION);
    copyCString(cfg.bandwidth, DEFAULT_BANDWIDTH);
    cfg.publish_interval_ms = DEFAULT_PUBLISH_INTERVAL_MS;
    cfg.reconnect_interval_ms = DEFAULT_RECONNECT_INTERVAL_MS;
    cfg.c2_enable = DEFAULT_C2_ENABLE;
    const char *init_rid = (g.uas_id[0] != '\0') ? g.uas_id : DEFAULT_DEVICE_ID;
    snprintf(cfg.c2_tx_topic, sizeof(cfg.c2_tx_topic), "p3000m/c2/%s/tx", init_rid);
    snprintf(cfg.c2_rx_topic, sizeof(cfg.c2_rx_topic), "p3000m/c2/%s/rx", init_rid);
    cfg.c2_batch_ms = DEFAULT_C2_BATCH_MS;

    Preferences prefs;
    if (prefs.begin("tdmnet", false)) {
        copyString(cfg.apn, prefs.getString("apn", cfg.apn));
        copyString(cfg.mqtt_host, prefs.getString("host", cfg.mqtt_host));
        cfg.mqtt_port = prefs.getUShort("port", cfg.mqtt_port);
        copyString(cfg.mqtt_user, prefs.getString("user", cfg.mqtt_user));
        copyString(cfg.mqtt_pass, prefs.getString("pass", cfg.mqtt_pass));
        copyString(cfg.mqtt_client, prefs.getString("client", cfg.mqtt_client));
        copyString(cfg.mqtt_topic, prefs.getString("topic", cfg.mqtt_topic));
        copyString(cfg.device_id, prefs.getString("device", cfg.device_id));
        copyString(cfg.uav_serial, prefs.getString("uavserial", cfg.uav_serial));
        copyString(cfg.version, prefs.getString("version", cfg.version));
        copyString(cfg.bandwidth, prefs.getString("bandwidth", cfg.bandwidth));
        cfg.publish_interval_ms = prefs.getUInt("pub_ms", cfg.publish_interval_ms);
        cfg.reconnect_interval_ms = prefs.getUInt("recon_ms", cfg.reconnect_interval_ms);
        cfg.c2_enable = prefs.getBool("c2_en", cfg.c2_enable);
        copyString(cfg.c2_tx_topic, prefs.getString("c2_tx", cfg.c2_tx_topic));
        copyString(cfg.c2_rx_topic, prefs.getString("c2_rx", cfg.c2_rx_topic));
        cfg.c2_batch_ms = prefs.getUShort("c2_bms", cfg.c2_batch_ms);

        // Chuyển đổi topic C2 cũ ("p3000m/c2/tx", "p3000m/c2/rx") sang định dạng phân biệt theo Remote ID.
        const char *rid = (g.uas_id[0] != '\0') ? g.uas_id :
                          (cfg.device_id[0] != '\0' ? cfg.device_id : DEFAULT_DEVICE_ID);
        if (cfg.c2_tx_topic[0] == '\0' || strcmp(cfg.c2_tx_topic, "p3000m/c2/tx") == 0) {
            snprintf(cfg.c2_tx_topic, sizeof(cfg.c2_tx_topic), "p3000m/c2/%s/tx", rid);
        }
        if (cfg.c2_rx_topic[0] == '\0' || strcmp(cfg.c2_rx_topic, "p3000m/c2/rx") == 0) {
            snprintf(cfg.c2_rx_topic, sizeof(cfg.c2_rx_topic), "p3000m/c2/%s/rx", rid);
        }

        // Store defaults/current values so the namespace is fully initialised.
        prefs.putString("apn", cfg.apn);
        prefs.putString("host", cfg.mqtt_host);
        prefs.putUShort("port", cfg.mqtt_port);
        prefs.putString("user", cfg.mqtt_user);
        prefs.putString("pass", cfg.mqtt_pass);
        prefs.putString("client", cfg.mqtt_client);
        prefs.putString("topic", cfg.mqtt_topic);
        prefs.putString("device", cfg.device_id);
        prefs.putString("uavserial", cfg.uav_serial);
        prefs.putString("version", cfg.version);
        prefs.putString("bandwidth", cfg.bandwidth);
        prefs.putUInt("pub_ms", cfg.publish_interval_ms);
        prefs.putUInt("recon_ms", cfg.reconnect_interval_ms);
        prefs.putBool("c2_en", cfg.c2_enable);
        prefs.putString("c2_tx", cfg.c2_tx_topic);
        prefs.putString("c2_rx", cfg.c2_rx_topic);
        prefs.putUShort("c2_bms", cfg.c2_batch_ms);

        prefs.end();
    }

    if (cfg.mqtt_port == 0) cfg.mqtt_port = DEFAULT_MQTT_PORT;
#if TDM2421_MQTT_HZ > 0
    // The make-time setting deliberately wins over any previously saved NVS value.
    cfg.publish_interval_ms = BUILD_PUBLISH_INTERVAL_MS;
#else
    if (cfg.publish_interval_ms < MIN_PUBLISH_INTERVAL_MS)
        cfg.publish_interval_ms = DEFAULT_PUBLISH_INTERVAL_MS;
#endif
    if (cfg.reconnect_interval_ms < 1000) cfg.reconnect_interval_ms = DEFAULT_RECONNECT_INTERVAL_MS;
    if (cfg.c2_tx_topic[0] == '\0') copyCString(cfg.c2_tx_topic, DEFAULT_C2_TX_TOPIC);
    if (cfg.c2_rx_topic[0] == '\0') copyCString(cfg.c2_rx_topic, DEFAULT_C2_RX_TOPIC);
#if defined(TDM2421_C2_MAX_HZ) && TDM2421_C2_MAX_HZ > 0
    // Tần số C2 cấu hình tại build-time ghi đè giá trị c2_bms cũ trong NVS.
    cfg.c2_batch_ms = BUILD_C2_BATCH_MS;
#else
    if (cfg.c2_batch_ms < MIN_C2_BATCH_MS || cfg.c2_batch_ms > MAX_C2_BATCH_MS) {
        cfg.c2_batch_ms = DEFAULT_C2_BATCH_MS;
    }
#endif
    tdm2421_c2.setEnabled(cfg.c2_enable);

#if TDM2421_MQTT_HZ > 0
    tdmDebugPrintf(1, "[MQTT CONFIG] build rate=%u Hz interval=%lu ms (overrides NVS/Web)\r\n",
                  (unsigned)TDM2421_MQTT_HZ,
                  (unsigned long)cfg.publish_interval_ms);
#else
    tdmDebugPrintf(1, "[MQTT CONFIG] NVS/Web rate control interval=%lu ms\r\n",
                  (unsigned long)cfg.publish_interval_ms);
#endif
#if defined(TDM2421_C2_MAX_HZ) && TDM2421_C2_MAX_HZ > 0
    tdmDebugPrintf(1, "[C2 CONFIG] RID=%s C2_MAX_HZ=%u batch=%u ms (overrides NVS/Web)\r\n",
                  TDM2421_ENABLE_RID ? "ON" : "OFF",
                  (unsigned)TDM2421_C2_MAX_HZ,
                  (unsigned)cfg.c2_batch_ms);
#endif

    if (config_mutex && xSemaphoreTake(config_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        memcpy(&config, &cfg, sizeof(config));
        xSemaphoreGive(config_mutex);
    } else {
        memcpy(&config, &cfg, sizeof(config));
    }
}

void TDM2421MQTT::getConfig(TDM2421Config &out)
{
    memset(&out, 0, sizeof(out));
    if (config_mutex && xSemaphoreTake(config_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        memcpy(&out, &config, sizeof(out));
        xSemaphoreGive(config_mutex);
    } else {
        memcpy(&out, &config, sizeof(out));
    }
}

bool TDM2421MQTT::saveConfig(const TDM2421Config &cfg)
{
    TDM2421Config effective = cfg;
#if TDM2421_MQTT_HZ > 0
    // Keep the make-time publish rate authoritative even if the Web form posts pub_ms.
    effective.publish_interval_ms = BUILD_PUBLISH_INTERVAL_MS;
#endif
#if defined(TDM2421_C2_MAX_HZ) && TDM2421_C2_MAX_HZ > 0
    effective.c2_batch_ms = BUILD_C2_BATCH_MS;
#endif

    if (effective.apn[0] == '\0' || effective.mqtt_host[0] == '\0' || effective.mqtt_port == 0 ||
        effective.mqtt_client[0] == '\0' || effective.mqtt_topic[0] == '\0' ||
        effective.device_id[0] == '\0' || effective.publish_interval_ms < MIN_PUBLISH_INTERVAL_MS ||
        effective.publish_interval_ms > 60000 || effective.reconnect_interval_ms < 1000 ||
        effective.reconnect_interval_ms > 300000) {
        return false;
    }
    if (effective.c2_enable) {
        if (effective.c2_tx_topic[0] == '\0' || effective.c2_rx_topic[0] == '\0' ||
            strcmp(effective.c2_tx_topic, effective.c2_rx_topic) == 0 ||
            effective.c2_batch_ms < MIN_C2_BATCH_MS || effective.c2_batch_ms > MAX_C2_BATCH_MS) {
            return false;
        }
    }

    Preferences prefs;
    if (!prefs.begin("tdmnet", false)) {
        return false;
    }
    prefs.putString("apn", effective.apn);
    prefs.putString("host", effective.mqtt_host);
    prefs.putUShort("port", effective.mqtt_port);
    prefs.putString("user", effective.mqtt_user);
    prefs.putString("pass", effective.mqtt_pass);
    prefs.putString("client", effective.mqtt_client);
    prefs.putString("topic", effective.mqtt_topic);
    prefs.putString("device", effective.device_id);
    prefs.putString("uavserial", effective.uav_serial);
    prefs.putString("version", effective.version);
    prefs.putString("bandwidth", effective.bandwidth);
    prefs.putUInt("pub_ms", effective.publish_interval_ms);
    prefs.putUInt("recon_ms", effective.reconnect_interval_ms);
    prefs.putBool("c2_en", effective.c2_enable);
    prefs.putString("c2_tx", effective.c2_tx_topic);
    prefs.putString("c2_rx", effective.c2_rx_topic);
    prefs.putUShort("c2_bms", effective.c2_batch_ms);
    prefs.end();

    if (config_mutex && xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&config, &effective, sizeof(config));
        xSemaphoreGive(config_mutex);
    } else {
        return false;
    }

    tdm2421_c2.setEnabled(effective.c2_enable);
    reconfigure_requested = true;
    return true;
}

bool TDM2421MQTT::setRuntimePublishIntervalMs(uint32_t interval_ms)
{
    // Chỉ cập nhật chu kỳ publish trong RAM để áp dụng tức thì mà không ghi NVS
    // và không ngắt kết nối LTE/MQTT; lần khởi động tiếp theo sẽ tự quay về mặc định.
    if (interval_ms < MIN_PUBLISH_INTERVAL_MS || interval_ms > 60000U) {
        return false;
    }
    if (config_mutex && xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        config.publish_interval_ms = interval_ms;
        xSemaphoreGive(config_mutex);
        return true;
    }
    return false;
}

void TDM2421MQTT::submit(const ODID_UAS_Data &uas, uint8_t wifi_channel)
{
    // UAS_data is the single source of truth shared by Broadcast RID and MQTT.

    if (snapshot_mutex == nullptr || !uas.LocationValid) {
        return;
    }
    if (xSemaphoreTake(snapshot_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(&snapshot, &uas, sizeof(snapshot));
        snapshot_wifi_channel = wifi_channel;
        have_snapshot = true;
        xSemaphoreGive(snapshot_mutex);
    }
}

void TDM2421MQTT::taskEntry(void *arg)
{
    static_cast<TDM2421MQTT *>(arg)->task();
}

void TDM2421MQTT::onMqttPublishEntry(const char *topic, size_t topic_len,
                                       const uint8_t *payload, size_t payload_len, void *ctx)
{
    if (ctx != nullptr) {
        static_cast<TDM2421MQTT *>(ctx)->handleMqttPublish(topic, topic_len, payload, payload_len);
    }
}

void TDM2421MQTT::handleMqttPublish(const char *topic, size_t topic_len,
                                    const uint8_t *payload, size_t payload_len)
{
    TDM2421Config cfg {};
    getConfig(cfg);
    if (!cfg.c2_enable || topic == nullptr || payload == nullptr || payload_len == 0) {
        return;
    }
    const size_t expected_len = strlen(cfg.c2_rx_topic);
    if (topic_len != expected_len || memcmp(topic, cfg.c2_rx_topic, expected_len) != 0) {
        return;
    }

    // Giải mã từng byte payload nhị phân qua parser MAVLink độc lập và kiểm tra CRC X.25.
    // Chỉ frame hợp lệ mới được đưa vào hàng đợi uplink để loop() ghi xuống FC (Serial1).
    uint8_t frame_buf[TDM2421_C2_MAX_FRAME_LEN] {};
    uint16_t frame_len = 0;
    uint16_t accepted = 0;
    for (size_t i = 0; i < payload_len; i++) {
        bool bad_crc = false;
        if (mavlink_parse_c2_uplink_byte(payload[i], frame_buf, frame_len, bad_crc)) {
            if (tdm2421_c2.pushUplinkFrame(frame_buf, frame_len, millis())) {
                accepted++;
            }
        } else if (bad_crc) {
            tdm2421_c2.recordRxInvalid();
        }
    }
    if (accepted > 0) {
        tdmDebugPrintf(2, "[C2 RX] accepted=%u frames bytes=%u\r\n",
                      (unsigned)accepted, (unsigned)payload_len);
    }
}

void TDM2421MQTT::pumpModemRX()
{
    // Duy nhất hàm này đọc Serial0: vừa hút URC nhị phân +QIURC: "recv" vào mqtt_ring,
    // vừa giữ lại phản hồi text AT thuần cho sendAT()/waitFor() mà không bao giờ làm rớt lệnh C2.
    const uint32_t now_ms = millis();
    while (Serial0.available()) {
        const uint8_t b = (uint8_t)Serial0.read();
        rx_demux.feedByte(b, now_ms);
    }
    rx_demux.checkTimeout(now_ms, 1000);

    if (rx_demux.consumeSocketClosed()) {
        raw_socket_ok = false;
        c2_subscribed = false;
        setMQTTState(false);
        setError("Raw MQTT TCP socket closed");
        tdmDebugPrintln(1, "[RAW MQTT] socket closed");
    }
    if (rx_demux.consumeProtocolError()) {
        raw_socket_ok = false;
        c2_subscribed = false;
        setMQTTState(false);
        setError("Modem RX stream error");
        tdmDebugPrintln(1, "[RAW MQTT] modem RX stream desync/error");
    }

    rx_demux.processMqttPackets(onMqttPublishEntry, this);
    if (rx_demux.takePingresp()) {
        tdmDebugPrintln(2, "[RAW MQTT] PINGRESP");
    }
}

String TDM2421MQTT::sendAT(const String &cmd, uint32_t timeout_ms)
{
    // Hút sạch byte tồn đọng qua demuxer trước khi xoá phần text AT cũ
    // (tuyệt đối không xoá mqtt_ring để tránh mất gói lệnh C2 đang chờ).
    pumpModemRX();
    rx_demux.clearATText();
    tdmDebugPrintf(2, "[EG800K] >> %s\r\n", cmd.c_str());
    Serial0.println(cmd);

    const uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        const size_t before = rx_demux.atLength();
        pumpModemRX();
        const bool received = (rx_demux.atLength() != before);

        if (rx_demux.atContains("\r\nOK\r\n") ||
            rx_demux.atContains("\nOK\r") ||
            rx_demux.atContains("\r\nERROR\r\n") ||
            rx_demux.atContains("+CME ERROR:")) {
            break;
        }

        if (!received) delay(1);
    }
    String out(rx_demux.atText());
    if (out.length()) {
        tdmDebugPrintf(2, "[EG800K] << %s\r\n", out.c_str());
    }
    return out;
}

bool TDM2421MQTT::waitFor(const String &needle, uint32_t timeout_ms, String *capture)
{
    const uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        pumpModemRX();
        if (rx_demux.atContains(needle.c_str())) {
            if (capture) *capture = String(rx_demux.atText());
            tdmDebugPrintf(2, "[EG800K] << %s\r\n", rx_demux.atText());
            return true;
        }
        if (rx_demux.atContains("ERROR")) {
            if (capture) *capture = String(rx_demux.atText());
            tdmDebugPrintf(2, "[EG800K] << %s\r\n", rx_demux.atText());
            return false;
        }
        delay(1);
    }
    if (capture) *capture = String(rx_demux.atText());
    if (rx_demux.atLength() > 0) tdmDebugPrintf(2, "[EG800K] << %s\r\n", rx_demux.atText());
    return false;
}

void TDM2421MQTT::setError(const char *msg)
{
    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        copyCString(last_error, msg ? msg : "");
        xSemaphoreGive(state_mutex);
    }
}

void TDM2421MQTT::setError(const String &msg)
{
    setError(msg.c_str());
}

void TDM2421MQTT::setNetworkState(bool value)
{
    network_ok = value;
    if (!value) {
        mqtt_ok = false;
        c2_subscribed = false;
    }
}

void TDM2421MQTT::setMQTTState(bool value)
{
    mqtt_ok = value;
    if (!value) {
        c2_subscribed = false;
    }
}

bool TDM2421MQTT::getGNSSFix(TDM2421GNSSFix &out, uint32_t max_age_ms)
{
    memset(&out, 0, sizeof(out));
    TDM2421GnssRecord rec {};
    if (!tdm2421_gnss.getEg800k(rec, max_age_ms, millis())) {
        return false;
    }
    out.valid = rec.valid;
    out.latitude = rec.latitude;
    out.longitude = rec.longitude;
    out.altitude_m = rec.altitude_msl_m;
    out.speed_mps = rec.speed_mps;
    out.course_deg = rec.course_deg;
    out.hdop = rec.hdop;
    out.pdop = rec.pdop;
    out.vdop = rec.vdop;
    out.fix_type = rec.fix_type;
    out.satellites = rec.satellites;
    out.satellites_used = rec.satellites_used;
    out.course_valid = rec.course_valid;
    out.dop_valid = rec.dop_valid;
    snprintf(out.utc, sizeof(out.utc), "%s", rec.utc);
    snprintf(out.date, sizeof(out.date), "%s", rec.date);
    out.age_ms = rec.age_ms;
    return true;
}

bool TDM2421MQTT::ensureGNSS()
{
    // First verify whether GNSS is already running.
    String r = sendAT("AT+QGPS?", 1000);
    if (r.indexOf("+QGPS: 1") >= 0) {
        gnss_started = true;
        // Best effort: enable NMEA acquisition so QGPSGNMEA="GSA" can provide PDOP/HDOP/VDOP.
        (void)sendAT("AT+QGPSCFG=\"nmeasrc\",1", 1000);
        tdmDebugPrintln(1, "[GNSS] EG800K GNSS already running");
        return true;
    }

    // Put the modem GNSS engine into a known state. QGPSEND may legitimately
    // return ERROR if no GNSS session exists, so that response is not fatal.
    tdmDebugPrintln(1, "[GNSS] resetting EG800K GNSS engine");
    (void)sendAT("AT+QGPSEND", 1200);
    delay(250);

    r = sendAT("AT+QGPS=1", 2000);
    if (r.indexOf("OK") < 0) {
        tdmDebugPrintln(1, "[GNSS] AT+QGPS=1 did not return OK; verifying state");
    }

    // Never trust only the start command response: verify the actual state.
    delay(250);
    r = sendAT("AT+QGPS?", 1000);
    gnss_started = (r.indexOf("+QGPS: 1") >= 0);
    if (gnss_started) {
        (void)sendAT("AT+QGPSCFG=\"nmeasrc\",1", 1000);
        tdmDebugPrintln(1, "[GNSS] EG800K GNSS started and verified (+QGPS: 1)");
    } else {
        tdmDebugPrintln(1, "[GNSS] EG800K GNSS unavailable/not started (+QGPS: 1 not seen)");
    }
    return gnss_started;
}

bool TDM2421MQTT::parseQGPSLOC(const String &response, TDM2421GNSSFix &fix)
{
    const int p = response.indexOf("+QGPSLOC:");
    if (p < 0) return false;

    int e = response.indexOf('\n', p);
    if (e < 0) e = response.length();
    String line = response.substring(p + 9, e);
    line.trim();

    // EG800K QGPSLOC=2 response observed on the TDM2421:
    // UTC,latitude,longitude,HDOP,altitude,fix,COG,spkm,spkn,date,nsat
    // Example:
    // 030336.00,21.06777,105.81160,1.54,-10.1,3,,0.046,0.025,150926,22
    //
    // IMPORTANT: COG may be empty (",,"). strtok()/strtok_r() skips empty
    // fields, shifting all following columns. Split manually and preserve
    // empty CSV fields so latitude/longitude/fix/speed/nsat stay aligned.
    String fields[11];
    size_t nf = 0;
    int field_start = 0;
    while (nf < 11) {
        const int comma = line.indexOf(',', field_start);
        if (comma < 0) {
            fields[nf++] = line.substring(field_start);
            break;
        }
        fields[nf++] = line.substring(field_start, comma);
        field_start = comma + 1;
    }
    if (nf != 11) {
        return false;
    }
    for (size_t i = 0; i < nf; i++) {
        fields[i].trim();
    }

    // Mandatory fields for a usable position.
    if (fields[1].length() == 0 || fields[2].length() == 0 ||
        fields[5].length() == 0) {
        return false;
    }

    const double lat = fields[1].toDouble();
    const double lon = fields[2].toDouble();
    const float hdop = fields[3].length() ? fields[3].toFloat() : 99.9f;
    const float alt = fields[4].length() ? fields[4].toFloat() : 0.0f;
    const int fix_type = fields[5].toInt();

    // COG can legitimately be empty while stationary. Use 0 deg until the
    // receiver reports a course. QGPSLOC=2 reports speed in km/h and knots;
    // use the km/h field and convert to m/s for OpenDroneID.
    const float course = fields[6].length() ? fields[6].toFloat() : 0.0f;
    const float speed_kmh = fields[7].length() ? fields[7].toFloat() : 0.0f;
    const int sats = fields[10].length() ? fields[10].toInt() : 0;

    if ((fix_type != 2 && fix_type != 3) ||
        lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 ||
        (lat == 0.0 && lon == 0.0)) {
        return false;
    }

    memset(&fix, 0, sizeof(fix));
    fix.valid = true;
    fix.latitude = lat;
    fix.longitude = lon;
    fix.altitude_m = alt;
    fix.speed_mps = speed_kmh / 3.6f;
    fix.course_deg = course;
    fix.hdop = hdop;
    fix.pdop = 0.0f;
    fix.vdop = 0.0f;
    fix.fix_type = (uint8_t)fix_type;
    fix.satellites = (uint8_t)constrain(sats, 0, 255);
    fix.satellites_used = 0;
    fix.course_valid = fields[6].length() > 0;
    fix.dop_valid = false;
    snprintf(fix.utc, sizeof(fix.utc), "%s", fields[0].c_str());
    snprintf(fix.date, sizeof(fix.date), "%s", fields[9].c_str());
    return true;
}

bool TDM2421MQTT::parseGSA(const String &response, float &pdop, float &hdop, float &vdop, uint8_t &used)
{
    // NMEA GSA: $xxGSA,<M/A>,<1/2/3>,12 satellite slots,<PDOP>,<HDOP>,<VDOP>[,<systemId>]*CS
    int pos = 0;
    while (true) {
        int start = response.indexOf('$', pos);
        if (start < 0) return false;
        int end = response.indexOf('\n', start);
        if (end < 0) end = response.length();
        String line = response.substring(start, end); line.trim();
        pos = end + 1;
        if (line.length() < 7 || line.substring(3,6) != "GSA") continue;
        int star = line.indexOf('*'); if (star > 0) line = line.substring(0, star);
        String f[20]; int nf=0, st=0;
        while (nf < 20) { int cm=line.indexOf(',',st); if(cm<0){f[nf++]=line.substring(st);break;} f[nf++]=line.substring(st,cm); st=cm+1; }
        if (nf < 18) continue;
        int fixmode=f[2].toInt(); if (fixmode < 2) continue;
        uint8_t n=0; for(int i=3;i<=14 && i<nf;i++) if(f[i].length()) n++;
        if (!f[15].length() || !f[16].length() || !f[17].length()) continue;
        pdop=f[15].toFloat(); hdop=f[16].toFloat(); vdop=f[17].toFloat(); used=n; return true;
    }
}

void TDM2421MQTT::refreshGNSSDOP()
{
    // EG800K supports acquisition of NMEA sentences through QGPSGNMEA. GSA carries PDOP/HDOP/VDOP.
    String r = sendAT("AT+QGPSGNMEA=\"GSA\"", 1000);
    float p = 0, h = 0, v = 0;
    uint8_t used = 0;
    if (!parseGSA(r, p, h, v, used)) return;
    const uint32_t now_ms = millis();
    gnss_last_dop_ms = now_ms;
    tdm2421_gnss.updateEg800kDop(p, h, v, used, now_ms);
}

void TDM2421MQTT::refreshGNSS(bool skip_dop)
{
    if (!gnss_started && !ensureGNSS()) return;

    const String r = sendAT("AT+QGPSLOC=2", 900);
    TDM2421GNSSFix fix {};
    if (!parseQGPSLOC(r, fix)) {
        // Quectel CME 516 means GNSS is running but there is currently no fix.
        // This is a normal acquisition state, not a modem/network failure.
        if (r.indexOf("+CME ERROR: 516") >= 0) {
            static uint32_t last_no_fix_print = 0;
            if (millis() - last_no_fix_print >= 5000) {
                last_no_fix_print = millis();
                tdmDebugPrintln(1, "[GNSS] NO FIX (CME 516) - waiting for satellites");
            }
        } else {
            static uint32_t last_parse_print = 0;
            if (millis() - last_parse_print >= 5000) {
                last_parse_print = millis();
                tdmDebugPrintln(1, "[GNSS] QGPSLOC response has no valid fix");
            }
        }
        return;
    }

    const uint32_t now_ms = millis();
    TDM2421GnssRecord rec {};
    rec.valid = true;
    rec.latitude = fix.latitude;
    rec.longitude = fix.longitude;
    rec.altitude_msl_m = fix.altitude_m;
    rec.height_agl_m = fix.altitude_m;
    rec.height_valid = true;
    rec.speed_mps = fix.speed_mps;
    rec.speed_vertical_mps = 0.0f;
    rec.course_deg = fix.course_deg;
    rec.course_valid = fix.course_valid;
    rec.hdop = fix.hdop;
    rec.fix_type = fix.fix_type;
    rec.satellites = fix.satellites;
    rec.odid_status = ODID_STATUS_AIRBORNE;
    rec.odid_timestamp_s = 0.0f;
    snprintf(rec.utc, sizeof(rec.utc), "%s", fix.utc);
    snprintf(rec.date, sizeof(rec.date), "%s", fix.date);
    snprintf(rec.sub_source, sizeof(rec.sub_source), "EG800K");
    rec.updated_ms = now_ms;
    tdm2421_gnss.updateEg800k(rec);

    if (!skip_dop && (uint32_t)(now_ms - gnss_last_dop_ms) >= 10000U) {
        refreshGNSSDOP();
    }

    static uint32_t last_print = 0;
    if (now_ms - last_print >= 5000) {
        last_print = now_ms;
        tdmDebugPrintf(2, "[GNSS] fix=%u sats=%u lat=%.7f lon=%.7f alt=%.1f speed=%.2f course=%.1f hdop=%.1f\r\n",
                      fix.fix_type, fix.satellites, fix.latitude, fix.longitude,
                      fix.altitude_m, fix.speed_mps, fix.course_deg, fix.hdop);
    }
}

bool TDM2421MQTT::modemReady()
{
    for (uint8_t i = 0; i < 20; i++) {
        String r = sendAT("AT", 500);
        if (r.indexOf("OK") >= 0) {
            sendAT("ATE0", 500);
            setError("");
            return true;
        }
        delay(500);
    }
    setError("EG800K not responding");
    return false;
}

bool TDM2421MQTT::ensureNetwork()
{
    TDM2421Config cfg {};
    getConfig(cfg);

    if (network_ok) {
        String r = sendAT("AT+QIACT?", 1000);
        if (r.indexOf("+QIACT: 1") >= 0) {
            return true;
        }
        setNetworkState(false);
    }

    String r = sendAT("AT+CPIN?", 1000);
    sim_ready = (r.indexOf("+CPIN: READY") >= 0);
    if (!sim_ready) {
        setError("SIM not ready");
        return false;
    }

    registered = false;
    for (uint8_t i = 0; i < 30; i++) {
        r = sendAT("AT+CEREG?", 700);
        if (r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0) {
            registered = true;
            break;
        }
        delay(500);
    }
    if (!registered) {
        setError("LTE registration failed");
        return false;
    }

    String cmd = "AT+QICSGP=1,1,\"" + String(cfg.apn) + "\",\"\",\"\",1";
    sendAT(cmd, 1500);

    // If context is already active QIACT can return ERROR, so query afterwards.
    sendAT("AT+QIACT=1", 10000);
    r = sendAT("AT+QIACT?", 1500);
    if (r.indexOf("+QIACT: 1") < 0) {
        setError("PDP activation failed");
        return false;
    }

    // Capture the assigned IP address when present.
    const int p = r.indexOf("+QIACT: 1");
    const int q1 = r.indexOf('"', p);
    const int q2 = (q1 >= 0) ? r.indexOf('"', q1 + 1) : -1;
    if (q1 >= 0 && q2 > q1) {
        String ip = r.substring(q1 + 1, q2);
        if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            copyString(lte_ip, ip);
            xSemaphoreGive(state_mutex);
        }
    }

    setNetworkState(true);
    setError("");
    tdmDebugPrintln(1, "[MQTT] LTE/PDP ready");
    return true;
}

bool TDM2421MQTT::rawTcpOpen()
{
    TDM2421Config cfg {};
    getConfig(cfg);

    // Socket 0 is dedicated to raw MQTT over TCP. Direct-push mode (1) keeps
    // the UART in AT command mode, so GNSS/CSQ can still share Serial0.
    sendAT("AT+QICLOSE=0", 800);
    rx_demux.resetMqttStream();
    rx_demux.clearATText();

    String cmd = "AT+QIOPEN=1,0,\"TCP\",\"" + String(cfg.mqtt_host) +
                 "\"," + String(cfg.mqtt_port) + ",0,1";
    tdmDebugPrintf(2, "[RAW MQTT] >> %s\r\n", cmd.c_str());
    Serial0.println(cmd);

    if (!waitFor("+QIOPEN: 0,0", 30000)) {
        raw_socket_ok = false;
        setError("Raw MQTT TCP open failed");
        return false;
    }

    raw_socket_ok = true;
    tdmDebugPrintln(1, "[RAW MQTT] TCP connected");
    return true;
}

size_t TDM2421MQTT::mqttEncodeRemainingLength(uint32_t value, uint8_t *out, size_t out_len)
{
    size_t n = 0;
    do {
        if (n >= out_len) return 0;
        uint8_t byte = value % 128U;
        value /= 128U;
        if (value > 0) byte |= 0x80U;
        out[n++] = byte;
    } while (value > 0);
    return n;
}

size_t TDM2421MQTT::mqttWriteUtf8(uint8_t *out, size_t out_len, const char *str)
{
    if (!str) str = "";
    const size_t len = strlen(str);
    if (len > 65535U || out_len < len + 2U) return 0;
    out[0] = (uint8_t)((len >> 8) & 0xFFU);
    out[1] = (uint8_t)(len & 0xFFU);
    memcpy(out + 2, str, len);
    return len + 2U;
}

bool TDM2421MQTT::rawTcpSend(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (!raw_socket_ok || data == nullptr || len == 0 || len > 1460U) return false;

    // Hút sạch URC đang chờ vào mqtt_ring rồi chỉ xoá phần text AT cũ trước QISEND.
    pumpModemRX();
    if (!raw_socket_ok) return false;
    rx_demux.clearATText();

    String cmd = "AT+QISEND=0," + String((unsigned int)len);
    Serial0.println(cmd);
    if (!waitFor(">", 500)) {
        raw_send_error++;
        raw_socket_ok = false;
        setMQTTState(false);
        setError("Raw MQTT QISEND prompt timeout");
        return false;
    }

    // Xoá ">" trước khi chờ "SEND OK" và gửi payload nhị phân.
    rx_demux.clearATText();
    Serial0.write(data, len);

    // SEND OK means the modem accepted the bytes for the TCP peer. This is
    // normally much lighter than running the modem MQTT parser per message.
    if (!waitFor("SEND OK", timeout_ms)) {
        raw_send_error++;
        raw_socket_ok = false;
        setMQTTState(false);
        setError("Raw MQTT QISEND failed");
        return false;
    }

    raw_send_ok++;
    return true;
}

bool TDM2421MQTT::rawMqttConnect()
{
    TDM2421Config cfg {};
    getConfig(cfg);

    uint8_t body[256];
    size_t pos = 0;

    // MQTT 3.1.1 variable header.
    const uint8_t vh[] = {0x00,0x04,'M','Q','T','T',0x04,0x00,0x00,0x3C};
    memcpy(body + pos, vh, sizeof(vh));
    pos += sizeof(vh);

    uint8_t flags = 0x02; // Clean Session
    if (cfg.mqtt_user[0]) flags |= 0x80;
    if (cfg.mqtt_pass[0]) flags |= 0x40;
    body[7] = flags;

    size_t n = mqttWriteUtf8(body + pos, sizeof(body) - pos, cfg.mqtt_client);
    if (!n) return false;
    pos += n;
    if (cfg.mqtt_user[0]) {
        n = mqttWriteUtf8(body + pos, sizeof(body) - pos, cfg.mqtt_user);
        if (!n) return false;
        pos += n;
    }
    if (cfg.mqtt_pass[0]) {
        n = mqttWriteUtf8(body + pos, sizeof(body) - pos, cfg.mqtt_pass);
        if (!n) return false;
        pos += n;
    }

    uint8_t packet[272];
    packet[0] = 0x10; // CONNECT
    const size_t rl = mqttEncodeRemainingLength((uint32_t)pos, packet + 1, 4);
    if (!rl || 1 + rl + pos > sizeof(packet)) return false;
    memcpy(packet + 1 + rl, body, pos);

    if (!rawTcpSend(packet, 1 + rl + pos, 1500)) return false;

    // Đọc gói CONNACK thông qua bộ tách nhị phân + MQTT parser thay vì quét trực tiếp Serial0.
    const uint32_t start = millis();
    while ((uint32_t)(millis() - start) < 5000U) {
        pumpModemRX();
        if (!raw_socket_ok) break;
        uint8_t rc = 0xFF;
        if (rx_demux.takeConnack(rc)) {
            if (rc == 0x00) {
                tdmDebugPrintln(1, "[RAW MQTT] CONNACK accepted");
                return true;
            }
            tdmDebugPrintf(1, "[RAW MQTT] CONNACK rejected rc=0x%02X\r\n", (unsigned)rc);
            return false;
        }
        delay(2);
    }

    tdmDebugPrintln(1, "[RAW MQTT] CONNACK timeout");
    return false;
}

bool TDM2421MQTT::rawMqttSubscribe(const char *topic, uint16_t packet_id)
{
    if (!topic || topic[0] == '\0') return false;
    const size_t topic_len = strlen(topic);
    // Variable header (2B packet_id) + Payload (2B topic_len + topic + 1B requested QoS0).
    const uint32_t remaining = (uint32_t)(2U + 2U + topic_len + 1U);

    uint8_t packet[160];
    size_t pos = 0;
    packet[pos++] = 0x82; // SUBSCRIBE (flags = 0010b theo MQTT 3.1.1)
    uint8_t rl[4];
    const size_t rl_len = mqttEncodeRemainingLength(remaining, rl, sizeof(rl));
    if (!rl_len || pos + rl_len + remaining > sizeof(packet)) return false;
    memcpy(packet + pos, rl, rl_len);
    pos += rl_len;

    packet[pos++] = (uint8_t)((packet_id >> 8) & 0xFFU);
    packet[pos++] = (uint8_t)(packet_id & 0xFFU);
    packet[pos++] = (uint8_t)((topic_len >> 8) & 0xFFU);
    packet[pos++] = (uint8_t)(topic_len & 0xFFU);
    memcpy(packet + pos, topic, topic_len);
    pos += topic_len;
    packet[pos++] = 0x00; // QoS 0

    if (!rawTcpSend(packet, pos, 1500)) return false;

    const uint32_t start = millis();
    while ((uint32_t)(millis() - start) < 3000U) {
        pumpModemRX();
        if (!raw_socket_ok) break;
        uint16_t ack_pid = 0;
        uint8_t rc = 0x80;
        if (rx_demux.takeSuback(ack_pid, rc)) {
            if (ack_pid == packet_id && rc != 0x80U) {
                tdmDebugPrintf(1, "[C2 MQTT] SUBACK ok topic='%s' qos=%u\r\n", topic, (unsigned)rc);
                return true;
            }
            tdmDebugPrintf(1, "[C2 MQTT] SUBACK rejected pid=%u rc=0x%02X\r\n",
                          (unsigned)ack_pid, (unsigned)rc);
            return false;
        }
        delay(2);
    }
    tdmDebugPrintln(1, "[C2 MQTT] SUBACK timeout");
    return false;
}

bool TDM2421MQTT::rawMqttPublish(const char *topic, const uint8_t *payload, size_t payload_len)
{
    if (!topic || !payload) return false;
    const size_t topic_len = strlen(topic);
    const uint32_t remaining = (uint32_t)(2U + topic_len + payload_len);

    uint8_t packet[1460];
    size_t pos = 0;
    packet[pos++] = 0x30; // PUBLISH, QoS0, retain=0
    uint8_t rl[4];
    const size_t rl_len = mqttEncodeRemainingLength(remaining, rl, sizeof(rl));
    if (!rl_len || pos + rl_len + remaining > sizeof(packet)) return false;
    memcpy(packet + pos, rl, rl_len);
    pos += rl_len;
    packet[pos++] = (uint8_t)((topic_len >> 8) & 0xFFU);
    packet[pos++] = (uint8_t)(topic_len & 0xFFU);
    memcpy(packet + pos, topic, topic_len);
    pos += topic_len;
    memcpy(packet + pos, payload, payload_len);
    pos += payload_len;

    if (!rawTcpSend(packet, pos, 500)) return false;
    mqtt_last_ping_ms = millis();
    return true;
}

bool TDM2421MQTT::publishC2Downlink(const char *topic, const uint8_t *data, size_t len)
{
    if (!rawMqttPublish(topic, data, len)) {
        setMQTTState(false);
        setError("C2 MQTT publish failed");
        return false;
    }
    return true;
}

bool TDM2421MQTT::ensureMQTT()
{
    if (mqtt_ok && raw_socket_ok) return true;

    TDM2421Config cfg {};
    getConfig(cfg);

    // Xoá dữ liệu tồn đọng cũ từ lúc mất mạng để tránh xả backlog lỗi thời ngay khi nối lại.
    tdm2421_c2.clearDownlinkBacklog();
    tdm2421_c2.clearUplinkQueue();
    c2_subscribed = false;

    // Stop the modem's internal MQTT client so only the raw TCP implementation
    // owns this broker connection.
    sendAT("AT+QMTDISC=0", 500);
    sendAT("AT+QMTCLOSE=0", 500);

    if (!rawTcpOpen()) return false;
    if (!rawMqttConnect()) {
        sendAT("AT+QICLOSE=0", 800);
        raw_socket_ok = false;
        setError("Raw MQTT CONNECT failed");
        return false;
    }

    if (cfg.c2_enable && cfg.c2_rx_topic[0] != '\0') {
        if (++mqtt_msg_id == 0) mqtt_msg_id = 1;
        if (!rawMqttSubscribe(cfg.c2_rx_topic, mqtt_msg_id)) {
            sendAT("AT+QICLOSE=0", 800);
            raw_socket_ok = false;
            setError("C2 MQTT SUBSCRIBE failed");
            return false;
        }
        c2_subscribed = true;
    }

    setMQTTState(true);
    mqtt_last_ping_ms = millis();
    c2_last_batch_ms = millis();
    setError("");
    tdmDebugPrintln(1, "[RAW MQTT] connected");
    return true;
}

int64_t TDM2421MQTT::daysFromCivil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int)doe - 719468;
}

uint64_t TDM2421MQTT::parseQLTS(const String &s) const
{
    const int p = s.indexOf("+QLTS:");
    if (p < 0) return 0;
    const int q1 = s.indexOf('"', p);
    const int q2 = s.indexOf('"', q1 + 1);
    if (q1 < 0 || q2 < 0) return 0;

    String t = s.substring(q1 + 1, q2);
    int year, mon, day, hour, minute, sec, tzq;
    char sign;
    if (sscanf(t.c_str(), "%d/%d/%d,%d:%d:%d%c%d",
               &year, &mon, &day, &hour, &minute, &sec, &sign, &tzq) != 8) {
        return 0;
    }
    if (year < 2024 || year > 2100) return 0;

    int64_t local_s = daysFromCivil(year, mon, day) * 86400LL +
                      hour * 3600LL + minute * 60LL + sec;
    int32_t tz_s = tzq * 15 * 60;
    if (sign == '-') tz_s = -tz_s;
    const int64_t utc_s = local_s - tz_s;
    return utc_s > 0 ? (uint64_t)utc_s * 1000ULL : 0;
}

bool TDM2421MQTT::syncTime()
{
    String r = sendAT("AT+QLTS=2", 1500);
    uint64_t t = parseQLTS(r);
    if (t == 0) {
        setError("Network time sync failed");
        return false;
    }
    unix_base_ms = t;
    millis_base = millis();
    time_ok = true;
    tdmDebugPrintf(1, "[MQTT] network time synced: %llu ms\r\n", (unsigned long long)unix_base_ms);
    return true;
}

uint64_t TDM2421MQTT::nowUnixMs() const
{
    if (!time_ok) return 0;
    return unix_base_ms + (uint32_t)(millis() - millis_base);
}

void TDM2421MQTT::wifiChannelString(uint8_t channel, char *out, size_t out_len)
{
    if (channel >= 1 && channel <= 13) {
        const int mhz = 2407 + 5 * channel;
        snprintf(out, out_len, "%d.%03dGHz", mhz / 1000, mhz % 1000);
    } else if (channel == 14) {
        snprintf(out, out_len, "2.484GHz");
    } else {
        snprintf(out, out_len, "");
    }
}


bool TDM2421MQTT::publishSnapshot()
{
    ODID_UAS_Data uas {};
    uint8_t wifi_channel = 6;
    TDM2421Config cfg {};
    getConfig(cfg);

    if (snapshot_mutex == nullptr ||
        xSemaphoreTake(snapshot_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }
    if (!have_snapshot) {
        xSemaphoreGive(snapshot_mutex);
        return true;
    }
    memcpy(&uas, &snapshot, sizeof(uas));
    wifi_channel = snapshot_wifi_channel;
    xSemaphoreGive(snapshot_mutex);

    if (!uas.LocationValid) return true;

    // Single identity source for BOTH Broadcast RID and MQTT.
    // RemoteIDModule.ino resolves MAVLink-vs-local fallback first and writes
    // the selected identity into UAS_data. Do not re-select an identity here.
    const bool have_basic_id =
        uas.BasicIDValid[0] &&
        strnlen((const char *)uas.BasicID[0].UASID, 20) > 0;

    const char *object_id = have_basic_id
        ? (const char *)uas.BasicID[0].UASID
        : cfg.device_id;

    // Keep extension.uavSN identical to the Broadcast RID BasicID/objectId.
    // This intentionally removes cfg.uav_serial as an alternate MQTT identity.
    const char *uas_sn = object_id;

    const char *operator_id =
        (uas.OperatorIDValid &&
         strnlen((const char *)uas.OperatorID.OperatorId, 20) > 0)
            ? (const char *)uas.OperatorID.OperatorId
            : "";

    char channel[20];
    wifiChannelString(wifi_channel, channel, sizeof(channel));

    // Cached LTE signal information. refreshSignal() obtains this with AT+CSQ
    // periodically in the modem task, so publishing MQTT does not issue an AT
    // command every second. CSQ 99 means the modem cannot report RSSI.
    int local_rssi_dbm = 0;
    int local_signal_quality = 99;
    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        local_rssi_dbm = rssi_dbm;
        local_signal_quality = signal_quality_csq;
        xSemaphoreGive(state_mutex);
    }

    const uint32_t millis_now = millis();
    const uint32_t time_delta_ms = (uint32_t)(millis_now - millis_base);
    const uint64_t ts = time_ok ? (unix_base_ms + time_delta_ms) : 0;
    msg_count++;

    char payload[1024];
    const int n = snprintf(
        payload, sizeof(payload),
        "{\"deviceId\":\"%s\",\"msgCnt\":%lu,\"objects\":[{"
        "\"altitude\":%.2f,\"extension\":{"
        "\"bandWidth\":\"%s\",\"channel\":\"%s\",\"objectType\":30,"
        "\"pilotLat\":%.15f,\"pilotLon\":%.15f,\"uavModel\":\"\",\"uavSN\":\"%s\",\"operatorId\":\"%s\","
        "\"rssi\":%d,\"signalQuality\":%d},"
        "\"height\":%.2f,\"latitude\":%.15f,\"longitude\":%.15f,"
        "\"objectId\":\"%s\",\"speed\":%.2f,\"time\":%llu}],"
        "\"ptTime\":%llu,\"ver\":\"%s\"}",
        cfg.device_id,
        (unsigned long)msg_count,
        uas.Location.AltitudeGeo,
        cfg.bandwidth,
        channel,
        uas.SystemValid ? uas.System.OperatorLatitude : 0.0,
        uas.SystemValid ? uas.System.OperatorLongitude : 0.0,
        uas_sn,
        operator_id,
        local_rssi_dbm,
        local_signal_quality,
        uas.Location.Height,
        uas.Location.Latitude,
        uas.Location.Longitude,
        object_id,
        uas.Location.SpeedHorizontal,
        (unsigned long long)ts,
        (unsigned long long)ts,
        cfg.version);

    if (n <= 0 || n >= (int)sizeof(payload)) {
        setError("JSON buffer overflow");
        tdmDebugPrintln(1, "[MQTT] JSON buffer overflow");
        return false;
    }

    tdmDebugPrintf(2, "[RAW MQTT] publish msgCnt=%lu bytes=%d\r\n",
                  (unsigned long)msg_count, n);
    if (!rawMqttPublish(cfg.mqtt_topic, (const uint8_t *)payload, (size_t)n)) {
        setMQTTState(false);
        setError("Raw MQTT publish failed");
        return false;
    }

    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        last_publish_ok_ms = millis();
        xSemaphoreGive(state_mutex);
    }
    setError("");
    // In log trạng thái tổng hợp mỗi 1 giây/lần (không phụ thuộc MQTT_HZ) và tự bỏ qua
    // nếu không mở Serial Monitor để tránh nghẽn USB CDC.
#if TDM2421_DEBUG_LEVEL > 0
    static uint32_t status_last_ms = 0;
    static uint32_t status_last_count = 0;
    const uint32_t status_now_ms = millis();
    if (status_last_ms == 0) {
        status_last_ms = status_now_ms;
        status_last_count = msg_count;
    } else if ((uint32_t)(status_now_ms - status_last_ms) >= 1000U) {
        const uint32_t dt = (uint32_t)(status_now_ms - status_last_ms);
        const uint32_t dn = (uint32_t)(msg_count - status_last_count);
        const float hz = dt > 0 ? ((float)dn * 1000.0f / (float)dt) : 0.0f;
        status_last_ms = status_now_ms;
        status_last_count = msg_count;

        TDM2421GNSSFix status_gnss {};
        const bool status_gnss_valid = getGNSSFix(status_gnss, 10000);

        char gnss_text[64];
        if (!status_gnss_valid) {
            snprintf(gnss_text, sizeof(gnss_text),
                     "NO_FIX sats=%u",
                     (unsigned)status_gnss.satellites);
        } else if (status_gnss.fix_type >= 3) {
            snprintf(gnss_text, sizeof(gnss_text),
                     "3D sats=%u hdop=%.1f",
                     (unsigned)status_gnss.satellites,
                     status_gnss.hdop);
        } else if (status_gnss.fix_type == 2) {
            snprintf(gnss_text, sizeof(gnss_text),
                     "2D sats=%u hdop=%.1f",
                     (unsigned)status_gnss.satellites,
                     status_gnss.hdop);
        } else {
            snprintf(gnss_text, sizeof(gnss_text),
                     "SEARCH sats=%u",
                     (unsigned)status_gnss.satellites);
        }

        tdmDebugPrintf(1,
            "[STATUS] MQTT=%.2fHz sent=%lu sendErr=%lu GNSS=%s LTE=%d dBm\r\n",
            hz, (unsigned long)msg_count, (unsigned long)raw_send_error,
            gnss_text, local_rssi_dbm);

        tdmDebugPrintf(2,
            "[TIME DEBUG] unix_base=%llu millis_base=%lu millis_now=%lu delta=%lu ts=%llu bytes=%d sendOK=%lu\r\n",
            (unsigned long long)unix_base_ms,
            (unsigned long)millis_base,
            (unsigned long)millis_now,
            (unsigned long)time_delta_ms,
            (unsigned long long)ts, n,
            (unsigned long)raw_send_ok);
    }
#endif
    return true;
}

void TDM2421MQTT::refreshSignal()
{
    String r = sendAT("AT+CSQ", 500);
    const int p = r.indexOf("+CSQ:");
    if (p < 0) return;
    int csq = 99;
    if (sscanf(r.c_str() + p, "+CSQ: %d", &csq) == 1) {
        int dbm = 0;
        if (csq >= 0 && csq <= 31) dbm = -113 + 2 * csq;
        if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            rssi_dbm = dbm;
            signal_quality_csq = csq;
            xSemaphoreGive(state_mutex);
        }
        tdmDebugPrintf(2, "[LTE] CSQ=%d RSSI=%d dBm\r\n", csq, dbm);
    }
}

void TDM2421MQTT::handleReconfigure()
{
    if (!reconfigure_requested) return;
    reconfigure_requested = false;

    tdmDebugPrintln(1, "[RAW MQTT] configuration changed; reconnecting LTE/MQTT");
    sendAT("AT+QMTDISC=0", 800);
    sendAT("AT+QMTCLOSE=0", 800);
    sendAT("AT+QICLOSE=0", 800);
    raw_socket_ok = false;
    sendAT("AT+QIDEACT=1", 5000);
    setNetworkState(false);
    setMQTTState(false);
    time_ok = false;
    registered = false;
    setError("Reconfiguring");
}

String TDM2421MQTT::jsonEscape(const String &s)
{
    String out;
    out.reserve(s.length() + 8);
    for (size_t i = 0; i < s.length(); i++) {
        const char c = s[i];
        if (c == '\\' || c == '"') {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else if (c != '\r') {
            out += c;
        }
    }
    return out;
}

String TDM2421MQTT::configJson()
{
    TDM2421Config cfg {};
    getConfig(cfg);
    String s = "{";
    s += "\"apn\":\"" + jsonEscape(cfg.apn) + "\",";
    s += "\"mqttHost\":\"" + jsonEscape(cfg.mqtt_host) + "\",";
    s += "\"mqttPort\":" + String(cfg.mqtt_port) + ",";
    s += "\"mqttUser\":\"" + jsonEscape(cfg.mqtt_user) + "\",";
    s += "\"mqttClient\":\"" + jsonEscape(cfg.mqtt_client) + "\",";
    s += "\"mqttTopic\":\"" + jsonEscape(cfg.mqtt_topic) + "\",";
    s += "\"deviceId\":\"" + jsonEscape(cfg.device_id) + "\",";
    s += "\"uavSerial\":\"" + jsonEscape(cfg.uav_serial) + "\",";
    s += "\"version\":\"" + jsonEscape(cfg.version) + "\",";
    s += "\"bandwidth\":\"" + jsonEscape(cfg.bandwidth) + "\",";
    s += "\"publishIntervalMs\":" + String(cfg.publish_interval_ms) + ",";
    s += "\"reconnectIntervalMs\":" + String(cfg.reconnect_interval_ms) + ",";
    s += "\"c2Enable\":" + String(cfg.c2_enable ? "true" : "false") + ",";
    s += "\"c2TxTopic\":\"" + jsonEscape(cfg.c2_tx_topic) + "\",";
    s += "\"c2RxTopic\":\"" + jsonEscape(cfg.c2_rx_topic) + "\",";
    s += "\"c2BatchMs\":" + String(cfg.c2_batch_ms) + ",";
    s += "\"passwordStored\":" + String(strlen(cfg.mqtt_pass) > 0 ? "true" : "false");
    s += "}";
    return s;
}

String TDM2421MQTT::statusJson()
{
    bool local_sim = sim_ready;
    bool local_reg = registered;
    bool local_net = network_ok;
    bool local_mqtt = mqtt_ok;
    bool local_time = time_ok;
    int local_rssi = rssi_dbm;
    int local_csq = signal_quality_csq;
    uint32_t local_last_pub = last_publish_ok_ms;
    char local_ip[32] {};
    char local_err[96] {};

    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        local_sim = sim_ready;
        local_reg = registered;
        local_net = network_ok;
        local_mqtt = mqtt_ok;
        local_time = time_ok;
        local_rssi = rssi_dbm;
        local_csq = signal_quality_csq;
        local_last_pub = last_publish_ok_ms;
        copyCString(local_ip, lte_ip);
        copyCString(local_err, last_error);
        xSemaphoreGive(state_mutex);
    }

    TDM2421Config cfg {};
    getConfig(cfg);

    String s = "{";
    s += "\"simReady\":" + String(local_sim ? "true" : "false") + ",";
    s += "\"lteRegistered\":" + String(local_reg ? "true" : "false") + ",";
    s += "\"pdpActive\":" + String(local_net ? "true" : "false") + ",";
    s += "\"mqttConnected\":" + String(local_mqtt ? "true" : "false") + ",";
    s += "\"c2Subscribed\":" + String(c2_subscribed ? "true" : "false") + ",";
    s += "\"timeSynced\":" + String(local_time ? "true" : "false") + ",";
    s += "\"lteIp\":\"" + jsonEscape(local_ip) + "\",";
    s += "\"rssiDbm\":" + String(local_rssi) + ",";
    s += "\"signalQuality\":" + String(local_csq) + ",";
    s += "\"msgCnt\":" + String(msg_count) + ",";
    s += "\"publishIntervalMs\":" + String(cfg.publish_interval_ms) + ",";
    s += "\"lastPublishAgeMs\":" + String(local_last_pub == 0 ? 0 : (millis() - local_last_pub)) + ",";
    s += "\"lastError\":\"" + jsonEscape(local_err) + "\",";
    s += "\"broker\":\"" + jsonEscape(String(cfg.mqtt_host) + ":" + String(cfg.mqtt_port)) + "\",";
    s += "\"topic\":\"" + jsonEscape(cfg.mqtt_topic) + "\",";
    s += "\"c2TxTopic\":\"" + jsonEscape(cfg.c2_tx_topic) + "\",";
    s += "\"c2RxTopic\":\"" + jsonEscape(cfg.c2_rx_topic) + "\",";
    s += "\"c2BatchMs\":" + String(cfg.c2_batch_ms) + ",";
    s += "\"deviceId\":\"" + jsonEscape(cfg.device_id) + "\",";
    s += "\"uavSerial\":\"" + jsonEscape(cfg.uav_serial) + "\",";
    TDM2421GNSSFix gf {};
    const bool gv = getGNSSFix(gf, 15000);
    s += "\"gnssValid\":" + String(gv ? "true" : "false") + ",";
    s += "\"gnssLat\":" + String(gf.latitude, 7) + ",";
    s += "\"gnssLon\":" + String(gf.longitude, 7) + ",";
    s += "\"gnssAlt\":" + String(gf.altitude_m, 1) + ",";
    s += "\"gnssSpeed\":" + String(gf.speed_mps, 2) + ",";
    s += "\"gnssCourse\":" + String(gf.course_deg, 1) + ",";
    s += "\"gnssEngineRunning\":" + String(gnss_started ? "true" : "false") + ",";
    s += "\"gnssHdop\":" + String(gf.hdop, 2) + ",";
    s += "\"gnssPdop\":" + String(gf.pdop, 2) + ",";
    s += "\"gnssVdop\":" + String(gf.vdop, 2) + ",";
    s += "\"gnssDopValid\":" + String(gf.dop_valid ? "true" : "false") + ",";
    s += "\"gnssFixType\":" + String(gf.fix_type) + ",";
    String fixText = gf.fix_type == 3 ? "3D FIX" : (gf.fix_type == 2 ? "2D FIX" : "NO FIX");
    s += "\"gnssFixText\":\"" + fixText + "\",";
    s += "\"gnssSatellites\":" + String(gf.satellites) + ",";
    s += "\"gnssSatellitesUsed\":" + String(gf.satellites_used) + ",";
    s += "\"gnssCourseValid\":" + String(gf.course_valid ? "true" : "false") + ",";
    s += "\"gnssUtc\":\"" + jsonEscape(gf.utc) + "\",";
    s += "\"gnssDate\":\"" + jsonEscape(gf.date) + "\",";
    s += "\"gnssAgeMs\":" + String(gf.age_ms);
    s += "}";
    return s;
}

void TDM2421MQTT::task()
{
    delay(1000);

    uint32_t last_reconnect = 0;
    uint32_t last_publish = 0;
    uint32_t last_time_sync = 0;
    uint32_t last_signal = 0;
    uint32_t last_gnss = 0;
    uint32_t last_modem_probe = 0;
    bool modem_seen = false;

    for (;;) {
        const uint32_t now = millis();
        handleReconfigure();

        TDM2421Config cfg {};
        getConfig(cfg);

        // Keep EG800K GNSS alive independently of MQTT. During an active 10 Hz
        // MQTT stream GNSS polling is intentionally slow because GNSS and MQTT
        // share the same modem UART. The latest GNSS fix remains available as a
        // fallback, while the 100 ms MQTT deadline gets priority.
        if (!modem_seen && now - last_modem_probe >= 5000UL) {
            last_modem_probe = now;
            modem_seen = modemReady();
            if (modem_seen) ensureGNSS();
        }

        if (!network_ok || !mqtt_ok) {
            // Khi chưa nối xong MQTT, vẫn giữ EG800K GNSS cập nhật đều để sẵn sàng dự phòng.
            if (modem_seen && now - last_gnss >= 1500UL) {
                last_gnss = now;
                refreshGNSS(false);
            }
            if (now - last_reconnect >= cfg.reconnect_interval_ms) {
                last_reconnect = now;
                if (!modem_seen) {
                    modem_seen = modemReady();
                    if (!modem_seen) {
                        delay(1000);
                        continue;
                    }
                    ensureGNSS();
                }
                if (!ensureNetwork()) {
                    // GNSS is independent; do not invalidate an existing GNSS fix.
                    delay(100);
                    continue;
                }
                if (!time_ok) syncTime();
                ensureMQTT();
            }
            delay(20);
            continue;
        }

        // Re-sync network time hourly.
        if (!time_ok || now - last_time_sync >= 3600000UL) {
            if (syncTime()) last_time_sync = now;
        }

        // Luôn bơm dữ liệu từ Serial0 qua demuxer trước để nhận tức thì lệnh C2 uplink từ MQTT.
        pumpModemRX();
        if (!network_ok || !mqtt_ok || !raw_socket_ok) {
            delay(5);
            continue;
        }

        // Ưu tiên 1 (trên đường 4G): chạy cố định theo chu kỳ tick c2_batch_ms (mặc định 33 ms ~ 30 Hz).
        // Đến hạn tick, hoán đổi cặp Ping-Pong Double Buffer trong vài microsecond rồi gửi toàn bộ
        // Send Buffer lên MQTT; trong lúc gửi, loop() vẫn ghi frame mới từ FC vào Fill Buffer còn lại.
        static uint32_t c2_stat_last_ms = 0;
        static uint32_t c2_stat_pubs = 0;
        static uint32_t c2_stat_frames = 0;
        static uint32_t c2_stat_bytes = 0;
        if (cfg.c2_enable && cfg.c2_tx_topic[0] != '\0' &&
            (uint32_t)(now - c2_last_batch_ms) >= cfg.c2_batch_ms) {
            if (c2_last_batch_ms == 0 ||
                (uint32_t)(now - c2_last_batch_ms) >= 2U * cfg.c2_batch_ms) {
                c2_last_batch_ms = now;
            } else {
                c2_last_batch_ms += cfg.c2_batch_ms;
            }

            if (tdm2421_c2.swapDownlinkForSend() > 0U) {
                uint8_t c2_batch[C2_BATCH_MAX_BYTES];
                for (;;) {
                    uint16_t c2_frames = 0;
                    const size_t batch_len = tdm2421_c2.popSendBatch(
                        c2_batch, sizeof(c2_batch), &c2_frames);
                    if (batch_len == 0) {
                        break;
                    }
                    if (!publishC2Downlink(cfg.c2_tx_topic, c2_batch, batch_len)) {
                        break;
                    }
                    c2_stat_pubs++;
                    c2_stat_frames += c2_frames;
                    c2_stat_bytes += (uint32_t)batch_len;
                    pumpModemRX();
                }
            }
        }

#if TDM2421_DEBUG_LEVEL > 0 && !TDM2421_ENABLE_RID
        if (c2_stat_last_ms == 0) {
            c2_stat_last_ms = now;
        } else if ((uint32_t)(now - c2_stat_last_ms) >= 1000U) {
            const uint32_t dt = (uint32_t)(now - c2_stat_last_ms);
            const float pub_hz = dt > 0 ? ((float)c2_stat_pubs * 1000.0f / (float)dt) : 0.0f;
            const TDM2421C2Stats cst = tdm2421_c2.getStats();
            tdmDebugPrintf(1,
                "[C2 STAT] pub=%.1fHz (%lu frames, %lu B/s) rx=%lu txDrop=%lu sendErr=%lu\r\n",
                pub_hz,
                (unsigned long)c2_stat_frames,
                (unsigned long)(dt > 0 ? (c2_stat_bytes * 1000UL / dt) : 0UL),
                (unsigned long)cst.rxFrames,
                (unsigned long)cst.txDropped,
                (unsigned long)raw_send_error);
            c2_stat_last_ms = now;
            c2_stat_pubs = 0;
            c2_stat_frames = 0;
            c2_stat_bytes = 0;
        }
#else
        (void)c2_stat_last_ms; (void)c2_stat_pubs; (void)c2_stat_frames; (void)c2_stat_bytes;
#endif

#if TDM2421_ENABLE_RID
        // Ưu tiên 2: publish JSON Remote ID theo chu kỳ deadline định sẵn.
        if ((uint32_t)(now - last_publish) >= cfg.publish_interval_ms) {
            if (last_publish == 0 || (uint32_t)(now - last_publish) >= 2U * cfg.publish_interval_ms) {
                last_publish = now;
            } else {
                last_publish += cfg.publish_interval_ms;
            }
            publishSnapshot();
            pumpModemRX();
            delay(1);
            continue;
        }
#else
        (void)last_publish; // Tạm tắt publish JSON Remote ID để tối ưu băng thông cho C2 30 Hz.
#endif

        // Keepalive MQTT khi hoàn toàn không có gói RID hay C2 nào đi qua trong 30 giây.
        if ((uint32_t)(now - mqtt_last_ping_ms) >= 30000U) {
            const uint8_t pingreq[2] = {0xC0, 0x00};
            if (rawTcpSend(pingreq, sizeof(pingreq), 500)) {
                mqtt_last_ping_ms = millis();
            }
        }

        // Warm-lock thông minh cho EG800K GNSS:
        //   - Khi FC đang có GPS tươi: poll thưa mỗi 10s (bỏ GSA) để giữ warm-lock mà không làm khựng C2 30 Hz.
        //   - Khi FC mất GPS: chuyển sang poll 1.5s/lần để chuyển dự phòng tức thì cho Remote ID.
        const uint32_t gnss_period_ms = mav_location_fresh_flag ? 10000UL : 1500UL;
        if (modem_seen && (uint32_t)(now - last_gnss) >= gnss_period_ms) {
            last_gnss = now;
            refreshGNSS(mav_location_fresh_flag);
        }

        if (now - last_signal >= SIGNAL_REFRESH_INTERVAL_MS) {
            last_signal = now;
            refreshSignal();
        }

        delay(1);
    }
}

#endif // BOARD_TDM2421
