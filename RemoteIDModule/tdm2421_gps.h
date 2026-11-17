#pragma once

#include <Arduino.h>
#include "board_config.h"
#include "tdm2421_mqtt.h"

#if defined(BOARD_TDM2421) && TDM2421_USE_EXT_GPS

/*
  Driver đọc module GPS ngoài (NMEA/UBX 1/5/10 Hz) trên cổng Serial1 cho BOARD_TDM2421.

  Dùng cho trường hợp Flight Controller không có cổng UART MAVLink ra ngoài:
  - Tự động cấu hình module u-blox (9600 -> 115200 bps, chọn tần số 1/5/10 Hz lưu trong NVS,
    chỉ bật GGA+RMC để giảm tải UART).
  - Phát hiện trạng thái đã cắm/chưa cắm dựa trên bản tin NMEA nhận được (kể cả khi chưa có fix vệ tinh).
  - Hỗ trợ chế độ thu mẫu 1 giây không chặn (non-blocking) cho nút Test module trên Web UI.
 */
class TDM2421ExtGPS {
public:
    void begin();
    void update();

    // Trả về true nếu có tọa độ hợp lệ mới hơn max_age_ms.
    // Biến timestamp_s nhận số giây tính từ đầu giờ UTC hiện tại (0..3600)
    // theo đúng chuẩn trường Location.TimeStamp của OpenDroneID.
    bool getFix(TDM2421GNSSFix &out, float &timestamp_s, uint32_t max_age_ms = 1500) const;

    // Kiểm tra module GPS đã cắm và đang truyền bản tin NMEA hay chưa.
    bool isConnected() const;

    // Đọc và thay đổi tần số tính toán GPS (chỉ nhận 1, 5 hoặc 10 Hz; lưu vào NVS).
    uint8_t getConfiguredHz() const { return configured_hz_; }
    uint8_t getMeasuredHz() const;
    bool setRateHz(uint8_t hz);

    // Chế độ Test module trong 1 giây: gom thống kê + câu NMEA thô mà không làm treo loop().
    void startTestCapture(uint32_t duration_ms = 1000);
    String statusJson() const;
    String testStatusJson();

private:
    static void sendUBX(uint8_t msg_class, uint8_t msg_id,
                        const uint8_t *payload, uint16_t len);
    static void setNmeaMsgRate(uint8_t nmea_msg_id, uint8_t rate);
    void applyUbxRateConfig(uint8_t hz);
    static bool isValidHz(uint8_t hz) { return hz == 1 || hz == 5 || hz == 10; }

    static bool verifyChecksum(const char *sentence);
    static size_t splitCsvFields(char *sentence, char *fields[], size_t max_fields);
    static bool parseNmeaCoord(const char *val_str, const char *hemi_str,
                               bool is_lon, double &out_deg);
    static float parseUtcTimestampSec(const char *utc_str);
    static String jsonEscape(const char *s);

    void processLine(char *line);
    void handleRMC(char *fields[], size_t nfields);
    void handleGGA(char *fields[], size_t nfields);
    void handleGSV(char *fields[], size_t nfields);
    void finalizeTestIfElapsed();

    char line_buf_[128] {};
    size_t line_len_ = 0;

    // Mốc thời gian phát hiện kết nối vật lý và đo tần số thực tế.
    uint32_t last_rx_ms_ = 0;
    uint32_t last_nmea_ms_ = 0;
    uint8_t configured_hz_ = 10;
    uint32_t rate_win_start_ms_ = 0;
    uint16_t rate_win_count_ = 0;
    uint8_t measured_hz_ = 0;

    // Dữ liệu GGA mới nhất (cung cấp độ cao MSL, số vệ tinh và HDOP).
    bool gga_valid_ = false;
    uint32_t last_gga_ms_ = 0;
    float gga_alt_m_ = 0.0f;
    uint8_t gga_sats_ = 0;
    float gga_hdop_ = 99.9f;

    // Bản ghi tọa độ hoàn chỉnh (kết hợp RMC + GGA) dùng chung cho RemoteID và Web Dashboard.
    TDM2421GNSSFix current_fix_ {};
    float current_timestamp_s_ = 0.0f;
    uint32_t last_fix_ms_ = 0;

    // Trạng thái phiên thu mẫu 1 giây cho nút Test module trên Web UI.
    bool test_active_ = false;
    bool test_done_ = false;
    bool test_gsv_enabled_ = false;
    uint32_t test_start_ms_ = 0;
    uint32_t test_duration_ms_ = 1000;
    uint32_t test_bytes_ = 0;
    uint16_t test_rmc_count_ = 0;
    uint16_t test_gga_count_ = 0;
    uint16_t test_gsv_count_ = 0;
    uint8_t test_sats_in_view_ = 0;
    uint8_t test_max_snr_ = 0;
    char test_samples_[2048] {};
    size_t test_samples_len_ = 0;
};

extern TDM2421ExtGPS tdm2421_gps;

#endif // BOARD_TDM2421 && TDM2421_USE_EXT_GPS
