#pragma once

#include <stddef.h>
#include <stdint.h>

#if !defined(ARDUINO) || defined(BOARD_TDM2421)

#if defined(ARDUINO)
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

// Nguồn tọa độ đang được chọn trong vùng nhớ chung cho Remote ID.
enum class TDM2421GnssSource : uint8_t {
    NONE = 0,
    FC_MAVLINK = 1,  // Ưu tiên 1: gói tọa độ parse từ FC qua UART1
    EG800K_GNSS = 2  // Ưu tiên 2: module GNSS gắn liền trên EG800K (dự phòng)
};

// Bản ghi tọa độ thống nhất lưu trong vùng nhớ chung cho cả 2 nguồn GPS.
struct TDM2421GnssRecord {
    bool valid;
    double latitude;
    double longitude;
    float altitude_msl_m;
    float height_agl_m;
    float speed_mps;
    float speed_vertical_mps;
    float course_deg;
    float hdop;
    float pdop;
    float vdop;
    uint8_t fix_type;        // 0/1 = no fix, 2 = 2D, 3+ = 3D
    uint8_t satellites;
    uint8_t satellites_used;
    bool course_valid;
    bool dop_valid;
    bool height_valid;
    uint8_t odid_status;     // ODID_status_t (ví dụ ODID_STATUS_AIRBORNE)
    float odid_timestamp_s;  // Giây sau đầu giờ UTC cho OpenDroneID Location
    char utc[16];
    char date[8];
    char sub_source[32];     // Tên gói cụ thể: GLOBAL_POSITION_INT, GPS_RAW_INT, EG800K...
    uint32_t updated_ms;
    uint32_t age_ms;
};

/*
  Vùng nhớ chung quản lý 2 nguồn tọa độ GPS độc lập cho TDM2421:
    1) Nguồn từ Flight Controller (MAVLink UART1: OPEN_DRONE_ID_LOCATION / GLOBAL_POSITION_INT / GPS_RAW_INT).
    2) Nguồn từ module GNSS tích hợp trên EG800K (AT+QGPSLOC=2).

  An toàn đa luồng giữa loop() (MAVLink + Remote ID) và task tdm_mqtt (EG800K + MQTT).
 */
class TDM2421GnssStore {
public:
    void begin();

    // Cập nhật nguồn GPS 1 (từ FC MAVLink).
    void updateFc(const TDM2421GnssRecord &rec);
    // Cập nhật nguồn GPS 2 (từ EG800K GNSS tích hợp).
    void updateEg800k(const TDM2421GnssRecord &rec);
    void updateEg800kDop(float pdop, float hdop, float vdop, uint8_t used, uint32_t now_ms);

    // Đọc riêng từng nguồn (trả về true nếu dữ liệu hợp lệ và chưa quá max_age_ms).
    bool getFc(TDM2421GnssRecord &out, uint32_t max_age_ms, uint32_t now_ms) const;
    bool getEg800k(TDM2421GnssRecord &out, uint32_t max_age_ms, uint32_t now_ms) const;

    // Chọn nguồn đang hoạt động cho Remote ID:
    //   Ưu tiên 1: FC_MAVLINK (còn tươi <= fc_max_age_ms)
    //   Ưu tiên 2: EG800K_GNSS (còn tươi <= eg_max_age_ms)
    TDM2421GnssSource selectActive(TDM2421GnssRecord &out, uint32_t now_ms,
                                   uint32_t fc_max_age_ms = 6500,
                                   uint32_t eg_max_age_ms = 15000) const;

private:
    bool lock(uint32_t wait_ms) const;
    void unlock() const;

#if defined(ARDUINO)
    mutable SemaphoreHandle_t mutex_ = nullptr;
#endif

    TDM2421GnssRecord fc_slot_ {};
    TDM2421GnssRecord eg_slot_ {};
    uint32_t eg_last_dop_ms_ = 0;
};

extern TDM2421GnssStore tdm2421_gnss;

#endif // !ARDUINO || BOARD_TDM2421
