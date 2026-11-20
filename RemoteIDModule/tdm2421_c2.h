#pragma once

#include <stddef.h>
#include <stdint.h>

#if !defined(ARDUINO) || defined(BOARD_TDM2421)

#if defined(ARDUINO)
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

// Kích thước tối đa 1 frame MAVLink v2 trên dây (header 10 + payload 255 + CRC 2 + sig 13 = 280).
static constexpr size_t TDM2421_C2_MAX_FRAME_LEN = 280;
// Mỗi buffer trong cặp Ping-Pong Double Buffer downlink (1 buffer đang gửi, 1 buffer nhận từ FC).
static constexpr size_t TDM2421_C2_DOWNLINK_BUF_SIZE = 4096;
// Hàng đợi uplink (MQTT -> FC): số frame tối đa chờ ghi ra Serial1.
static constexpr size_t TDM2421_C2_UPLINK_SLOTS = 16;

struct TDM2421C2Stats {
    bool enabled;
    uint32_t txFrames;
    uint32_t txBytes;
    uint32_t txDropped;
    uint32_t rxFrames;
    uint32_t rxBytes;
    uint32_t rxDropped;
    uint32_t rxInvalid;
    uint32_t lastTxMs;
    uint32_t lastRxMs;
    size_t downlinkPendingBytes;
    size_t uplinkPendingFrames;
};

/*
  Cầu nối đệm dữ liệu 2 chiều không cấp phát heap động giữa:
    - MAVLinkSerial (loop() trên Serial1)
    - TDM2421MQTT (tdm_mqtt task trên Serial0 / EG800K)

  Chiều Downlink (FC -> MQTT) dùng kiến trúc Double Buffer (Ping-Pong):
    - Fill Buffer: loop() ghi liên tục các frame từ FC vào đây.
    - Send Buffer: khi đến tick (ví dụ 33 ms ~ 30 Hz), task MQTT đổi chỗ 2 buffer
      trong vài microsecond rồi rút hết Send Buffer gửi lên 4G mà không bao giờ
      giữ lock cản trở loop() nhận frame mới của FC vào Fill Buffer mới.
 */
class TDM2421C2Bridge {
public:
    void begin();

    void setEnabled(bool en);
    bool isEnabled() const;

    // Chiều Downlink (FC -> ESP32 -> MQTT) dùng Ping-Pong Double Buffer:
    bool pushDownlinkFrame(const uint8_t *frame, uint16_t len, uint32_t now_ms);
    // Đổi Fill Buffer thành Send Buffer nếu Send Buffer đã xả hết; trả về số byte còn trong Send Buffer.
    size_t swapDownlinkForSend();
    size_t downlinkBytes() const;
    // Rút 1 batch (<= max_bytes, tròn frame MAVLink) từ Send Buffer (chỉ gọi từ task MQTT).
    size_t popSendBatch(uint8_t *out, size_t max_bytes, uint16_t *out_frames = nullptr);
    void clearDownlinkBacklog();

    // Chiều Uplink: MQTT -> ESP32 -> FC
    bool pushUplinkFrame(const uint8_t *frame, uint16_t len, uint32_t now_ms);
    bool popUplinkFrame(uint8_t *out, uint16_t &out_len);
    void clearUplinkQueue();

    void recordRxInvalid();
    TDM2421C2Stats getStats() const;

#if defined(ARDUINO)
    String statusJson(bool subscribed) const;
#endif

private:
    bool lock(uint32_t wait_ms) const;
    void unlock() const;

#if defined(ARDUINO)
    mutable SemaphoreHandle_t mutex_ = nullptr;
#endif

    volatile bool enabled_ = true;

    // Ping-Pong Double Buffer cho Downlink (2 x 4096 B).
    // Trong mỗi buffer, các frame được lưu nối tiếp theo định dạng [uint16_t len][frame_bytes].
    struct DownlinkBuffer {
        uint8_t data[TDM2421_C2_DOWNLINK_BUF_SIZE];
        size_t used;
        size_t read_pos;
    };
    DownlinkBuffer dl_bufs_[2] {};
    uint8_t fill_idx_ = 0; // Chỉ số buffer đang nhận dữ liệu mới từ FC (loop())
    uint8_t send_idx_ = 1; // Chỉ số buffer đang xả lên MQTT (tdm_mqtt task)

    // Uplink frame queue
    struct UplinkSlot {
        uint16_t len;
        uint8_t data[TDM2421_C2_MAX_FRAME_LEN];
    };
    UplinkSlot ul_slots_[TDM2421_C2_UPLINK_SLOTS] {};
    size_t ul_head_ = 0;
    size_t ul_tail_ = 0;
    size_t ul_count_ = 0;

    // Thống kê
    uint32_t tx_frames_ = 0;
    uint32_t tx_bytes_ = 0;
    uint32_t tx_dropped_ = 0;
    uint32_t rx_frames_ = 0;
    uint32_t rx_bytes_ = 0;
    uint32_t rx_dropped_ = 0;
    uint32_t rx_invalid_ = 0;
    uint32_t last_tx_ms_ = 0;
    uint32_t last_rx_ms_ = 0;
};

extern TDM2421C2Bridge tdm2421_c2;

#endif // !ARDUINO || BOARD_TDM2421
