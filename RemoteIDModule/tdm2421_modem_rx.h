#pragma once

#include <stddef.h>
#include <stdint.h>

#if !defined(ARDUINO) || defined(BOARD_TDM2421)

static constexpr size_t TDM2421_MODEM_AT_BUF_SIZE = 2048;
static constexpr size_t TDM2421_MODEM_LINE_BUF_SIZE = 192;
static constexpr size_t TDM2421_MQTT_RX_RING_SIZE = 2048;
static constexpr size_t TDM2421_MQTT_MAX_RX_PKT = 1600;

/*
  Bộ phân tách luồng Serial0 (Quectel EG800K direct-push mode) an toàn nhị phân:
    - Tách các dòng URC "+QIURC: \"recv\",0,<len>\\r\\n<binary>" vào vòng đệm MQTT RX.
    - Giữ nguyên toàn bộ phản hồi text AT (OK, ERROR, >, SEND OK, +CSQ, +QGPSLOC)
      trong at_buf_ mà không bao giờ làm lẫn byte 0x00 hoặc payload nhị phân vào AT text.
    - Giải mã các gói MQTT 3.1.1 đến (CONNACK, SUBACK, PUBLISH, PINGRESP), hỗ trợ
      cả gói bị cắt mảnh qua nhiều lần +QIURC hoặc nhiều gói gộp chung 1 chunk.
 */
class TDM2421ModemRxDemux {
public:
    typedef void (*PublishCallback)(const char *topic, size_t topic_len,
                                    const uint8_t *payload, size_t payload_len,
                                    void *ctx);

    void resetAll();
    void resetMqttStream();
    void clearATText();

    // Nạp byte từ Serial0 vào state machine.
    void feedByte(uint8_t b, uint32_t now_ms);
    void feedBytes(const uint8_t *data, size_t len, uint32_t now_ms);
    void checkTimeout(uint32_t now_ms, uint32_t timeout_ms = 1000);

    // Giải mã các gói MQTT hoàn chỉnh hiện có trong vòng đệm RX.
    void processMqttPackets(PublishCallback on_publish, void *ctx);

    // Truy xuất bộ đệm text AT thuần (luôn kết thúc NUL).
    const char *atText() const { return at_buf_; }
    size_t atLength() const { return at_len_; }
    bool atContains(const char *needle) const;

    // Cờ trạng thái do parser ghi nhận.
    bool consumeSocketClosed();
    bool consumeProtocolError();

    bool takeConnack(uint8_t &out_rc);
    bool takeSuback(uint16_t &out_pid, uint8_t &out_rc);
    bool takePingresp();

    size_t mqttPendingBytes() const { return mq_used_; }
    bool inPayloadState() const { return state_ == State::RECV_PAYLOAD; }

private:
    enum class State : uint8_t {
        TEXT_LINE = 0,
        RECV_PAYLOAD = 1,
    };

    void appendAtChar(char c);
    void stripAtTail(size_t n);
    void handleCompletedLine(uint32_t now_ms);

    bool mqPushByte(uint8_t b);
    uint8_t mqPeekByte(size_t offset) const;
    void mqReadBytes(uint8_t *dst, size_t len);

    State state_ = State::TEXT_LINE;
    size_t recv_remaining_ = 0;
    uint32_t recv_start_ms_ = 0;

    char at_buf_[TDM2421_MODEM_AT_BUF_SIZE] {};
    size_t at_len_ = 0;

    char line_buf_[TDM2421_MODEM_LINE_BUF_SIZE] {};
    size_t line_len_ = 0;

    uint8_t mqtt_ring_[TDM2421_MQTT_RX_RING_SIZE] {};
    size_t mq_head_ = 0;
    size_t mq_tail_ = 0;
    size_t mq_used_ = 0;

    uint8_t pkt_buf_[TDM2421_MQTT_MAX_RX_PKT] {};

    bool socket_closed_ = false;
    bool protocol_error_ = false;

    bool connack_seen_ = false;
    uint8_t connack_rc_ = 0;

    bool suback_seen_ = false;
    uint16_t suback_pid_ = 0;
    uint8_t suback_rc_ = 0;

    bool pingresp_seen_ = false;
};

#endif // !ARDUINO || BOARD_TDM2421
