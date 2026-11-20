#include "tdm2421_modem_rx.h"

#if !defined(ARDUINO) || defined(BOARD_TDM2421)

#include <stdio.h>
#include <string.h>

void TDM2421ModemRxDemux::resetAll()
{
    state_ = State::TEXT_LINE;
    recv_remaining_ = 0;
    recv_start_ms_ = 0;
    at_len_ = 0;
    at_buf_[0] = '\0';
    line_len_ = 0;
    line_buf_[0] = '\0';
    mq_head_ = 0;
    mq_tail_ = 0;
    mq_used_ = 0;
    socket_closed_ = false;
    protocol_error_ = false;
    connack_seen_ = false;
    connack_rc_ = 0;
    suback_seen_ = false;
    suback_pid_ = 0;
    suback_rc_ = 0;
    pingresp_seen_ = false;
}

void TDM2421ModemRxDemux::resetMqttStream()
{
    if (state_ == State::RECV_PAYLOAD) {
        state_ = State::TEXT_LINE;
        recv_remaining_ = 0;
        line_len_ = 0;
        line_buf_[0] = '\0';
    }
    mq_head_ = 0;
    mq_tail_ = 0;
    mq_used_ = 0;
    connack_seen_ = false;
    suback_seen_ = false;
    pingresp_seen_ = false;
}

void TDM2421ModemRxDemux::clearATText()
{
    at_len_ = 0;
    at_buf_[0] = '\0';
}

bool TDM2421ModemRxDemux::atContains(const char *needle) const
{
    if (needle == nullptr || needle[0] == '\0') {
        return false;
    }
    return strstr(at_buf_, needle) != nullptr;
}

bool TDM2421ModemRxDemux::consumeSocketClosed()
{
    const bool v = socket_closed_;
    socket_closed_ = false;
    return v;
}

bool TDM2421ModemRxDemux::consumeProtocolError()
{
    const bool v = protocol_error_;
    protocol_error_ = false;
    return v;
}

bool TDM2421ModemRxDemux::takeConnack(uint8_t &out_rc)
{
    if (!connack_seen_) {
        return false;
    }
    connack_seen_ = false;
    out_rc = connack_rc_;
    return true;
}

bool TDM2421ModemRxDemux::takeSuback(uint16_t &out_pid, uint8_t &out_rc)
{
    if (!suback_seen_) {
        return false;
    }
    suback_seen_ = false;
    out_pid = suback_pid_;
    out_rc = suback_rc_;
    return true;
}

bool TDM2421ModemRxDemux::takePingresp()
{
    if (!pingresp_seen_) {
        return false;
    }
    pingresp_seen_ = false;
    return true;
}

void TDM2421ModemRxDemux::appendAtChar(char c)
{
    if (c == '\0') {
        return;
    }
    if (at_len_ >= TDM2421_MODEM_AT_BUF_SIZE - 1U) {
        const size_t drop = TDM2421_MODEM_AT_BUF_SIZE / 2U;
        const size_t keep = at_len_ - drop;
        memmove(at_buf_, at_buf_ + drop, keep);
        at_len_ = keep;
    }
    at_buf_[at_len_++] = c;
    at_buf_[at_len_] = '\0';
}

void TDM2421ModemRxDemux::stripAtTail(size_t n)
{
    if (n > at_len_) {
        at_len_ = 0;
    } else {
        at_len_ -= n;
    }
    at_buf_[at_len_] = '\0';
}

bool TDM2421ModemRxDemux::mqPushByte(uint8_t b)
{
    if (mq_used_ >= TDM2421_MQTT_RX_RING_SIZE) {
        return false;
    }
    mqtt_ring_[mq_tail_] = b;
    mq_tail_ = (mq_tail_ + 1U) % TDM2421_MQTT_RX_RING_SIZE;
    mq_used_++;
    return true;
}

uint8_t TDM2421ModemRxDemux::mqPeekByte(size_t offset) const
{
    const size_t idx = (mq_head_ + offset) % TDM2421_MQTT_RX_RING_SIZE;
    return mqtt_ring_[idx];
}

void TDM2421ModemRxDemux::mqReadBytes(uint8_t *dst, size_t len)
{
    size_t first = TDM2421_MQTT_RX_RING_SIZE - mq_head_;
    if (first > len) {
        first = len;
    }
    memcpy(dst, mqtt_ring_ + mq_head_, first);
    if (len > first) {
        memcpy(dst + first, mqtt_ring_, len - first);
    }
    mq_head_ = (mq_head_ + len) % TDM2421_MQTT_RX_RING_SIZE;
    mq_used_ -= len;
}

void TDM2421ModemRxDemux::handleCompletedLine(uint32_t now_ms)
{
    // Kiểm tra URC nhận dữ liệu TCP của Quectel EG800K: +QIURC: "recv",0,<len>\r\n
    const char *recv_urc = strstr(line_buf_, "+QIURC: \"recv\",");
    if (recv_urc != nullptr) {
        const size_t prefix_len = (size_t)(recv_urc - line_buf_);
        const size_t urc_len = line_len_ - prefix_len;
        // Giữ nguyên bất kỳ tiền tố nào đứng trước URC (ví dụ dấu nhắc "> " của QISEND).
        stripAtTail(urc_len);

        int sock = -1;
        int payload_len = -1;
        if (sscanf(recv_urc, "+QIURC: \"recv\",%d,%d", &sock, &payload_len) == 2 && sock == 0) {
            if (payload_len > 0 && (size_t)payload_len <= TDM2421_MQTT_RX_RING_SIZE) {
                state_ = State::RECV_PAYLOAD;
                recv_remaining_ = (size_t)payload_len;
                recv_start_ms_ = now_ms;
            } else if (payload_len < 0 || (size_t)payload_len > TDM2421_MQTT_RX_RING_SIZE) {
                protocol_error_ = true;
            }
        }
    } else if (strstr(line_buf_, "+QIURC: \"closed\",0") != nullptr ||
               strstr(line_buf_, "NO CARRIER") != nullptr) {
        socket_closed_ = true;
    }

    line_len_ = 0;
    line_buf_[0] = '\0';
}

void TDM2421ModemRxDemux::feedByte(uint8_t b, uint32_t now_ms)
{
    if (state_ == State::RECV_PAYLOAD) {
        if (!mqPushByte(b)) {
            protocol_error_ = true;
        }
        recv_remaining_--;
        if (recv_remaining_ == 0) {
            state_ = State::TEXT_LINE;
            line_len_ = 0;
            line_buf_[0] = '\0';
        }
        return;
    }

    appendAtChar((char)b);
    if (b != '\0') {
        if (line_len_ >= TDM2421_MODEM_LINE_BUF_SIZE - 1U) {
            const size_t drop = TDM2421_MODEM_LINE_BUF_SIZE / 2U;
            const size_t keep = line_len_ - drop;
            memmove(line_buf_, line_buf_ + drop, keep);
            line_len_ = keep;
        }
        line_buf_[line_len_++] = (char)b;
        line_buf_[line_len_] = '\0';
    }

    if (b == '\n') {
        handleCompletedLine(now_ms);
    }
}

void TDM2421ModemRxDemux::feedBytes(const uint8_t *data, size_t len, uint32_t now_ms)
{
    if (data == nullptr) {
        return;
    }
    for (size_t i = 0; i < len; i++) {
        feedByte(data[i], now_ms);
    }
}

void TDM2421ModemRxDemux::checkTimeout(uint32_t now_ms, uint32_t timeout_ms)
{
    if (state_ == State::RECV_PAYLOAD &&
        (uint32_t)(now_ms - recv_start_ms_) >= timeout_ms) {
        state_ = State::TEXT_LINE;
        recv_remaining_ = 0;
        line_len_ = 0;
        line_buf_[0] = '\0';
        protocol_error_ = true;
    }
}

void TDM2421ModemRxDemux::processMqttPackets(PublishCallback on_publish, void *ctx)
{
    while (mq_used_ >= 2U) {
        const uint8_t b0 = mqPeekByte(0);

        uint32_t rem_len = 0;
        uint32_t mult = 1;
        size_t hdr_len = 0;
        bool incomplete = false;
        for (size_t i = 1; i <= 4; i++) {
            if (mq_used_ <= i) {
                incomplete = true;
                break;
            }
            const uint8_t enc = mqPeekByte(i);
            rem_len += (uint32_t)(enc & 0x7FU) * mult;
            if ((enc & 0x80U) == 0) {
                hdr_len = i + 1U;
                break;
            }
            mult *= 128U;
            if (i == 4) {
                protocol_error_ = true;
                return;
            }
        }
        if (incomplete) {
            break;
        }
        if (hdr_len == 0 || hdr_len + rem_len > TDM2421_MQTT_MAX_RX_PKT) {
            protocol_error_ = true;
            return;
        }
        const size_t total_len = hdr_len + rem_len;
        if (mq_used_ < total_len) {
            // Gói MQTT bị tách làm nhiều mảnh TCP: chờ phần còn lại.
            break;
        }

        mqReadBytes(pkt_buf_, total_len);

        const uint8_t pkt_type = (b0 >> 4) & 0x0FU;
        const uint8_t flags = b0 & 0x0FU;
        const uint8_t *body = pkt_buf_ + hdr_len;

        switch (pkt_type) {
        case 2: // CONNACK
            if (rem_len >= 2U) {
                connack_seen_ = true;
                connack_rc_ = body[1];
            }
            break;
        case 3: { // PUBLISH
            if (rem_len < 2U) {
                break;
            }
            const uint16_t topic_len = ((uint16_t)body[0] << 8) | body[1];
            size_t pos = 2U;
            if (pos + topic_len > rem_len) {
                break;
            }
            const char *topic = (const char *)(body + pos);
            pos += topic_len;
            const uint8_t qos = (flags >> 1) & 0x03U;
            if (qos > 0) {
                if (pos + 2U > rem_len) {
                    break;
                }
                pos += 2U; // Bỏ qua packet identifier
            }
            if (on_publish != nullptr) {
                on_publish(topic, topic_len, body + pos, rem_len - pos, ctx);
            }
            break;
        }
        case 9: // SUBACK
            if (rem_len >= 3U) {
                suback_seen_ = true;
                suback_pid_ = ((uint16_t)body[0] << 8) | body[1];
                suback_rc_ = body[2];
            }
            break;
        case 13: // PINGRESP
            pingresp_seen_ = true;
            break;
        default:
            break;
        }
    }
}

#endif // !ARDUINO || BOARD_TDM2421
