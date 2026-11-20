#include "tdm2421_c2.h"

#if !defined(ARDUINO) || defined(BOARD_TDM2421)

#include <string.h>

TDM2421C2Bridge tdm2421_c2;

void TDM2421C2Bridge::begin()
{
#if defined(ARDUINO)
    if (mutex_ == nullptr) {
        mutex_ = xSemaphoreCreateMutex();
    }
#endif
}

bool TDM2421C2Bridge::lock(uint32_t wait_ms) const
{
#if defined(ARDUINO)
    if (mutex_ == nullptr) {
        return true;
    }
    return xSemaphoreTake(mutex_, pdMS_TO_TICKS(wait_ms)) == pdTRUE;
#else
    (void)wait_ms;
    return true;
#endif
}

void TDM2421C2Bridge::unlock() const
{
#if defined(ARDUINO)
    if (mutex_ != nullptr) {
        xSemaphoreGive(mutex_);
    }
#endif
}

void TDM2421C2Bridge::setEnabled(bool en)
{
    if (!lock(20)) {
        enabled_ = en;
        return;
    }
    enabled_ = en;
    if (!en) {
        dl_bufs_[0].used = 0;
        dl_bufs_[0].read_pos = 0;
        dl_bufs_[1].used = 0;
        dl_bufs_[1].read_pos = 0;
        ul_head_ = 0;
        ul_tail_ = 0;
        ul_count_ = 0;
    }
    unlock();
}

bool TDM2421C2Bridge::isEnabled() const
{
    return enabled_;
}

bool TDM2421C2Bridge::pushDownlinkFrame(const uint8_t *frame, uint16_t len, uint32_t now_ms)
{
    if (!enabled_ || frame == nullptr || len == 0 || len > TDM2421_C2_MAX_FRAME_LEN) {
        return false;
    }
    // Không block loop(): timeout 1 ms; chỉ thao tác trên Fill Buffer hiện tại.
    if (!lock(1)) {
        tx_dropped_++;
        return false;
    }
    if (!enabled_) {
        unlock();
        return false;
    }

    DownlinkBuffer &fb = dl_bufs_[fill_idx_];
    const size_t needed = 2U + (size_t)len;
    if (TDM2421_C2_DOWNLINK_BUF_SIZE - fb.used < needed) {
        tx_dropped_++;
        unlock();
        return false;
    }

    // Ghi 2 byte độ dài little-endian rồi ghi nguyên frame MAVLink nối tiếp vào Fill Buffer.
    fb.data[fb.used] = (uint8_t)(len & 0xFFU);
    fb.data[fb.used + 1U] = (uint8_t)((len >> 8) & 0xFFU);
    memcpy(fb.data + fb.used + 2U, frame, len);
    fb.used += needed;

    tx_frames_++;
    tx_bytes_ += len;
    last_tx_ms_ = now_ms;
    unlock();
    return true;
}

size_t TDM2421C2Bridge::swapDownlinkForSend()
{
    if (!lock(10)) {
        return 0;
    }
    DownlinkBuffer &sb = dl_bufs_[send_idx_];
    const size_t send_remaining = (sb.used > sb.read_pos) ? (sb.used - sb.read_pos) : 0U;
    // Nếu Send Buffer cũ vẫn còn dữ liệu chưa xả hết từ tick trước thì xả nốt trước khi swap,
    // đảm bảo thứ tự thời gian và không mất dữ liệu đang gửi dở.
    if (send_remaining > 0U) {
        unlock();
        return send_remaining;
    }

    // Send Buffer đã trống: reset sạch và hoán đổi vai trò với Fill Buffer nếu Fill Buffer có dữ liệu.
    sb.used = 0;
    sb.read_pos = 0;
    DownlinkBuffer &fb = dl_bufs_[fill_idx_];
    if (fb.used == 0U) {
        unlock();
        return 0U;
    }

    const uint8_t old_fill = fill_idx_;
    fill_idx_ = send_idx_;
    send_idx_ = old_fill;
    dl_bufs_[send_idx_].read_pos = 0U;
    const size_t ready = dl_bufs_[send_idx_].used;
    unlock();
    return ready;
}

size_t TDM2421C2Bridge::downlinkBytes() const
{
    if (!lock(5)) {
        return 0;
    }
    const size_t fill_n = dl_bufs_[fill_idx_].used;
    const DownlinkBuffer &sb = dl_bufs_[send_idx_];
    const size_t send_n = (sb.used > sb.read_pos) ? (sb.used - sb.read_pos) : 0U;
    unlock();
    return fill_n + send_n;
}

size_t TDM2421C2Bridge::popSendBatch(uint8_t *out, size_t max_bytes, uint16_t *out_frames)
{
    if (out_frames != nullptr) {
        *out_frames = 0;
    }
    if (out == nullptr || max_bytes == 0) {
        return 0;
    }

    // Lấy chỉ số Send Buffer hiện tại; chỉ có task MQTT đọc Send Buffer và send_idx_
    // chỉ đổi khi Send Buffer đã trống hoàn toàn nên không cần giữ mutex suốt lúc copy.
    if (!lock(10)) {
        return 0;
    }
    DownlinkBuffer &sb = dl_bufs_[send_idx_];
    unlock();

    size_t total = 0;
    uint16_t frames = 0;
    while (sb.read_pos + 2U <= sb.used) {
        const uint16_t frame_len =
            (uint16_t)sb.data[sb.read_pos] | ((uint16_t)sb.data[sb.read_pos + 1U] << 8);
        if (frame_len == 0 || frame_len > TDM2421_C2_MAX_FRAME_LEN ||
            sb.read_pos + 2U + frame_len > sb.used) {
            // Dữ liệu mất đồng bộ; dọn sạch Send Buffer để khôi phục an toàn.
            sb.read_pos = 0;
            sb.used = 0;
            break;
        }
        if (total + frame_len > max_bytes) {
            break;
        }
        memcpy(out + total, sb.data + sb.read_pos + 2U, frame_len);
        sb.read_pos += (2U + frame_len);
        total += frame_len;
        frames++;
    }

    if (sb.read_pos >= sb.used) {
        sb.read_pos = 0;
        sb.used = 0;
    }

    if (out_frames != nullptr) {
        *out_frames = frames;
    }
    return total;
}

void TDM2421C2Bridge::clearDownlinkBacklog()
{
    if (!lock(20)) {
        return;
    }
    dl_bufs_[0].used = 0;
    dl_bufs_[0].read_pos = 0;
    dl_bufs_[1].used = 0;
    dl_bufs_[1].read_pos = 0;
    unlock();
}

bool TDM2421C2Bridge::pushUplinkFrame(const uint8_t *frame, uint16_t len, uint32_t now_ms)
{
    if (!enabled_ || frame == nullptr || len == 0 || len > TDM2421_C2_MAX_FRAME_LEN) {
        return false;
    }
    if (!lock(10)) {
        rx_dropped_++;
        return false;
    }
    if (!enabled_) {
        unlock();
        return false;
    }
    if (ul_count_ >= TDM2421_C2_UPLINK_SLOTS) {
        rx_dropped_++;
        unlock();
        return false;
    }
    ul_slots_[ul_tail_].len = len;
    memcpy(ul_slots_[ul_tail_].data, frame, len);
    ul_tail_ = (ul_tail_ + 1U) % TDM2421_C2_UPLINK_SLOTS;
    ul_count_++;
    rx_frames_++;
    rx_bytes_ += len;
    last_rx_ms_ = now_ms;
    unlock();
    return true;
}

bool TDM2421C2Bridge::popUplinkFrame(uint8_t *out, uint16_t &out_len)
{
    out_len = 0;
    if (out == nullptr) {
        return false;
    }
    // Không block loop(): timeout 1 ms.
    if (!lock(1)) {
        return false;
    }
    if (ul_count_ == 0) {
        unlock();
        return false;
    }
    const UplinkSlot &slot = ul_slots_[ul_head_];
    out_len = slot.len;
    memcpy(out, slot.data, slot.len);
    ul_head_ = (ul_head_ + 1U) % TDM2421_C2_UPLINK_SLOTS;
    ul_count_--;
    unlock();
    return true;
}

void TDM2421C2Bridge::clearUplinkQueue()
{
    if (!lock(20)) {
        return;
    }
    ul_head_ = 0;
    ul_tail_ = 0;
    ul_count_ = 0;
    unlock();
}

void TDM2421C2Bridge::recordRxInvalid()
{
    if (!lock(5)) {
        rx_invalid_++;
        return;
    }
    rx_invalid_++;
    unlock();
}

TDM2421C2Stats TDM2421C2Bridge::getStats() const
{
    TDM2421C2Stats st {};
    if (!lock(10)) {
        st.enabled = enabled_;
        st.txFrames = tx_frames_;
        st.txBytes = tx_bytes_;
        st.txDropped = tx_dropped_;
        st.rxFrames = rx_frames_;
        st.rxBytes = rx_bytes_;
        st.rxDropped = rx_dropped_;
        st.rxInvalid = rx_invalid_;
        st.lastTxMs = last_tx_ms_;
        st.lastRxMs = last_rx_ms_;
        st.downlinkPendingBytes = 0;
        st.uplinkPendingFrames = ul_count_;
        return st;
    }
    const DownlinkBuffer &fb = dl_bufs_[fill_idx_];
    const DownlinkBuffer &sb = dl_bufs_[send_idx_];
    st.enabled = enabled_;
    st.txFrames = tx_frames_;
    st.txBytes = tx_bytes_;
    st.txDropped = tx_dropped_;
    st.rxFrames = rx_frames_;
    st.rxBytes = rx_bytes_;
    st.rxDropped = rx_dropped_;
    st.rxInvalid = rx_invalid_;
    st.lastTxMs = last_tx_ms_;
    st.lastRxMs = last_rx_ms_;
    st.downlinkPendingBytes = fb.used + ((sb.used > sb.read_pos) ? (sb.used - sb.read_pos) : 0U);
    st.uplinkPendingFrames = ul_count_;
    unlock();
    return st;
}

#if defined(ARDUINO)
String TDM2421C2Bridge::statusJson(bool subscribed) const
{
    const TDM2421C2Stats st = getStats();
    const uint32_t now_ms = millis();
    const uint32_t tx_age = st.lastTxMs == 0 ? 0 : (uint32_t)(now_ms - st.lastTxMs);
    const uint32_t rx_age = st.lastRxMs == 0 ? 0 : (uint32_t)(now_ms - st.lastRxMs);

    String s = "{";
    s += "\"enabled\":" + String(st.enabled ? "true" : "false") + ",";
    s += "\"subscribed\":" + String(subscribed ? "true" : "false") + ",";
    s += "\"txFrames\":" + String((unsigned long)st.txFrames) + ",";
    s += "\"txBytes\":" + String((unsigned long)st.txBytes) + ",";
    s += "\"txDropped\":" + String((unsigned long)st.txDropped) + ",";
    s += "\"rxFrames\":" + String((unsigned long)st.rxFrames) + ",";
    s += "\"rxBytes\":" + String((unsigned long)st.rxBytes) + ",";
    s += "\"rxDropped\":" + String((unsigned long)st.rxDropped) + ",";
    s += "\"rxInvalid\":" + String((unsigned long)st.rxInvalid) + ",";
    s += "\"lastTxAgeMs\":" + String((unsigned long)tx_age) + ",";
    s += "\"lastRxAgeMs\":" + String((unsigned long)rx_age) + ",";
    s += "\"downlinkPendingBytes\":" + String((unsigned long)st.downlinkPendingBytes) + ",";
    s += "\"uplinkPendingFrames\":" + String((unsigned long)st.uplinkPendingFrames);
    s += "}";
    return s;
}
#endif

#endif // !ARDUINO || BOARD_TDM2421
