#include "tdm2421_gnss.h"

#if !defined(ARDUINO) || defined(BOARD_TDM2421)

#include <string.h>

TDM2421GnssStore tdm2421_gnss;

void TDM2421GnssStore::begin()
{
#if defined(ARDUINO)
    if (mutex_ == nullptr) {
        mutex_ = xSemaphoreCreateMutex();
    }
#endif
}

bool TDM2421GnssStore::lock(uint32_t wait_ms) const
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

void TDM2421GnssStore::unlock() const
{
#if defined(ARDUINO)
    if (mutex_ != nullptr) {
        xSemaphoreGive(mutex_);
    }
#endif
}

void TDM2421GnssStore::updateFc(const TDM2421GnssRecord &rec)
{
    if (!rec.valid) {
        return;
    }
    // Không block loop(): timeout 2 ms.
    if (!lock(2)) {
        return;
    }
    fc_slot_ = rec;
    fc_slot_.valid = true;
    unlock();
}

void TDM2421GnssStore::updateEg800k(const TDM2421GnssRecord &rec)
{
    if (!rec.valid) {
        return;
    }
    if (!lock(20)) {
        return;
    }
    TDM2421GnssRecord merged = rec;
    // Giữ lại thông số DOP mới từ GSA (nếu vừa lấy trong 15 giây gần đây) khi QGPSLOC chưa mang DOP đầy đủ.
    if (!merged.dop_valid && eg_slot_.dop_valid &&
        (uint32_t)(rec.updated_ms - eg_last_dop_ms_) < 15000U) {
        merged.pdop = eg_slot_.pdop;
        merged.hdop = eg_slot_.hdop;
        merged.vdop = eg_slot_.vdop;
        merged.satellites_used = eg_slot_.satellites_used;
        merged.dop_valid = true;
    }
    eg_slot_ = merged;
    eg_slot_.valid = true;
    unlock();
}

void TDM2421GnssStore::updateEg800kDop(float pdop, float hdop, float vdop, uint8_t used, uint32_t now_ms)
{
    if (!lock(20)) {
        return;
    }
    eg_slot_.pdop = pdop;
    eg_slot_.hdop = hdop;
    eg_slot_.vdop = vdop;
    eg_slot_.satellites_used = used;
    eg_slot_.dop_valid = true;
    eg_last_dop_ms_ = now_ms;
    unlock();
}

bool TDM2421GnssStore::getFc(TDM2421GnssRecord &out, uint32_t max_age_ms, uint32_t now_ms) const
{
    memset(&out, 0, sizeof(out));
    if (!lock(5)) {
        return false;
    }
    out = fc_slot_;
    unlock();

    if (!out.valid || out.updated_ms == 0) {
        return false;
    }
    out.age_ms = (uint32_t)(now_ms - out.updated_ms);
    if (out.age_ms > max_age_ms) {
        out.valid = false;
        return false;
    }
    return true;
}

bool TDM2421GnssStore::getEg800k(TDM2421GnssRecord &out, uint32_t max_age_ms, uint32_t now_ms) const
{
    memset(&out, 0, sizeof(out));
    if (!lock(20)) {
        return false;
    }
    out = eg_slot_;
    unlock();

    if (!out.valid || out.updated_ms == 0) {
        return false;
    }
    out.age_ms = (uint32_t)(now_ms - out.updated_ms);
    if (out.age_ms > max_age_ms) {
        out.valid = false;
        return false;
    }
    return true;
}

TDM2421GnssSource TDM2421GnssStore::selectActive(TDM2421GnssRecord &out, uint32_t now_ms,
                                                 uint32_t fc_max_age_ms,
                                                 uint32_t eg_max_age_ms) const
{
    // Ưu tiên 1: Gói tọa độ từ FC (MAVLink).
    if (getFc(out, fc_max_age_ms, now_ms)) {
        return TDM2421GnssSource::FC_MAVLINK;
    }
    // Ưu tiên 2: GNSS gắn liền trên EG800K (dự phòng khi mất FC/mất GPS FC).
    if (getEg800k(out, eg_max_age_ms, now_ms)) {
        return TDM2421GnssSource::EG800K_GNSS;
    }
    memset(&out, 0, sizeof(out));
    return TDM2421GnssSource::NONE;
}

#endif // !ARDUINO || BOARD_TDM2421
