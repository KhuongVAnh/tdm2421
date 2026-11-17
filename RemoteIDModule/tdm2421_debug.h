#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

#if defined(BOARD_TDM2421)

#ifndef TDM2421_DEBUG_LEVEL
#define TDM2421_DEBUG_LEVEL 1
#endif

#if TDM2421_DEBUG_LEVEL < 0 || TDM2421_DEBUG_LEVEL > 2
#error "TDM2421_DEBUG_LEVEL must be 0 (production), 1 (1 Hz status), or 2 (verbose)"
#endif

// Ghi log USB CDC không chặn (non-blocking) cho TDM2421:
// Khi chạy độc lập bằng nguồn ngoài (không có PC mở /dev/ttyACM0),
// nếu kiểm tra thấy cổng USB chưa sẵn sàng hoặc hết chỗ trống trong buffer thì bỏ qua
// dòng log ngay lập tức để không làm treo task gửi MQTT 10-30 Hz.
static inline bool tdmDebugReady(size_t need = 1)
{
#if TDM2421_DEBUG_LEVEL == 0
    (void)need;
    return false;
#else
    if (!Serial) return false;
    const int avail = Serial.availableForWrite();
    if (avail <= 0) return false;
    if (need > 0 && (size_t)avail < need) return false;
    return true;
#endif
}

static inline void tdmDebugPrintf(int level, const char *fmt, ...)
{
#if TDM2421_DEBUG_LEVEL > 0
    if (level > TDM2421_DEBUG_LEVEL || !fmt) return;

    char buf[384];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;

    size_t len = (size_t)n;
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    if (!tdmDebugReady(len)) return;
    Serial.write((const uint8_t *)buf, len);
#else
    (void)level; (void)fmt;
#endif
}

static inline void tdmDebugPrintln(int level, const char *s)
{
#if TDM2421_DEBUG_LEVEL > 0
    if (level > TDM2421_DEBUG_LEVEL || !s) return;
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", s);
    if (n <= 0) return;
    size_t len = (size_t)n;
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    if (!tdmDebugReady(len)) return;
    Serial.write((const uint8_t *)buf, len);
#else
    (void)level; (void)s;
#endif
}

#else

// Giữ nguyên hành vi in log gốc trên các board không phải BOARD_TDM2421.
static inline void tdmDebugPrintf(int level, const char *fmt, ...)
{
    (void)level;
    if (!fmt) return;
    char buf[384];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    size_t len = (size_t)n;
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    Serial.write((const uint8_t *)buf, len);
}

static inline void tdmDebugPrintln(int level, const char *s)
{
    (void)level;
    if (s) Serial.println(s);
}

#endif // BOARD_TDM2421
