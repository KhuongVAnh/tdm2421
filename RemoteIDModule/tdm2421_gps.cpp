#include "tdm2421_gps.h"
#include "tdm2421_debug.h"

#if defined(BOARD_TDM2421) && TDM2421_USE_EXT_GPS

#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

TDM2421ExtGPS tdm2421_gps;

// Lệnh UBX-CFG-PRT: chuyển UART1 của module u-blox sang 115200 bps, 8N1, bật cả UBX + NMEA.
static const uint8_t UBX_SET_BAUD_115200[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
};

// Lệnh UBX-CFG-CFG: lưu cấu hình hiện tại vào BBR + Flash trên module GPS (mask 0xFFFF).
static const uint8_t UBX_SAVE_CONFIG[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB
};

static const char *GPS_NVS_NAMESPACE = "tdmgps";
static constexpr float KNOTS_TO_MPS = 0.514444f;
static constexpr uint32_t GGA_STALE_MS = 2500;
// Ngưỡng phát hiện mất kết nối NMEA: để 2500 ms để hỗ trợ an toàn cả chế độ 1 Hz (1000 ms/lần).
static constexpr uint32_t EXT_GPS_CONNECT_TIMEOUT_MS = 2500;

void TDM2421ExtGPS::sendUBX(uint8_t msg_class, uint8_t msg_id,
                            const uint8_t *payload, uint16_t len)
{
    // Tính checksum Fletcher-8 (CK_A, CK_B) từ Class, ID, Length đến hết Payload.
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    auto acc = [&](uint8_t b) {
        ck_a = (uint8_t)(ck_a + b);
        ck_b = (uint8_t)(ck_b + ck_a);
    };

    const uint8_t len_l = (uint8_t)(len & 0xFFU);
    const uint8_t len_h = (uint8_t)((len >> 8) & 0xFFU);

    Serial1.write(0xB5);
    Serial1.write(0x62);
    Serial1.write(msg_class); acc(msg_class);
    Serial1.write(msg_id);    acc(msg_id);
    Serial1.write(len_l);     acc(len_l);
    Serial1.write(len_h);     acc(len_h);

    for (uint16_t i = 0; i < len; i++) {
        Serial1.write(payload[i]);
        acc(payload[i]);
    }
    Serial1.write(ck_a);
    Serial1.write(ck_b);
}

void TDM2421ExtGPS::setNmeaMsgRate(uint8_t nmea_msg_id, uint8_t rate)
{
    // UBX-CFG-MSG (payload 3 byte) đặt tần suất xuất bản tin cho chính cổng UART đang nhận lệnh.
    // Nhóm NMEA chuẩn có Class = 0xF0: GGA=0x00, GLL=0x01, GSA=0x02, GSV=0x03, RMC=0x04, VTG=0x05.
    const uint8_t payload[3] = { 0xF0, nmea_msg_id, rate };
    sendUBX(0x06, 0x01, payload, sizeof(payload));
}

void TDM2421ExtGPS::applyUbxRateConfig(uint8_t hz)
{
    // Chu kỳ đo tính theo ms: 1 Hz -> 1000 ms, 5 Hz -> 200 ms, 10 Hz -> 100 ms.
    const uint16_t meas_ms = (hz == 10) ? 100U : ((hz == 5) ? 200U : 1000U);
    const uint8_t rate_payload[6] = {
        (uint8_t)(meas_ms & 0xFFU),
        (uint8_t)((meas_ms >> 8) & 0xFFU),
        0x01, 0x00, // navRate = 1 chu kỳ đo / 1 nghiệm định vị
        0x01, 0x00  // timeRef = GPS/UTC
    };
    sendUBX(0x06, 0x08, rate_payload, sizeof(rate_payload));
    Serial1.flush();
    delay(40);

    // Chỉ giữ GGA (độ cao/số vệ tinh/HDOP) và RMC (tọa độ/tốc độ/hướng/giờ UTC);
    // tắt GLL/GSA/GSV/VTG để giảm tải UART và CPU cho ESP32-C3 đơn nhân.
    setNmeaMsgRate(0x00, 1); // Bật GGA
    setNmeaMsgRate(0x04, 1); // Bật RMC
    setNmeaMsgRate(0x01, 0); // Tắt GLL
    setNmeaMsgRate(0x02, 0); // Tắt GSA
    setNmeaMsgRate(0x03, 0); // Tắt GSV
    setNmeaMsgRate(0x05, 0); // Tắt VTG
    Serial1.flush();
    delay(40);

    // Lưu cấu hình vào BBR/Flash của module GPS để lần cấp nguồn sau vẫn giữ nguyên.
    Serial1.write(UBX_SAVE_CONFIG, sizeof(UBX_SAVE_CONFIG));
    Serial1.flush();
    delay(40);
}

void TDM2421ExtGPS::begin()
{
    // Đọc tần số đã lưu trong NVS (mặc định 10 Hz nếu chưa từng cấu hình).
    Preferences prefs;
    if (prefs.begin(GPS_NVS_NAMESPACE, true)) {
        const uint8_t saved_hz = prefs.getUChar("hz", 10);
        prefs.end();
        if (isValidHz(saved_hz)) {
            configured_hz_ = saved_hz;
        }
    }

    // Tăng kích thước buffer nhận UART1 lên 1024 byte để tránh tràn FIFO khi phát 10 Hz
    // lúc Wi-Fi/BLE đang bận xử lý.
    Serial1.setRxBufferSize(1024);

    // Bước 1: Mở cổng ở baud mặc định 9600 bps để ép module GPS chuyển sang 115200 bps.
    // Nếu module đã được lưu sẵn ở 115200 bps từ lần chạy trước, các lệnh sau bước
    // updateBaudRate(115200) vẫn có hiệu lực bình thường.
    Serial1.begin(9600, SERIAL_8N1, PIN_GPS_UART_RX, PIN_GPS_UART_TX);
    delay(150);
    Serial1.write(UBX_SET_BAUD_115200, sizeof(UBX_SET_BAUD_115200));
    Serial1.flush();
    delay(100);

    // Bước 2: Chuyển tốc độ UART1 của ESP32 sang 115200 bps theo GPS.
    Serial1.updateBaudRate(GPS_UART_BAUD);
    delay(100);

    // Bước 3: Áp dụng tần số 1/5/10 Hz đang cấu hình và lưu xuống module GPS.
    applyUbxRateConfig(configured_hz_);

    // Xóa các byte rác còn sót trong lúc đổi baudrate.
    while (Serial1.available()) {
        (void)Serial1.read();
    }

    rate_win_start_ms_ = millis();
    tdmDebugPrintf(1, "[EXT GPS] Serial1 started at %u bps (RX=GPIO%d TX=GPIO%d, %u Hz)\r\n",
                  (unsigned)GPS_UART_BAUD, (int)PIN_GPS_UART_RX,
                  (int)PIN_GPS_UART_TX, (unsigned)configured_hz_);
}

bool TDM2421ExtGPS::setRateHz(uint8_t hz)
{
    if (!isValidHz(hz)) {
        return false;
    }

    configured_hz_ = hz;

    // Lưu cấu hình vào NVS ESP32 để giữ lại sau khi reboot.
    Preferences prefs;
    if (prefs.begin(GPS_NVS_NAMESPACE, false)) {
        prefs.putUChar("hz", configured_hz_);
        prefs.end();
    }

    // Gửi trực tiếp lệnh đổi tần số qua UART1 (nếu GPS đang ở baud 9600 chưa kịp đổi
    // thì cũng gửi thêm 1 nhịp kích baud 115200 trước khi áp tần số mới).
    applyUbxRateConfig(configured_hz_);
    test_gsv_enabled_ = false;

    // Reset cửa sổ đo tần số thực tế để Web UI phản ánh ngay nhịp mới.
    rate_win_start_ms_ = millis();
    rate_win_count_ = 0;
    measured_hz_ = 0;

    tdmDebugPrintf(1, "[EXT GPS] Updated GPS rate to %u Hz\r\n", (unsigned)configured_hz_);
    return true;
}

bool TDM2421ExtGPS::isConnected() const
{
    if (last_nmea_ms_ == 0) {
        return false;
    }
    return (uint32_t)(millis() - last_nmea_ms_) <= EXT_GPS_CONNECT_TIMEOUT_MS;
}

uint8_t TDM2421ExtGPS::getMeasuredHz() const
{
    if (!isConnected()) {
        return 0;
    }
    return measured_hz_;
}

bool TDM2421ExtGPS::verifyChecksum(const char *sentence)
{
    if (!sentence || sentence[0] != '$') {
        return false;
    }
    const char *star = strchr(sentence + 1, '*');
    if (!star || star[1] == '\0' || star[2] == '\0') {
        return false;
    }

    // Checksum NMEA là phép XOR tất cả ký tự nằm giữa '$' và '*'.
    uint8_t calc = 0;
    for (const char *p = sentence + 1; p < star; p++) {
        calc ^= (uint8_t)(*p);
    }

    char hex_buf[3] = { star[1], star[2], '\0' };
    char *endptr = nullptr;
    const unsigned long expected = strtoul(hex_buf, &endptr, 16);
    if (endptr != hex_buf + 2) {
        return false;
    }
    return calc == (uint8_t)expected;
}

size_t TDM2421ExtGPS::splitCsvFields(char *sentence, char *fields[], size_t max_fields)
{
    // Cắt bỏ phần *CS ở cuối câu trước khi tách cột.
    char *star = strchr(sentence, '*');
    if (star) {
        *star = '\0';
    }

    // Tách thủ công để giữ nguyên các ô trống (ví dụ ",,") — tuyệt đối không dùng strtok()
    // vì strtok() bỏ qua dấu phẩy liên tiếp làm lệch toàn bộ cột phía sau.
    size_t count = 0;
    char *p = sentence;
    while (count < max_fields) {
        fields[count++] = p;
        char *comma = strchr(p, ',');
        if (!comma) {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }
    return count;
}

bool TDM2421ExtGPS::parseNmeaCoord(const char *val_str, const char *hemi_str,
                                   bool is_lon, double &out_deg)
{
    // Đổi tọa độ chuẩn NMEA (ddmm.mmmm hoặc dddmm.mmmm) sang độ thập phân.
    if (!val_str || !val_str[0] || !hemi_str || !hemi_str[0]) {
        return false;
    }
    const double raw = atof(val_str);
    const int deg_part = (int)(raw / 100.0);
    const double min_part = raw - (double)deg_part * 100.0;
    double deg = (double)deg_part + (min_part / 60.0);

    const char h = hemi_str[0];
    if (h == 'S' || h == 'W') {
        deg = -deg;
    } else if (h != 'N' && h != 'E') {
        return false;
    }

    if (is_lon) {
        if (deg < -180.0 || deg > 180.0) return false;
    } else {
        if (deg < -90.0 || deg > 90.0) return false;
    }
    out_deg = deg;
    return true;
}

float TDM2421ExtGPS::parseUtcTimestampSec(const char *utc_str)
{
    // Chuỗi UTC NMEA dạng hhmmss hoặc hhmmss.ss -> OpenDroneID yêu cầu số giây tính
    // từ đầu giờ UTC hiện tại (nằm trong khoảng 0..3600).
    if (!utc_str || strlen(utc_str) < 6) {
        return 0.0f;
    }
    const int mm = (utc_str[2] - '0') * 10 + (utc_str[3] - '0');
    if (mm < 0 || mm > 59) {
        return 0.0f;
    }
    const float ss = (float)atof(utc_str + 4);
    if (ss < 0.0f || ss >= 61.0f) {
        return 0.0f;
    }
    float total = (float)mm * 60.0f + ss;
    if (total > 3600.0f) {
        total = 3600.0f;
    }
    return total;
}

void TDM2421ExtGPS::handleGGA(char *fields[], size_t nfields)
{
    // Định dạng $xxGGA,utc,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,...
    if (nfields < 10) {
        return;
    }
    const int quality = atoi(fields[6]);
    if (quality <= 0) {
        gga_valid_ = false;
        return;
    }

    const int sats = fields[7][0] ? atoi(fields[7]) : 0;
    const float hdop = fields[8][0] ? (float)atof(fields[8]) : 99.9f;
    float alt_m = fields[9][0] ? (float)atof(fields[9]) : 0.0f;
    // Giới hạn độ cao theo dải hợp lệ của OpenDroneID [-1000, 31767.5] m để check_parse() không báo lỗi.
    if (alt_m < -1000.0f) alt_m = -1000.0f;
    if (alt_m > 31767.5f) alt_m = 31767.5f;

    gga_alt_m_ = alt_m;
    gga_sats_ = (uint8_t)constrain(sats, 0, 255);
    gga_hdop_ = hdop;
    gga_valid_ = true;
    last_gga_ms_ = millis();

    // Nếu GGA đến ngay sau RMC trong cùng chu kỳ 10 Hz, cập nhật luôn độ cao/số vệ tinh.
    if (current_fix_.valid && last_fix_ms_ != 0 &&
        (uint32_t)(last_gga_ms_ - last_fix_ms_) < 500U) {
        current_fix_.altitude_m = gga_alt_m_;
        current_fix_.satellites = gga_sats_;
        current_fix_.hdop = gga_hdop_;
        current_fix_.fix_type = 3;
        tdm2421_mqtt.updateExternalGNSS(current_fix_);
    }
}

void TDM2421ExtGPS::handleGSV(char *fields[], size_t nfields)
{
    // Định dạng $xxGSV,totalMsgs,msgNum,satsInView,prn1,elev1,az1,snr1,...
    // Chỉ dùng trong lúc chạy nút Test module để xem số vệ tinh nhìn thấy & độ mạnh tín hiệu SNR (dB-Hz).
    if (!test_active_ || nfields < 4) {
        return;
    }
    const int sats_in_view = fields[3][0] ? atoi(fields[3]) : 0;
    if (sats_in_view > 0 && sats_in_view <= 255 && (uint8_t)sats_in_view > test_sats_in_view_) {
        test_sats_in_view_ = (uint8_t)sats_in_view;
    }

    // Mỗi bản tin GSV chứa tối đa 4 vệ tinh bắt đầu từ cột 4 (4 cột/vệ tinh, cột cuối là SNR).
    for (size_t i = 4; i + 3 < nfields && i <= 16; i += 4) {
        if (fields[i + 3][0] != '\0') {
            const int snr = atoi(fields[i + 3]);
            if (snr > 0 && snr <= 99 && (uint8_t)snr > test_max_snr_) {
                test_max_snr_ = (uint8_t)snr;
            }
        }
    }
}

void TDM2421ExtGPS::handleRMC(char *fields[], size_t nfields)
{
    // Định dạng $xxRMC,utc,status,lat,N/S,lon,E/W,sog_kn,cog_deg,date,...
    if (nfields < 10) {
        return;
    }
    // Trạng thái 'A' = dữ liệu hợp lệ (Active), 'V' = chưa bắt được vệ tinh (Void).
    if (fields[2][0] != 'A') {
        return;
    }

    double lat = 0.0;
    double lon = 0.0;
    if (!parseNmeaCoord(fields[3], fields[4], false, lat) ||
        !parseNmeaCoord(fields[5], fields[6], true, lon)) {
        return;
    }
    if (lat == 0.0 && lon == 0.0) {
        return;
    }

    // Chuyển tốc độ từ hải lý/giờ (knots) sang m/s và kẹp trong dải hợp lệ của OpenDroneID.
    float speed_mps = fields[7][0] ? ((float)atof(fields[7]) * KNOTS_TO_MPS) : 0.0f;
    if (speed_mps < 0.0f) speed_mps = 0.0f;
    if (speed_mps > 254.25f) speed_mps = 254.25f;

    // Khi đứng yên, trường COG có thể bị bỏ trống; chuẩn hóa về [0, 360) độ.
    const bool course_valid = (fields[8][0] != '\0');
    float course_deg = 0.0f;
    if (course_valid) {
        course_deg = fmodf((float)atof(fields[8]), 360.0f);
        if (course_deg < 0.0f) {
            course_deg += 360.0f;
        }
    }

    const uint32_t now_ms = millis();
    const bool gga_fresh = gga_valid_ && last_gga_ms_ != 0 &&
                           (uint32_t)(now_ms - last_gga_ms_) <= GGA_STALE_MS;

    TDM2421GNSSFix fix {};
    fix.valid = true;
    fix.latitude = lat;
    fix.longitude = lon;
    fix.altitude_m = gga_fresh ? gga_alt_m_ : 0.0f;
    fix.speed_mps = speed_mps;
    fix.course_deg = course_deg;
    fix.hdop = gga_fresh ? gga_hdop_ : 99.9f;
    fix.fix_type = gga_fresh ? 3 : 2;
    fix.satellites = gga_fresh ? gga_sats_ : 0;
    fix.course_valid = course_valid;
    fix.dop_valid = false;
    snprintf(fix.utc, sizeof(fix.utc), "%s", fields[1]);
    snprintf(fix.date, sizeof(fix.date), "%s", fields[9]);

    current_fix_ = fix;
    current_timestamp_s_ = parseUtcTimestampSec(fields[1]);
    last_fix_ms_ = now_ms;

    // Đẩy dữ liệu sang TDM2421MQTT để Web Dashboard (/api/status) hiển thị thông số GNSS.
    tdm2421_mqtt.updateExternalGNSS(current_fix_);

    // Chỉ in log debug không chặn ra USB mỗi 1 giây/lần để tránh làm nghẽn lúc không mở Serial Monitor.
    static uint32_t last_log_ms = 0;
    if (now_ms - last_log_ms >= 1000U) {
        last_log_ms = now_ms;
        tdmDebugPrintf(2, "[EXT GPS] fix=%uD sats=%u lat=%.7f lon=%.7f alt=%.1fm spd=%.2fm/s hdg=%.1f hdop=%.2f\r\n",
                      (unsigned)fix.fix_type, (unsigned)fix.satellites,
                      fix.latitude, fix.longitude, fix.altitude_m,
                      fix.speed_mps, fix.course_deg, fix.hdop);
    }
}

void TDM2421ExtGPS::processLine(char *line)
{
    if (!verifyChecksum(line)) {
        return;
    }

    const uint32_t now_ms = millis();
    // Ghi nhận có bản tin NMEA hợp lệ để biết module đã cắm (ngay cả khi trong nhà chưa fix).
    last_nmea_ms_ = now_ms;

    // Lưu câu NMEA nguyên bản vào bộ đệm test 1 giây trước khi splitCsvFields() chèn '\0'.
    if (test_active_ && test_samples_len_ < sizeof(test_samples_) - 2U) {
        const size_t len = strlen(line);
        const size_t avail = sizeof(test_samples_) - 1U - test_samples_len_;
        if (len + 1U <= avail) {
            memcpy(test_samples_ + test_samples_len_, line, len);
            test_samples_len_ += len;
            test_samples_[test_samples_len_++] = '\n';
            test_samples_[test_samples_len_] = '\0';
        }
    }

    char *fields[24] {};
    const size_t nfields = splitCsvFields(line, fields, 24);
    if (nfields == 0 || strlen(fields[0]) < 6) {
        return;
    }

    // Nhận diện theo hậu tố bản tin (bỏ qua 2 ký tự hệ vệ tinh GP/GN/GL/GA/GB).
    const char *msg_type = fields[0] + 3;
    if (strcmp(msg_type, "RMC") == 0) {
        // Đếm số bản tin RMC mỗi giây để hiển thị tần số thực tế của module (1/5/10 Hz).
        rate_win_count_++;
        if ((uint32_t)(now_ms - rate_win_start_ms_) >= 1000U) {
            const uint32_t dt = (uint32_t)(now_ms - rate_win_start_ms_);
            measured_hz_ = (uint8_t)((rate_win_count_ * 1000U + dt / 2U) / dt);
            rate_win_count_ = 0;
            rate_win_start_ms_ = now_ms;
        }
        if (test_active_) {
            test_rmc_count_++;
        }
        handleRMC(fields, nfields);
    } else if (strcmp(msg_type, "GGA") == 0) {
        if (test_active_) {
            test_gga_count_++;
        }
        handleGGA(fields, nfields);
    } else if (strcmp(msg_type, "GSV") == 0) {
        if (test_active_) {
            test_gsv_count_++;
        }
        handleGSV(fields, nfields);
    }
}

void TDM2421ExtGPS::finalizeTestIfElapsed()
{
    if (test_active_ && (uint32_t)(millis() - test_start_ms_) >= test_duration_ms_) {
        test_active_ = false;
        test_done_ = true;
        // Tắt lại bản tin GSV sau khi test xong để không tốn băng thông UART lúc chạy bình thường.
        if (test_gsv_enabled_) {
            test_gsv_enabled_ = false;
            setNmeaMsgRate(0x03, 0);
            Serial1.flush();
        }
    }
}

void TDM2421ExtGPS::startTestCapture(uint32_t duration_ms)
{
    // Bắt đầu cửa sổ thu dữ liệu 1 giây cho nút Test module trên Web UI.
    test_bytes_ = 0;
    test_rmc_count_ = 0;
    test_gga_count_ = 0;
    test_gsv_count_ = 0;
    test_sats_in_view_ = 0;
    test_max_snr_ = 0;
    test_samples_len_ = 0;
    test_samples_[0] = '\0';
    test_done_ = false;
    uint32_t eff_ms = duration_ms > 0 ? duration_ms : 1000U;
    // Nếu đang ở 1 Hz thì nới nhẹ lên 1300 ms để kịp hứng trọn ít nhất 1 nhịp GSV.
    if (configured_hz_ <= 1 && eff_ms < 1300U) {
        eff_ms = 1300U;
    }
    test_duration_ms_ = eff_ms;
    test_start_ms_ = millis();
    test_active_ = true;

    // Bật tạm bản tin GSV (chỉ trên RAM của GPS, không lưu flash) với nhịp ~2 Hz (ở 5/10 Hz)
    // để xem được số vệ tinh nhìn thấy & SNR kể cả khi chưa fix.
    const uint8_t gsv_rate = (configured_hz_ >= 10) ? 5U : ((configured_hz_ >= 5) ? 2U : 1U);
    setNmeaMsgRate(0x03, gsv_rate);
    Serial1.flush();
    test_gsv_enabled_ = true;
}

void TDM2421ExtGPS::update()
{
    finalizeTestIfElapsed();

    // Đọc hết các byte đang có trong Serial1 theo kiểu không chặn (non-blocking)
    // để không làm trễ chu kỳ phát Wi-Fi/BLE trong loop().
    while (Serial1.available()) {
        const char c = (char)Serial1.read();
        last_rx_ms_ = millis();
        if (test_active_) {
            test_bytes_++;
        }
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            if (line_len_ > 0) {
                line_buf_[line_len_] = '\0';
                processLine(line_buf_);
                line_len_ = 0;
                finalizeTestIfElapsed();
            }
            continue;
        }
        if (line_len_ == 0 && c != '$') {
            // Bỏ qua các byte phản hồi UBX nhị phân hoặc mảnh dòng dở dang cho tới ký tự '$'.
            continue;
        }
        if (line_len_ < sizeof(line_buf_) - 1) {
            line_buf_[line_len_++] = c;
        } else {
            // Bảo vệ tràn bộ đệm dòng: hủy dòng lỗi và chờ câu '$' tiếp theo.
            line_len_ = 0;
        }
    }

    // Nếu quá 1.5s không nhận được RMC mới thì reset đồng hồ đo tần số về 0.
    if (last_nmea_ms_ != 0 && (uint32_t)(millis() - last_nmea_ms_) > EXT_GPS_CONNECT_TIMEOUT_MS) {
        measured_hz_ = 0;
        rate_win_count_ = 0;
        rate_win_start_ms_ = millis();
    }

    finalizeTestIfElapsed();
}

bool TDM2421ExtGPS::getFix(TDM2421GNSSFix &out, float &timestamp_s, uint32_t max_age_ms) const
{
    if (!current_fix_.valid || last_fix_ms_ == 0) {
        return false;
    }
    const uint32_t age_ms = (uint32_t)(millis() - last_fix_ms_);
    if (age_ms > max_age_ms) {
        return false;
    }
    out = current_fix_;
    out.age_ms = age_ms;
    timestamp_s = current_timestamp_s_;
    return true;
}

String TDM2421ExtGPS::jsonEscape(const char *s)
{
    String out;
    if (!s) return out;
    out.reserve(strlen(s) + 16);
    for (const char *p = s; *p != '\0'; p++) {
        const char c = *p;
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

String TDM2421ExtGPS::statusJson() const
{
    const uint32_t now_ms = millis();
    const bool connected = isConnected();
    const uint32_t nmea_age = (last_nmea_ms_ == 0) ? 0U : (uint32_t)(now_ms - last_nmea_ms_);

    TDM2421GNSSFix fix {};
    float ts = 0.0f;
    const bool fix_valid = getFix(fix, ts, 2500);

    String s;
    s.reserve(384);
    s = "{";
    s += "\"connected\":" + String(connected ? "true" : "false") + ",";
    s += "\"nmeaAgeMs\":" + String(nmea_age) + ",";
    s += "\"configuredHz\":" + String((unsigned)configured_hz_) + ",";
    s += "\"measuredHz\":" + String((unsigned)getMeasuredHz()) + ",";
    s += "\"fixValid\":" + String(fix_valid ? "true" : "false") + ",";
    s += "\"fixType\":" + String((unsigned)(fix_valid ? fix.fix_type : 0)) + ",";
    s += "\"sats\":" + String((unsigned)(fix_valid ? fix.satellites : gga_sats_)) + ",";
    s += "\"hdop\":" + String(fix_valid ? fix.hdop : gga_hdop_, 2) + ",";
    s += "\"lat\":" + String(fix_valid ? fix.latitude : 0.0, 7) + ",";
    s += "\"lon\":" + String(fix_valid ? fix.longitude : 0.0, 7) + ",";
    s += "\"alt\":" + String(fix_valid ? fix.altitude_m : 0.0f, 1) + ",";
    s += "\"speed\":" + String(fix_valid ? fix.speed_mps : 0.0f, 2) + ",";
    s += "\"course\":" + String(fix_valid ? fix.course_deg : 0.0f, 1) + ",";
    s += "\"courseValid\":" + String((fix_valid && fix.course_valid) ? "true" : "false") + ",";
    s += "\"utc\":\"" + jsonEscape(fix_valid ? fix.utc : "") + "\",";
    s += "\"date\":\"" + jsonEscape(fix_valid ? fix.date : "") + "\"";
    s += "}";
    return s;
}

String TDM2421ExtGPS::testStatusJson()
{
    finalizeTestIfElapsed();

    TDM2421GNSSFix fix {};
    float ts = 0.0f;
    const bool fix_valid = getFix(fix, ts, 2500);
    const uint16_t measured_test_hz = (test_rmc_count_ > 0) ? test_rmc_count_ : test_gga_count_;

    String s;
    s.reserve(512 + test_samples_len_);
    s = "{";
    s += "\"active\":" + String(test_active_ ? "true" : "false") + ",";
    s += "\"done\":" + String(test_done_ ? "true" : "false") + ",";
    s += "\"durationMs\":" + String((unsigned)test_duration_ms_) + ",";
    s += "\"connected\":" + String(isConnected() ? "true" : "false") + ",";
    s += "\"configuredHz\":" + String((unsigned)configured_hz_) + ",";
    s += "\"byteCount\":" + String((unsigned)test_bytes_) + ",";
    s += "\"rmcCount\":" + String((unsigned)test_rmc_count_) + ",";
    s += "\"ggaCount\":" + String((unsigned)test_gga_count_) + ",";
    s += "\"gsvCount\":" + String((unsigned)test_gsv_count_) + ",";
    s += "\"satsInView\":" + String((unsigned)test_sats_in_view_) + ",";
    s += "\"maxSnr\":" + String((unsigned)test_max_snr_) + ",";
    s += "\"measuredHz\":" + String((unsigned)measured_test_hz) + ",";
    s += "\"fixValid\":" + String(fix_valid ? "true" : "false") + ",";
    s += "\"fixType\":" + String((unsigned)(fix_valid ? fix.fix_type : 0)) + ",";
    s += "\"sats\":" + String((unsigned)(fix_valid ? fix.satellites : gga_sats_)) + ",";
    s += "\"hdop\":" + String(fix_valid ? fix.hdop : gga_hdop_, 2) + ",";
    s += "\"lat\":" + String(fix_valid ? fix.latitude : 0.0, 7) + ",";
    s += "\"lon\":" + String(fix_valid ? fix.longitude : 0.0, 7) + ",";
    s += "\"alt\":" + String(fix_valid ? fix.altitude_m : 0.0f, 1) + ",";
    s += "\"speed\":" + String(fix_valid ? fix.speed_mps : 0.0f, 2) + ",";
    s += "\"course\":" + String(fix_valid ? fix.course_deg : 0.0f, 1) + ",";
    s += "\"utc\":\"" + jsonEscape(fix_valid ? fix.utc : "") + "\",";
    s += "\"samples\":\"" + jsonEscape(test_samples_) + "\"";
    s += "}";
    return s;
}

#endif // BOARD_TDM2421 && TDM2421_USE_EXT_GPS
