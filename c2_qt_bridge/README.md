# TDM2421 C2 Bridge (Qt 6)

Ứng dụng giao diện Qt 6 thay thế cho `scripts/c2_mqtt_tcp_bridge.py`, tạo cầu nối 2 chiều giữa **MQTT Broker** (raw binary MAVLink) và **QGroundControl** (TCP Server `127.0.0.1:5760`).

## Tính năng chính

- Nhập **Tài khoản MQTT**, **Mật khẩu MQTT** và **Mã Remote ID** rồi nhấn **Kết nối**.
- Tự động cố định 2 topic theo mã Remote ID:
  - Nhận từ FC (Subscribe): `p3000m/c2/<RemoteID>/tx`
  - Gửi xuống FC (Publish): `p3000m/c2/<RemoteID>/rx`
- Mục **Cài đặt nâng cao** cho phép chỉnh `MQTT Host` (mặc định `171.251.89.98`), `MQTT Port` (`8903`), `TCP Port` (`5760`).
- Tự lưu cấu hình lần cuối qua `QSettings` (tùy chọn `Nhớ mật khẩu`).
- Tự sinh `Client ID` duy nhất dạng `TDM2421_QGC_<RemoteID>_<4hex>` để tránh xung đột khi nhiều máy cùng kết nối broker.
- Hiển thị trạng thái MQTT / QGC, tốc độ RX/TX (`Hz`, `B/s`), số frame MAVLink và nhật ký hoạt động.

## Build & Chạy (Ubuntu / Debian với Qt 6)

```bash
cd c2_qt_bridge
./build.sh
./build/tdm2421_c2_bridge
```

## Cấu hình trên QGroundControl

Vào **Application Settings → Comm Links → Add**:
- **Type**: `TCP`
- **Server Address**: `127.0.0.1`
- **Port**: `5760` (hoặc cổng TCP đã chỉnh trong mục Nâng cao)
