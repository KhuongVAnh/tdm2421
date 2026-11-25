#pragma once

#include <QByteArray>
#include <QString>
#include <QVector>
#include <cstdint>

/**
 * Đại diện cho 1 gói tin MQTT 3.1.1 đã được tách hoàn chỉnh từ luồng TCP.
 * - headerByte: byte đầu tiên của Fixed Header (4 bit cao = Packet Type, 4 bit thấp = Flags).
 * - body: phần dữ liệu sau Remaining Length (bao gồm Variable Header + Payload).
 */
struct MqttPacket {
    uint8_t headerByte = 0;
    QByteArray body;

    /// Loại gói MQTT (1=CONNECT, 2=CONNACK, 3=PUBLISH, 8=SUBSCRIBE, 9=SUBACK, 12=PINGREQ, 13=PINGRESP...).
    uint8_t packetType() const { return (headerByte >> 4) & 0x0F; }
    /// Mức QoS của gói PUBLISH (bit 2..1 của headerByte).
    uint8_t qos() const { return (headerByte >> 1) & 0x03; }
};

/**
 * Kết quả giải mã phần thân (body) của một gói MQTT PUBLISH.
 */
struct MqttPublishData {
    QString topic;
    QByteArray payload;
    bool valid = false;
};

/**
 * Bộ mã hóa / giải mã giao thức MQTT 3.1.1 thuần trên byte stream TCP
 * và bộ đếm frame MAVLink (không cần cài thêm thư viện QtMqtt ngoài).
 */
namespace MqttCodec {

/// Mã hóa trường Remaining Length theo chuẩn biến dài 1..4 byte của MQTT 3.1.1 (7 bit/byte, bit MSB báo còn byte sau).
QByteArray encodeRemainingLength(uint32_t value);

/// Mã hóa chuỗi UTF-8 kèm tiền tố 2 byte độ dài big-endian theo chuẩn MQTT.
QByteArray encodeUtf8String(const QString &text);

/// Đóng gói bản tin MQTT CONNECT (MQTT 3.1.1, Clean Session = 1, hỗ trợ user/password).
QByteArray buildConnect(const QString &clientId,
                        const QString &username,
                        const QString &password,
                        uint16_t keepAliveSec = 30);

/// Đóng gói bản tin MQTT SUBSCRIBE (QoS 0) để đăng ký nhận topic downlink từ drone.
QByteArray buildSubscribe(uint16_t packetId, const QString &topic);

/// Đóng gói bản tin MQTT PUBLISH (QoS 0, retain = 0 để không lưu lại lệnh C2 cũ trên broker).
QByteArray buildPublish(const QString &topic, const QByteArray &payload);

/// Đóng gói bản tin MQTT PINGREQ giữ kết nối sống với broker.
QByteArray buildPingReq();

/// Đóng gói bản tin MQTT DISCONNECT để đóng phiên gọn gàng khi người dùng bấm Ngắt kết nối.
QByteArray buildDisconnect();

/// Tách toàn bộ gói MQTT hoàn chỉnh từ buffer tích lũy (xử lý cả trường hợp gói bị cắt lẻ hoặc dính nhiều gói);
/// chỉ xóa phần byte đã tách thành công khỏi buffer. Đặt *malformed = true nếu Remaining Length vượt quá 4 byte.
QVector<MqttPacket> parsePackets(QByteArray &buffer, bool *malformed = nullptr);

/// Tách tên topic và payload nhị phân từ thân một gói PUBLISH (packetType == 3).
MqttPublishData parsePublishBody(uint8_t headerByte, const QByteArray &body);

/// Đếm số frame MAVLink v1 (0xFE) và v2 (0xFD, có tính chữ ký 13 byte nếu incompat bit 0 = 1) nối đuôi nhau trong 1 payload.
int countMavlinkFrames(const QByteArray &payload);

} // namespace MqttCodec
