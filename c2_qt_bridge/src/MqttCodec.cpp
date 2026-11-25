#include "MqttCodec.h"

namespace MqttCodec {

QByteArray encodeRemainingLength(uint32_t value)
{
    QByteArray out;
    out.reserve(4);
    do {
        // Lấy 7 bit thấp cho mỗi byte; nếu vẫn còn phần dư thì bật bit 7 (0x80) để báo tiếp tục.
        uint8_t encoded = static_cast<uint8_t>(value % 128U);
        value /= 128U;
        if (value > 0U) {
            encoded |= 0x80U;
        }
        out.append(static_cast<char>(encoded));
    } while (value > 0U);
    return out;
}

QByteArray encodeUtf8String(const QString &text)
{
    const QByteArray raw = text.toUtf8();
    const uint16_t len = static_cast<uint16_t>(qMin<qsizetype>(raw.size(), 65535));
    QByteArray out;
    out.reserve(2 + len);
    // Chuỗi MQTT luôn bắt đầu bằng 2 byte độ dài theo thứ tự Big-Endian (network byte order).
    out.append(static_cast<char>((len >> 8) & 0xFFU));
    out.append(static_cast<char>(len & 0xFFU));
    out.append(raw.constData(), len);
    return out;
}

QByteArray buildConnect(const QString &clientId,
                        const QString &username,
                        const QString &password,
                        uint16_t keepAliveSec)
{
    QByteArray body;
    // Variable Header của MQTT 3.1.1: Protocol Name "MQTT" (0x00 0x04 'M' 'Q' 'T' 'T') + Protocol Level 4.
    body.append("\x00\x04MQTT\x04", 7);

    // Connect Flags: luôn bật Clean Session (bit 1 = 0x02) để broker không giữ hàng đợi lệnh C2 cũ.
    uint8_t flags = 0x02U;
    if (!username.isEmpty()) {
        flags |= 0x80U; // Username flag
        if (!password.isEmpty()) {
            flags |= 0x40U; // Password flag (chỉ hợp lệ khi có Username theo chuẩn MQTT)
        }
    }
    body.append(static_cast<char>(flags));
    body.append(static_cast<char>((keepAliveSec >> 8) & 0xFFU));
    body.append(static_cast<char>(keepAliveSec & 0xFFU));

    // Payload của gói CONNECT: Client ID bắt buộc đứng đầu, tiếp theo là Username / Password nếu có.
    body.append(encodeUtf8String(clientId));
    if (!username.isEmpty()) {
        body.append(encodeUtf8String(username));
        if (!password.isEmpty()) {
            body.append(encodeUtf8String(password));
        }
    }

    QByteArray packet;
    packet.reserve(1 + 4 + body.size());
    packet.append(static_cast<char>(0x10)); // Packet Type = 1 (CONNECT), Flags = 0
    packet.append(encodeRemainingLength(static_cast<uint32_t>(body.size())));
    packet.append(body);
    return packet;
}

QByteArray buildSubscribe(uint16_t packetId, const QString &topic)
{
    QByteArray body;
    body.append(static_cast<char>((packetId >> 8) & 0xFFU));
    body.append(static_cast<char>(packetId & 0xFFU));
    body.append(encodeUtf8String(topic));
    body.append(static_cast<char>(0x00)); // Requested QoS = 0 (giảm độ trễ tối đa cho telemetry)

    QByteArray packet;
    packet.reserve(1 + 4 + body.size());
    packet.append(static_cast<char>(0x82)); // Packet Type = 8 (SUBSCRIBE), Reserved flags bắt buộc = 0x02
    packet.append(encodeRemainingLength(static_cast<uint32_t>(body.size())));
    packet.append(body);
    return packet;
}

QByteArray buildPublish(const QString &topic, const QByteArray &payload)
{
    const QByteArray encodedTopic = encodeUtf8String(topic);
    const uint32_t remLen = static_cast<uint32_t>(encodedTopic.size() + payload.size());

    QByteArray packet;
    packet.reserve(1 + 4 + static_cast<int>(remLen));
    // Fixed Header 0x30: Type = 3 (PUBLISH), DUP = 0, QoS = 0, RETAIN = 0 (tuyệt đối không retain lệnh điều khiển C2).
    packet.append(static_cast<char>(0x30));
    packet.append(encodeRemainingLength(remLen));
    packet.append(encodedTopic);
    packet.append(payload);
    return packet;
}

QByteArray buildPingReq()
{
    return QByteArray("\xC0\x00", 2);
}

QByteArray buildDisconnect()
{
    return QByteArray("\xE0\x00", 2);
}

QVector<MqttPacket> parsePackets(QByteArray &buffer, bool *malformed)
{
    if (malformed != nullptr) {
        *malformed = false;
    }
    QVector<MqttPacket> packets;

    int offset = 0;
    // Mỗi gói MQTT tối thiểu gồm 2 byte (1 byte header + >= 1 byte Remaining Length).
    while (buffer.size() - offset >= 2) {
        const uint8_t b0 = static_cast<uint8_t>(buffer.at(offset));
        uint32_t multiplier = 1U;
        uint32_t remLen = 0U;
        int hdrLen = 0;
        bool needMore = false;

        // Giải mã Remaining Length tối đa 4 byte theo chuẩn MQTT 3.1.1.
        for (int i = 1; i <= 4; ++i) {
            if (buffer.size() - offset <= i) {
                needMore = true; // Chưa nhận đủ byte độ dài từ TCP, đợi lần readyRead tiếp theo.
                break;
            }
            const uint8_t enc = static_cast<uint8_t>(buffer.at(offset + i));
            remLen += static_cast<uint32_t>(enc & 0x7FU) * multiplier;
            if ((enc & 0x80U) == 0U) {
                hdrLen = i + 1;
                break;
            }
            multiplier *= 128U;
        }

        if (needMore) {
            break;
        }
        // Quá 4 byte mà bit tiếp tục (0x80) vẫn bật => luồng MQTT bị lỗi định dạng.
        if (hdrLen == 0) {
            if (malformed != nullptr) {
                *malformed = true;
            }
            buffer.clear();
            return packets;
        }

        const qsizetype totalLen = static_cast<qsizetype>(hdrLen) + static_cast<qsizetype>(remLen);
        if (buffer.size() - offset < totalLen) {
            // Đã biết đủ kích thước gói nhưng payload chưa về đủ hết qua TCP => giữ lại trong buffer chờ thêm.
            break;
        }

        MqttPacket pkt;
        pkt.headerByte = b0;
        pkt.body = QByteArray(buffer.constData() + offset + hdrLen, static_cast<int>(remLen));
        packets.append(pkt);
        offset += static_cast<int>(totalLen);
    }

    if (offset > 0) {
        buffer.remove(0, offset);
    }
    return packets;
}

MqttPublishData parsePublishBody(uint8_t headerByte, const QByteArray &body)
{
    MqttPublishData out;
    if (body.size() < 2) {
        return out;
    }
    const uint16_t topicLen =
        (static_cast<uint16_t>(static_cast<uint8_t>(body.at(0))) << 8) |
        static_cast<uint16_t>(static_cast<uint8_t>(body.at(1)));
    int pos = 2 + static_cast<int>(topicLen);
    if (pos > body.size()) {
        return out;
    }

    out.topic = QString::fromUtf8(body.constData() + 2, topicLen);
    // Nếu broker gửi xuống ở mức QoS 1 hoặc QoS 2 thì sau tên topic còn có 2 byte Packet Identifier.
    const uint8_t qos = (headerByte >> 1) & 0x03U;
    if (qos > 0U) {
        pos += 2;
        if (pos > body.size()) {
            return out;
        }
    }
    out.payload = QByteArray(body.constData() + pos, body.size() - pos);
    out.valid = true;
    return out;
}

int countMavlinkFrames(const QByteArray &payload)
{
    // Duyệt nhanh chuỗi binary nối đuôi từ ESP32 để đếm số frame MAVLink phục vụ hiển thị thống kê (không chặn/sửa dữ liệu).
    int pos = 0;
    int frames = 0;
    const int n = payload.size();
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(payload.constData());

    while (pos < n) {
        const uint8_t stx = raw[pos];
        if (stx == 0xFD && pos + 12 <= n) {
            // MAVLink v2: 10 byte header + payload (plen) + 2 byte CRC + 13 byte signature nếu incompat_flags bit 0 bật.
            const int plen = raw[pos + 1];
            const uint8_t incompat = raw[pos + 2];
            const int totalLen = 10 + plen + 2 + ((incompat & 0x01U) ? 13 : 0);
            if (pos + totalLen > n) {
                break;
            }
            frames++;
            pos += totalLen;
        } else if (stx == 0xFE && pos + 8 <= n) {
            // MAVLink v1: 6 byte header + payload (plen) + 2 byte CRC.
            const int plen = raw[pos + 1];
            const int totalLen = 6 + plen + 2;
            if (pos + totalLen > n) {
                break;
            }
            frames++;
            pos += totalLen;
        } else {
            break;
        }
    }
    return frames;
}

} // namespace MqttCodec
