#include "C2BridgeWorker.h"
#include "MqttCodec.h"

#include <QAbstractSocket>
#include <QHostAddress>
#include <QRandomGenerator>
#include <QTcpServer>
#include <QTcpSocket>

C2BridgeWorker::C2BridgeWorker(QObject *parent)
    : QObject(parent)
{
    // Timer kiểm soát thời gian chờ phản hồi CONNACK/SUBACK khi mới kết nối TCP tới broker.
    handshakeTimer_ = new QTimer(this);
    handshakeTimer_->setSingleShot(true);
    connect(handshakeTimer_, &QTimer::timeout, this, &C2BridgeWorker::onHandshakeTimeout);

    // Timer gửi PINGREQ định kỳ mỗi 20s (nhỏ hơn keepalive 30s) để tránh bị broker đóng socket khi rảnh.
    pingTimer_ = new QTimer(this);
    pingTimer_->setInterval(PING_INTERVAL_MS);
    connect(pingTimer_, &QTimer::timeout, this, &C2BridgeWorker::onPingTimer);

    // Timer chờ 3s trước khi thử kết nối lại khi mất mạng hoặc broker ngắt kết nối.
    reconnectTimer_ = new QTimer(this);
    reconnectTimer_->setSingleShot(true);
    reconnectTimer_->setInterval(RECONNECT_DELAY_MS);
    connect(reconnectTimer_, &QTimer::timeout, this, &C2BridgeWorker::onReconnectTimer);

    // Timer tổng hợp tốc độ RX/TX, số frame MAVLink mỗi giây để đẩy lên giao diện.
    statsTimer_ = new QTimer(this);
    statsTimer_->setInterval(STATS_INTERVAL_MS);
    connect(statsTimer_, &QTimer::timeout, this, &C2BridgeWorker::onStatsTimer);
}

C2BridgeWorker::~C2BridgeWorker()
{
    stop();
}

QString C2BridgeWorker::makeClientId(const QString &remoteId)
{
    // Lọc bỏ ký tự đặc biệt nếu có và ghép thêm 4 ký tự hex ngẫu nhiên để nhiều người
    // cùng kết nối vào 1 broker không bị broker đá phiên của nhau vì trùng Client ID.
    QString clean;
    clean.reserve(remoteId.size());
    for (const QChar ch : remoteId) {
        if (ch.isLetterOrNumber() || ch == '_' || ch == '-') {
            clean.append(ch);
        }
    }
    if (clean.isEmpty()) {
        clean = QStringLiteral("UNKNOWN");
    }
    const uint32_t suffix = QRandomGenerator::global()->generate() & 0xFFFFU;
    return QStringLiteral("TDM2421_QGC_%1_%2")
        .arg(clean)
        .arg(suffix, 4, 16, QLatin1Char('0'));
}

QString C2BridgeWorker::connackReason(uint8_t rc)
{
    switch (rc) {
    case 1: return QStringLiteral("Phiên bản giao thức MQTT không được hỗ trợ (rc=1)");
    case 2: return QStringLiteral("Client ID bị từ chối (rc=2)");
    case 3: return QStringLiteral("MQTT Broker hiện không khả dụng (rc=3)");
    case 4: return QStringLiteral("Sai tài khoản hoặc mật khẩu MQTT (rc=4)");
    case 5: return QStringLiteral("Tài khoản không có quyền kết nối (rc=5)");
    default: return QStringLiteral("Broker từ chối kết nối (rc=%1)").arg(rc);
    }
}

void C2BridgeWorker::start(const C2BridgeConfig &config)
{
    stop();

    config_ = config;
    config_.mqttHost = config_.mqttHost.trimmed();
    config_.username = config_.username.trimmed();
    config_.remoteId = config_.remoteId.trimmed();

    txTopic_ = config_.txTopic();
    rxTopic_ = config_.rxTopic();
    clientId_ = makeClientId(config_.remoteId);

    // Reset toàn bộ bộ đếm cho phiên mới.
    winDlPackets_ = 0;
    winDlBytes_ = 0;
    winDlFrames_ = 0;
    winMaxGapMs_ = 0.0;
    totalDlFrames_ = 0;
    totalDlBytes_ = 0;
    winUlPackets_ = 0;
    winUlBytes_ = 0;
    totalUlBytes_ = 0;
    dlGapTimer_.invalidate();

    // Mở TCP Server trước để QGC có thể kết nối ngay (127.0.0.1:<tcpPort>).
    tcpServer_ = new QTcpServer(this);
    connect(tcpServer_, &QTcpServer::newConnection, this, &C2BridgeWorker::onNewQgcConnection);
    if (!tcpServer_->listen(QHostAddress::AnyIPv4, config_.tcpPort)) {
        const QString err = tcpServer_->errorString();
        emit logMessage(QStringLiteral("[LỖI TCP] Không mở được cổng %1 cho QGC: %2")
                            .arg(config_.tcpPort)
                            .arg(err));
        tcpServer_->deleteLater();
        tcpServer_ = nullptr;
        mqttState_ = MqttConnState::Disconnected;
        qgcState_ = QgcConnState::Stopped;
        emitStatus(QStringLiteral("Không mở được cổng TCP %1: %2").arg(config_.tcpPort).arg(err));
        return;
    }

    running_ = true;
    qgcState_ = QgcConnState::Waiting;
    qgcPeer_.clear();
    emit logMessage(QStringLiteral("[TCP] Đang lắng nghe QGC tại 127.0.0.1:%1").arg(config_.tcpPort));
    emit logMessage(QStringLiteral("[MQTT] Client ID: %1 | Sub: %2 | Pub: %3")
                        .arg(clientId_, txTopic_, rxTopic_));

    statsWindowTimer_.start();
    statsTimer_->start();
    connectMqtt();
}

void C2BridgeWorker::stop()
{
    const bool wasRunning = running_;
    running_ = false;
    subscribed_ = false;
    awaitingPingResp_ = false;

    handshakeTimer_->stop();
    pingTimer_->stop();
    reconnectTimer_->stop();
    statsTimer_->stop();

    // Gửi gói DISCONNECT gọn gàng nếu socket MQTT vẫn đang mở.
    if (mqttSocket_ != nullptr && mqttSocket_->state() == QAbstractSocket::ConnectedState) {
        mqttSocket_->write(MqttCodec::buildDisconnect());
        mqttSocket_->flush();
    }
    closeMqttSocket();
    closeQgcSocket();

    if (tcpServer_ != nullptr) {
        tcpServer_->close();
        tcpServer_->deleteLater();
        tcpServer_ = nullptr;
    }

    // Giữ nguyên trạng thái AuthError nếu vừa bị dừng do sai tài khoản/mật khẩu để UI hiển thị cảnh báo.
    if (mqttState_ != MqttConnState::AuthError) {
        mqttState_ = MqttConnState::Disconnected;
    }
    qgcState_ = QgcConnState::Stopped;
    qgcPeer_.clear();

    if (wasRunning) {
        emit logMessage(QStringLiteral("[HỆ THỐNG] Đã ngắt kết nối cầu nối C2."));
    }
    emitStatus();
    onStatsTimer();
}

void C2BridgeWorker::connectMqtt()
{
    if (!running_) {
        return;
    }
    closeMqttSocket();
    subscribed_ = false;
    awaitingPingResp_ = false;
    mqttRxBuffer_.clear();

    mqttSocket_ = new QTcpSocket(this);
    connect(mqttSocket_, &QTcpSocket::connected, this, &C2BridgeWorker::onMqttConnected);
    connect(mqttSocket_, &QTcpSocket::readyRead, this, &C2BridgeWorker::onMqttReadyRead);
    connect(mqttSocket_, &QTcpSocket::disconnected, this, &C2BridgeWorker::onMqttDisconnected);
    connect(mqttSocket_, &QTcpSocket::errorOccurred, this, &C2BridgeWorker::onMqttError);

    mqttState_ = MqttConnState::Connecting;
    emitStatus(QStringLiteral("Đang kết nối %1:%2...").arg(config_.mqttHost).arg(config_.mqttPort));
    emit logMessage(QStringLiteral("[MQTT] Đang kết nối %1:%2 ...").arg(config_.mqttHost).arg(config_.mqttPort));

    handshakeTimer_->start(HANDSHAKE_TIMEOUT_MS);
    mqttSocket_->connectToHost(config_.mqttHost, config_.mqttPort);
}

void C2BridgeWorker::closeMqttSocket()
{
    if (mqttSocket_ == nullptr) {
        return;
    }
    // Ngắt toàn bộ signal trước khi abort để tránh onMqttDisconnected/onMqttError kích hoạt lại reconnect chồng chéo.
    mqttSocket_->disconnect(this);
    mqttSocket_->abort();
    mqttSocket_->deleteLater();
    mqttSocket_ = nullptr;
}

void C2BridgeWorker::closeQgcSocket()
{
    if (qgcSocket_ == nullptr) {
        return;
    }
    qgcSocket_->disconnect(this);
    qgcSocket_->abort();
    qgcSocket_->deleteLater();
    qgcSocket_ = nullptr;
    qgcPeer_.clear();
}

void C2BridgeWorker::scheduleReconnect(const QString &reason)
{
    if (!running_) {
        return;
    }
    handshakeTimer_->stop();
    pingTimer_->stop();
    subscribed_ = false;
    awaitingPingResp_ = false;
    closeMqttSocket();

    mqttState_ = MqttConnState::Reconnecting;
    emitStatus(reason);
    emit logMessage(QStringLiteral("[CẢNH BÁO] %1 — thử kết nối lại sau 3 giây...").arg(reason));
    if (!reconnectTimer_->isActive()) {
        reconnectTimer_->start();
    }
}

void C2BridgeWorker::emitStatus(const QString &detail)
{
    emit statusChanged(mqttState_, qgcState_, detail);
}

void C2BridgeWorker::onNewQgcConnection()
{
    while (tcpServer_ != nullptr && tcpServer_->hasPendingConnections()) {
        QTcpSocket *client = tcpServer_->nextPendingConnection();
        if (client == nullptr) {
            continue;
        }
        // Bật TCP_NODELAY (LowDelayOption) để tránh Nagle algorithm gây trễ gói MAVLink nhỏ.
        client->setSocketOption(QAbstractSocket::LowDelayOption, 1);

        // Nếu QGC mở lại kết nối mới thì đóng kết nối cũ (giống logic scripts/c2_mqtt_tcp_bridge.py).
        if (qgcSocket_ != nullptr) {
            emit logMessage(QStringLiteral("[TCP] Thay thế kết nối QGC cũ (%1)").arg(qgcPeer_));
            closeQgcSocket();
        }

        qgcSocket_ = client;
        qgcPeer_ = QStringLiteral("%1:%2").arg(client->peerAddress().toString()).arg(client->peerPort());
        qgcState_ = QgcConnState::Connected;

        connect(qgcSocket_, &QTcpSocket::readyRead, this, &C2BridgeWorker::onQgcReadyRead);
        connect(qgcSocket_, &QTcpSocket::disconnected, this, &C2BridgeWorker::onQgcDisconnected);

        emit logMessage(QStringLiteral("[TCP] QGroundControl đã kết nối từ %1").arg(qgcPeer_));
        emitStatus();
    }
}

void C2BridgeWorker::onQgcReadyRead()
{
    if (qgcSocket_ == nullptr) {
        return;
    }
    const QByteArray data = qgcSocket_->readAll();
    if (data.isEmpty() || !subscribed_ || mqttSocket_ == nullptr ||
        mqttSocket_->state() != QAbstractSocket::ConnectedState) {
        return;
    }

    // Chiều Uplink (QGC -> Broker -> ESP32 -> FC): chia nhỏ từng khúc <= 1024 byte rồi PUBLISH lên topic RX.
    for (int off = 0; off < data.size(); off += MAX_UPLINK_CHUNK) {
        const QByteArray chunk = data.mid(off, MAX_UPLINK_CHUNK);
        mqttSocket_->write(MqttCodec::buildPublish(rxTopic_, chunk));
        winUlPackets_++;
        winUlBytes_ += static_cast<uint64_t>(chunk.size());
        totalUlBytes_ += static_cast<uint64_t>(chunk.size());
    }
}

void C2BridgeWorker::onQgcDisconnected()
{
    emit logMessage(QStringLiteral("[TCP] QGroundControl đã ngắt kết nối (%1)").arg(qgcPeer_));
    closeQgcSocket();
    qgcState_ = running_ ? QgcConnState::Waiting : QgcConnState::Stopped;
    emitStatus();
}

void C2BridgeWorker::onMqttConnected()
{
    if (!running_ || mqttSocket_ == nullptr) {
        return;
    }
    // Bật TCP_NODELAY + TCP KeepAlive sau khi TCP handshake hoàn tất, rồi gửi bản tin MQTT CONNECT.
    mqttSocket_->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    mqttSocket_->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
    mqttSocket_->write(MqttCodec::buildConnect(clientId_, config_.username, config_.password, KEEPALIVE_SEC));
}

void C2BridgeWorker::onMqttReadyRead()
{
    if (!running_ || mqttSocket_ == nullptr) {
        return;
    }
    mqttRxBuffer_.append(mqttSocket_->readAll());

    bool malformed = false;
    const QVector<MqttPacket> packets = MqttCodec::parsePackets(mqttRxBuffer_, &malformed);
    if (malformed) {
        scheduleReconnect(QStringLiteral("Gói tin MQTT sai định dạng"));
        return;
    }

    for (const MqttPacket &pkt : packets) {
        const uint8_t ptype = pkt.packetType();
        if (ptype == 2) { // CONNACK: Broker phản hồi kết quả đăng nhập.
            if (pkt.body.size() < 2) {
                scheduleReconnect(QStringLiteral("CONNACK không hợp lệ"));
                return;
            }
            const uint8_t rc = static_cast<uint8_t>(pkt.body.at(1));
            if (rc != 0) {
                const QString reason = connackReason(rc);
                emit logMessage(QStringLiteral("[LỖI MQTT] %1").arg(reason));
                if (rc == 4 || rc == 5) {
                    // Nếu sai tài khoản/mật khẩu hoặc không có quyền -> dừng hẳn để người dùng sửa lại.
                    mqttState_ = MqttConnState::AuthError;
                    emitStatus(reason);
                    stop();
                } else {
                    scheduleReconnect(reason);
                }
                return;
            }
            // Đăng nhập thành công -> gửi SUBSCRIBE topic TX (nhận dữ liệu từ FC/ESP32).
            mqttSocket_->write(MqttCodec::buildSubscribe(1, txTopic_));
        } else if (ptype == 9) { // SUBACK: Broker xác nhận đã subscribe topic TX thành công.
            handshakeTimer_->stop();
            subscribed_ = true;
            awaitingPingResp_ = false;
            mqttState_ = MqttConnState::Connected;
            pingTimer_->start();
            emitStatus();
            emit logMessage(QStringLiteral("[MQTT] Đã kết nối & subscribe thành công: %1").arg(txTopic_));
        } else if (ptype == 3) { // PUBLISH: Nhận dữ liệu binary MAVLink từ ESP32/FC.
            const MqttPublishData pub = MqttCodec::parsePublishBody(pkt.headerByte, pkt.body);
            if (pub.valid && pub.topic == txTopic_ && !pub.payload.isEmpty()) {
                // Đo khoảng trễ tối đa giữa 2 bản tin MQTT downlink liên tiếp.
                if (dlGapTimer_.isValid()) {
                    const double gapMs = static_cast<double>(dlGapTimer_.elapsed());
                    if (gapMs > winMaxGapMs_) {
                        winMaxGapMs_ = gapMs;
                    }
                }
                dlGapTimer_.restart();

                const int frames = MqttCodec::countMavlinkFrames(pub.payload);
                winDlPackets_++;
                winDlBytes_ += static_cast<uint64_t>(pub.payload.size());
                winDlFrames_ += static_cast<uint64_t>(frames);
                totalDlFrames_ += static_cast<uint64_t>(frames);
                totalDlBytes_ += static_cast<uint64_t>(pub.payload.size());

                // Đẩy nguyên luồng binary MAVLink sang QGroundControl qua TCP socket cục bộ.
                if (qgcSocket_ != nullptr && qgcSocket_->state() == QAbstractSocket::ConnectedState) {
                    qgcSocket_->write(pub.payload);
                }
            }
        } else if (ptype == 13) { // PINGRESP
            awaitingPingResp_ = false;
        }
    }
}

void C2BridgeWorker::onMqttDisconnected()
{
    if (!running_) {
        return;
    }
    scheduleReconnect(QStringLiteral("MQTT Broker đã đóng kết nối"));
}

void C2BridgeWorker::onMqttError()
{
    if (!running_ || mqttSocket_ == nullptr) {
        return;
    }
    scheduleReconnect(QStringLiteral("Lỗi kết nối MQTT: %1").arg(mqttSocket_->errorString()));
}

void C2BridgeWorker::onHandshakeTimeout()
{
    if (!running_ || subscribed_) {
        return;
    }
    scheduleReconnect(QStringLiteral("Quá thời gian bắt tay MQTT (CONNACK/SUBACK timeout)"));
}

void C2BridgeWorker::onPingTimer()
{
    if (!running_ || !subscribed_ || mqttSocket_ == nullptr) {
        return;
    }
    // Nếu tới chu kỳ ping mới mà PINGREQ trước đó chưa có PINGRESP -> coi như đứt kết nối.
    if (awaitingPingResp_) {
        scheduleReconnect(QStringLiteral("Mất phản hồi PINGRESP từ MQTT Broker"));
        return;
    }
    awaitingPingResp_ = true;
    mqttSocket_->write(MqttCodec::buildPingReq());
}

void C2BridgeWorker::onReconnectTimer()
{
    if (running_) {
        connectMqtt();
    }
}

void C2BridgeWorker::onStatsTimer()
{
    C2BridgeStats st;
    st.mqttState = mqttState_;
    st.qgcState = qgcState_;
    st.qgcPeer = qgcPeer_;
    st.clientId = clientId_;
    st.totalDlFrames = totalDlFrames_;
    st.totalDlBytes = totalDlBytes_;
    st.totalUlBytes = totalUlBytes_;

    const double dtSec = statsWindowTimer_.isValid()
        ? (static_cast<double>(statsWindowTimer_.restart()) / 1000.0)
        : 1.0;
    if (running_ && dtSec > 0.05) {
        st.dlHz = static_cast<double>(winDlPackets_) / dtSec;
        st.dlBytesPerSec = static_cast<double>(winDlBytes_) / dtSec;
        st.dlFramesPerSec = static_cast<double>(winDlFrames_) / dtSec;
        st.dlMaxGapMs = winMaxGapMs_;
        st.ulHz = static_cast<double>(winUlPackets_) / dtSec;
        st.ulBytesPerSec = static_cast<double>(winUlBytes_) / dtSec;
    }

    winDlPackets_ = 0;
    winDlBytes_ = 0;
    winDlFrames_ = 0;
    winMaxGapMs_ = 0.0;
    winUlPackets_ = 0;
    winUlBytes_ = 0;

    emit statsUpdated(st);
}
