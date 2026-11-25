#pragma once

#include <QElapsedTimer>
#include <QObject>
#include <QString>
#include <QTimer>
#include <cstdint>

class QTcpServer;
class QTcpSocket;

/**
 * Thông số cấu hình phiên kết nối của cầu nối C2.
 * Hai topic MQTT luôn được suy ra tự động từ remoteId để tránh nhập sai.
 */
struct C2BridgeConfig {
    QString mqttHost = QStringLiteral("171.251.89.98");
    uint16_t mqttPort = 8903;
    uint16_t tcpPort = 5760;
    QString username;
    QString password;
    QString remoteId;

    /// Topic ESP32 publish từ FC lên -> App Qt subscribe để đẩy sang QGC.
    QString txTopic() const { return QStringLiteral("p3000m/c2/%1/tx").arg(remoteId); }
    /// Topic App Qt publish lệnh từ QGC lên -> ESP32 subscribe để đẩy xuống FC.
    QString rxTopic() const { return QStringLiteral("p3000m/c2/%1/rx").arg(remoteId); }
};

/// Trạng thái phiên kết nối tới MQTT Broker.
enum class MqttConnState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    AuthError,
};

/// Trạng thái TCP Server lắng nghe QGroundControl ở máy cục bộ.
enum class QgcConnState {
    Stopped,
    Waiting,
    Connected,
};

/// Ảnh chụp số liệu thống kê mỗi giây để cập nhật lên giao diện MainWindow.
struct C2BridgeStats {
    MqttConnState mqttState = MqttConnState::Disconnected;
    QgcConnState qgcState = QgcConnState::Stopped;
    QString qgcPeer;
    QString clientId;

    double dlHz = 0.0;
    double dlBytesPerSec = 0.0;
    double dlFramesPerSec = 0.0;
    double dlMaxGapMs = 0.0;
    uint64_t totalDlFrames = 0;
    uint64_t totalDlBytes = 0;

    double ulHz = 0.0;
    double ulBytesPerSec = 0.0;
    uint64_t totalUlBytes = 0;
};

/**
 * Thành phần lõi chạy bất đồng bộ trên event-loop Qt (không block UI):
 *  - Mở QTcpServer (mặc định 127.0.0.1:5760) chờ QGroundControl kết nối vào.
 *  - Mở QTcpSocket tới MQTT Broker, thực hiện CONNECT -> SUBSCRIBE topic TX của Remote ID.
 *  - Chuyển tiếp 2 chiều raw binary MAVLink giữa QGC và MQTT Broker.
 *  - Tự động giữ kết nối (PINGREQ 20s) và tự reconnect sau 3s nếu rớt mạng.
 */
class C2BridgeWorker : public QObject {
    Q_OBJECT

public:
    explicit C2BridgeWorker(QObject *parent = nullptr);
    ~C2BridgeWorker() override;

    bool isRunning() const { return running_; }

public slots:
    /// Khởi động TCP Server cho QGC và bắt đầu kết nối tới MQTT Broker theo cấu hình đã nhập.
    void start(const C2BridgeConfig &config);
    /// Ngắt phiên MQTT (gửi DISCONNECT nếu đang mở) và đóng TCP Server.
    void stop();

signals:
    void statusChanged(MqttConnState mqttState, QgcConnState qgcState, const QString &detail);
    void statsUpdated(const C2BridgeStats &stats);
    void logMessage(const QString &line);

private slots:
    void onNewQgcConnection();
    void onQgcReadyRead();
    void onQgcDisconnected();

    void onMqttConnected();
    void onMqttReadyRead();
    void onMqttDisconnected();
    void onMqttError();

    void onHandshakeTimeout();
    void onPingTimer();
    void onReconnectTimer();
    void onStatsTimer();

private:
    void connectMqtt();
    void closeMqttSocket();
    void closeQgcSocket();
    void scheduleReconnect(const QString &reason);
    void emitStatus(const QString &detail = QString());

    static QString makeClientId(const QString &remoteId);
    static QString connackReason(uint8_t rc);

    // Giới hạn kích thước mỗi gói PUBLISH uplink <= 1024 B để luôn lọt vừa buffer RX của modem EG800K trên ESP32.
    static constexpr int MAX_UPLINK_CHUNK = 1024;
    static constexpr int KEEPALIVE_SEC = 30;
    static constexpr int PING_INTERVAL_MS = 20000;
    static constexpr int HANDSHAKE_TIMEOUT_MS = 6000;
    static constexpr int RECONNECT_DELAY_MS = 3000;
    static constexpr int STATS_INTERVAL_MS = 1000;

    C2BridgeConfig config_;
    QString txTopic_;
    QString rxTopic_;
    QString clientId_;
    bool running_ = false;
    bool subscribed_ = false;
    bool awaitingPingResp_ = false;

    MqttConnState mqttState_ = MqttConnState::Disconnected;
    QgcConnState qgcState_ = QgcConnState::Stopped;
    QString qgcPeer_;

    QTcpServer *tcpServer_ = nullptr;
    QTcpSocket *qgcSocket_ = nullptr;
    QTcpSocket *mqttSocket_ = nullptr;
    QByteArray mqttRxBuffer_;

    QTimer *handshakeTimer_ = nullptr;
    QTimer *pingTimer_ = nullptr;
    QTimer *reconnectTimer_ = nullptr;
    QTimer *statsTimer_ = nullptr;
    QElapsedTimer dlGapTimer_;
    QElapsedTimer statsWindowTimer_;

    uint64_t winDlPackets_ = 0;
    uint64_t winDlBytes_ = 0;
    uint64_t winDlFrames_ = 0;
    double winMaxGapMs_ = 0.0;
    uint64_t totalDlFrames_ = 0;
    uint64_t totalDlBytes_ = 0;

    uint64_t winUlPackets_ = 0;
    uint64_t winUlBytes_ = 0;
    uint64_t totalUlBytes_ = 0;
};
