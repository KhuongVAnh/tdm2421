#pragma once

#include "C2BridgeWorker.h"
#include <QMainWindow>

class QCheckBox;
class QFrame;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QToolButton;

/**
 * Cửa sổ chính của ứng dụng TDM2421 C2 Bridge:
 *  - Cho phép nhập tài khoản MQTT, mật khẩu, mã Remote ID (và tùy chọn mở rộng Host/Port).
 *  - Tự động sinh và khóa cố định 2 topic TX/RX theo mã Remote ID.
 *  - Lưu/nạp lại cấu hình lần trước bằng QSettings.
 *  - Hiển thị trạng thái MQTT, QGC, tốc độ luồng dữ liệu MAVLink và nhật ký hoạt động.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    /// Bật hoặc tắt cầu nối C2 (Connect / Disconnect).
    void onToggleConnect();
    /// Cập nhật ngay 2 topic TX/RX mỗi khi người dùng gõ mã Remote ID.
    void onRemoteIdEdited(const QString &text);
    /// Ẩn/hiện khung Cài đặt nâng cao (Host, MQTT Port, TCP Port).
    void onToggleAdvanced(bool checked);
    /// Ẩn/hiện ký tự mật khẩu MQTT.
    void onTogglePasswordVisibility(bool checked);
    /// Ẩn/hiện khung Nhật ký hoạt động.
    void onToggleLogVisibility(bool checked);
    /// Cập nhật badge trạng thái MQTT và QGC khi worker phát tín hiệu đổi trạng thái.
    void onStatusChanged(MqttConnState mqttState, QgcConnState qgcState, const QString &detail);
    /// Cập nhật số liệu Hz, B/s, số frame MAVLink mỗi giây.
    void onStatsUpdated(const C2BridgeStats &stats);
    /// Ghi thêm 1 dòng log kèm dấu thời gian vào khung nhật ký.
    void onAppendLog(const QString &line);

private:
    void setupUi();
    void loadSettings();
    void saveSettings();
    void updateTopicLabels();
    void setInputsEditable(bool editable);
    void setConnectButtonVisual(bool connectedMode);

    C2BridgeWorker *worker_ = nullptr;

    // Nhóm cấu hình cơ bản
    QLineEdit *usernameEdit_ = nullptr;
    QLineEdit *passwordEdit_ = nullptr;
    QToolButton *showPassBtn_ = nullptr;
    QCheckBox *rememberPassCheck_ = nullptr;
    QLineEdit *remoteIdEdit_ = nullptr;

    // Nhãn hiển thị 2 topic tự động sinh theo Remote ID
    QLabel *txTopicValue_ = nullptr;
    QLabel *rxTopicValue_ = nullptr;

    // Nhóm Cài đặt nâng cao thu gọn
    QToolButton *advancedToggleBtn_ = nullptr;
    QFrame *advancedPanel_ = nullptr;
    QLineEdit *hostEdit_ = nullptr;
    QSpinBox *mqttPortSpin_ = nullptr;
    QSpinBox *tcpPortSpin_ = nullptr;

    QPushButton *connectBtn_ = nullptr;
    QLabel *qgcHintLabel_ = nullptr;

    // Nhóm trạng thái & thống kê
    QLabel *mqttStatusBadge_ = nullptr;
    QLabel *qgcStatusBadge_ = nullptr;
    QLabel *clientIdValue_ = nullptr;
    QLabel *dlRateValue_ = nullptr;
    QLabel *mavFrameValue_ = nullptr;
    QLabel *ulRateValue_ = nullptr;
    QLabel *maxGapValue_ = nullptr;

    // Nhóm nhật ký hoạt động
    QToolButton *logToggleBtn_ = nullptr;
    QPushButton *clearLogBtn_ = nullptr;
    QPlainTextEdit *logView_ = nullptr;
};
