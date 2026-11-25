#include "MainWindow.h"

#include <QCheckBox>
#include <QDateTime>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>

namespace {

/// Tạo CSS cho nhãn trạng thái dạng badge bo góc.
QString badgeStyle(const QString &bg, const QString &fg)
{
    return QStringLiteral(
        "QLabel { background-color: %1; color: %2; font-weight: 600; "
        "padding: 4px 10px; border-radius: 6px; }")
        .arg(bg, fg);
}

QString monoLabelStyle()
{
    return QStringLiteral("QLabel { font-family: monospace; color: #1e293b; }");
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    worker_ = new C2BridgeWorker(this);

    setupUi();
    loadSettings();
    updateTopicLabels();

    // Kết nối các signal từ C2BridgeWorker sang slot cập nhật giao diện.
    connect(worker_, &C2BridgeWorker::statusChanged, this, &MainWindow::onStatusChanged);
    connect(worker_, &C2BridgeWorker::statsUpdated, this, &MainWindow::onStatsUpdated);
    connect(worker_, &C2BridgeWorker::logMessage, this, &MainWindow::onAppendLog);
}

MainWindow::~MainWindow()
{
    saveSettings();
    worker_->stop();
}

void MainWindow::setupUi()
{
    setWindowTitle(QStringLiteral("TDM2421 C2 Bridge (MQTT ↔ QGroundControl)"));
    setMinimumWidth(560);

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(12);

    // --- Thẻ 1: Cấu hình kết nối ---
    auto *cfgGroup = new QGroupBox(QStringLiteral("Cấu hình kết nối C2"), central);
    auto *cfgLayout = new QVBoxLayout(cfgGroup);
    cfgLayout->setSpacing(10);

    auto *form = new QFormLayout();
    form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    form->setSpacing(8);

    usernameEdit_ = new QLineEdit(cfgGroup);
    usernameEdit_->setPlaceholderText(QStringLiteral("Ví dụ: pilot"));
    form->addRow(QStringLiteral("Tài khoản MQTT:"), usernameEdit_);

    auto *passRow = new QHBoxLayout();
    passRow->setSpacing(6);
    passwordEdit_ = new QLineEdit(cfgGroup);
    passwordEdit_->setEchoMode(QLineEdit::Password);
    passwordEdit_->setPlaceholderText(QStringLiteral("Nhập mật khẩu MQTT"));
    showPassBtn_ = new QToolButton(cfgGroup);
    showPassBtn_->setText(QStringLiteral("Hiện"));
    showPassBtn_->setCheckable(true);
    connect(showPassBtn_, &QToolButton::toggled, this, &MainWindow::onTogglePasswordVisibility);
    passRow->addWidget(passwordEdit_, 1);
    passRow->addWidget(showPassBtn_);
    form->addRow(QStringLiteral("Mật khẩu MQTT:"), passRow);

    rememberPassCheck_ = new QCheckBox(QStringLiteral("Nhớ mật khẩu cho lần mở sau"), cfgGroup);
    form->addRow(QString(), rememberPassCheck_);

    remoteIdEdit_ = new QLineEdit(cfgGroup);
    remoteIdEdit_->setMaxLength(20);
    remoteIdEdit_->setPlaceholderText(QStringLiteral("Ví dụ: 1866ASDD0000VT5"));
    connect(remoteIdEdit_, &QLineEdit::textChanged, this, &MainWindow::onRemoteIdEdited);
    form->addRow(QStringLiteral("Mã Remote ID:"), remoteIdEdit_);

    // Hai topic tự động cố định theo Remote ID, chỉ hiển thị để người dùng kiểm tra chứ không cho sửa tay.
    txTopicValue_ = new QLabel(cfgGroup);
    txTopicValue_->setStyleSheet(monoLabelStyle());
    txTopicValue_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    form->addRow(QStringLiteral("Topic TX (nhận FC):"), txTopicValue_);

    rxTopicValue_ = new QLabel(cfgGroup);
    rxTopicValue_->setStyleSheet(monoLabelStyle());
    rxTopicValue_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    form->addRow(QStringLiteral("Topic RX (gửi FC):"), rxTopicValue_);

    cfgLayout->addLayout(form);

    // Mục Cài đặt nâng cao thu gọn để giao diện mặc định đơn giản cho người dùng phổ thông.
    advancedToggleBtn_ = new QToolButton(cfgGroup);
    advancedToggleBtn_->setText(QStringLiteral("▶ Cài đặt nâng cao (Host, MQTT Port, TCP Port)"));
    advancedToggleBtn_->setCheckable(true);
    advancedToggleBtn_->setChecked(false);
    advancedToggleBtn_->setToolButtonStyle(Qt::ToolButtonTextOnly);
    advancedToggleBtn_->setStyleSheet(
        QStringLiteral("QToolButton { border: none; color: #0f4c81; font-weight: 600; text-align: left; }"));
    connect(advancedToggleBtn_, &QToolButton::toggled, this, &MainWindow::onToggleAdvanced);
    cfgLayout->addWidget(advancedToggleBtn_);

    advancedPanel_ = new QFrame(cfgGroup);
    advancedPanel_->setFrameShape(QFrame::StyledPanel);
    advancedPanel_->setVisible(false);
    auto *advForm = new QFormLayout(advancedPanel_);
    advForm->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    advForm->setContentsMargins(10, 10, 10, 10);
    advForm->setSpacing(8);

    hostEdit_ = new QLineEdit(advancedPanel_);
    hostEdit_->setPlaceholderText(QStringLiteral("171.251.89.98"));
    advForm->addRow(QStringLiteral("MQTT Host / IP:"), hostEdit_);

    mqttPortSpin_ = new QSpinBox(advancedPanel_);
    mqttPortSpin_->setRange(1, 65535);
    advForm->addRow(QStringLiteral("MQTT Port:"), mqttPortSpin_);

    tcpPortSpin_ = new QSpinBox(advancedPanel_);
    tcpPortSpin_->setRange(1, 65535);
    // Khi đổi TCP port thì cập nhật ngay dòng gợi ý bên dưới.
    connect(tcpPortSpin_, &QSpinBox::valueChanged, this, [this](int) { updateTopicLabels(); });
    advForm->addRow(QStringLiteral("TCP Port cho QGC:"), tcpPortSpin_);

    cfgLayout->addWidget(advancedPanel_);

    // Nút Kết nối / Ngắt kết nối + dòng hướng dẫn kết nối QGroundControl.
    auto *actionRow = new QHBoxLayout();
    connectBtn_ = new QPushButton(cfgGroup);
    connectBtn_->setMinimumHeight(38);
    setConnectButtonVisual(false);
    connect(connectBtn_, &QPushButton::clicked, this, &MainWindow::onToggleConnect);

    qgcHintLabel_ = new QLabel(cfgGroup);
    qgcHintLabel_->setWordWrap(true);
    qgcHintLabel_->setStyleSheet(QStringLiteral("QLabel { color: #475569; font-size: 12px; }"));
    actionRow->addWidget(connectBtn_);
    actionRow->addWidget(qgcHintLabel_, 1);
    cfgLayout->addLayout(actionRow);

    root->addWidget(cfgGroup);

    // --- Thẻ 2: Trạng thái & Thống kê luồng C2 ---
    auto *statGroup = new QGroupBox(QStringLiteral("Trạng thái & Thống kê luồng C2"), central);
    auto *statGrid = new QGridLayout(statGroup);
    statGrid->setSpacing(8);

    mqttStatusBadge_ = new QLabel(QStringLiteral("Chưa kết nối"), statGroup);
    mqttStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#e2e8f0"), QStringLiteral("#334155")));

    qgcStatusBadge_ = new QLabel(QStringLiteral("Chưa chạy"), statGroup);
    qgcStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#e2e8f0"), QStringLiteral("#334155")));

    clientIdValue_ = new QLabel(QStringLiteral("-"), statGroup);
    clientIdValue_->setStyleSheet(monoLabelStyle());
    dlRateValue_ = new QLabel(QStringLiteral("0.0 Hz | 0 B/s"), statGroup);
    dlRateValue_->setStyleSheet(monoLabelStyle());
    mavFrameValue_ = new QLabel(QStringLiteral("0.0 frame/s (Tổng: 0)"), statGroup);
    mavFrameValue_->setStyleSheet(monoLabelStyle());
    ulRateValue_ = new QLabel(QStringLiteral("0.0 Hz | 0 B/s (Tổng: 0 B)"), statGroup);
    ulRateValue_->setStyleSheet(monoLabelStyle());
    maxGapValue_ = new QLabel(QStringLiteral("0 ms"), statGroup);
    maxGapValue_->setStyleSheet(monoLabelStyle());

    int row = 0;
    statGrid->addWidget(new QLabel(QStringLiteral("MQTT Broker:"), statGroup), row, 0);
    statGrid->addWidget(mqttStatusBadge_, row, 1);
    statGrid->addWidget(new QLabel(QStringLiteral("QGroundControl:"), statGroup), row, 2);
    statGrid->addWidget(qgcStatusBadge_, row, 3);

    row++;
    statGrid->addWidget(new QLabel(QStringLiteral("Client ID:"), statGroup), row, 0);
    statGrid->addWidget(clientIdValue_, row, 1, 1, 3);

    row++;
    statGrid->addWidget(new QLabel(QStringLiteral("Downlink (FC→QGC):"), statGroup), row, 0);
    statGrid->addWidget(dlRateValue_, row, 1);
    statGrid->addWidget(new QLabel(QStringLiteral("Frame MAVLink:"), statGroup), row, 2);
    statGrid->addWidget(mavFrameValue_, row, 3);

    row++;
    statGrid->addWidget(new QLabel(QStringLiteral("Uplink (QGC→FC):"), statGroup), row, 0);
    statGrid->addWidget(ulRateValue_, row, 1);
    statGrid->addWidget(new QLabel(QStringLiteral("Max Gap RX:"), statGroup), row, 2);
    statGrid->addWidget(maxGapValue_, row, 3);

    statGrid->setColumnStretch(1, 1);
    statGrid->setColumnStretch(3, 1);
    root->addWidget(statGroup);

    // --- Thẻ 3: Nhật ký hoạt động (có nút ẩn/hiện và xóa log) ---
    auto *logHeader = new QHBoxLayout();
    logToggleBtn_ = new QToolButton(central);
    logToggleBtn_->setText(QStringLiteral("▼ Nhật ký hoạt động"));
    logToggleBtn_->setCheckable(true);
    logToggleBtn_->setChecked(true);
    logToggleBtn_->setToolButtonStyle(Qt::ToolButtonTextOnly);
    logToggleBtn_->setStyleSheet(
        QStringLiteral("QToolButton { border: none; color: #334155; font-weight: 600; }"));
    connect(logToggleBtn_, &QToolButton::toggled, this, &MainWindow::onToggleLogVisibility);

    clearLogBtn_ = new QPushButton(QStringLiteral("Xóa log"), central);
    clearLogBtn_->setFixedHeight(26);
    logHeader->addWidget(logToggleBtn_);
    logHeader->addStretch(1);
    logHeader->addWidget(clearLogBtn_);
    root->addLayout(logHeader);

    logView_ = new QPlainTextEdit(central);
    logView_->setReadOnly(true);
    logView_->setMaximumBlockCount(300); // Giới hạn 300 dòng log để không phình RAM khi chạy lâu.
    logView_->setMinimumHeight(130);
    logView_->setStyleSheet(
        QStringLiteral("QPlainTextEdit { font-family: monospace; font-size: 12px; "
                       "background: #0f172a; color: #e2e8f0; border-radius: 6px; padding: 6px; }"));
    connect(clearLogBtn_, &QPushButton::clicked, logView_, &QPlainTextEdit::clear);
    root->addWidget(logView_, 1);

    setCentralWidget(central);
}

void MainWindow::loadSettings()
{
    QSettings settings(QStringLiteral("TDLOGY"), QStringLiteral("TDM2421_C2_Bridge"));
    usernameEdit_->setText(settings.value(QStringLiteral("username"), QStringLiteral("pilot")).toString());
    const bool rememberPass = settings.value(QStringLiteral("rememberPassword"), true).toBool();
    rememberPassCheck_->setChecked(rememberPass);
    if (rememberPass) {
        passwordEdit_->setText(settings.value(QStringLiteral("password"), QStringLiteral("pilot123")).toString());
    }
    remoteIdEdit_->setText(settings.value(QStringLiteral("remoteId"), QStringLiteral("1866ASDD0000VT5")).toString());
    hostEdit_->setText(settings.value(QStringLiteral("mqttHost"), QStringLiteral("171.251.89.98")).toString());
    mqttPortSpin_->setValue(settings.value(QStringLiteral("mqttPort"), 8903).toInt());
    tcpPortSpin_->setValue(settings.value(QStringLiteral("tcpPort"), 5760).toInt());
}

void MainWindow::saveSettings()
{
    QSettings settings(QStringLiteral("TDLOGY"), QStringLiteral("TDM2421_C2_Bridge"));
    settings.setValue(QStringLiteral("username"), usernameEdit_->text().trimmed());
    const bool rememberPass = rememberPassCheck_->isChecked();
    settings.setValue(QStringLiteral("rememberPassword"), rememberPass);
    if (rememberPass) {
        settings.setValue(QStringLiteral("password"), passwordEdit_->text());
    } else {
        // Bỏ chọn "Nhớ mật khẩu" thì xóa ngay mật khẩu đã lưu trước đó.
        settings.remove(QStringLiteral("password"));
    }
    settings.setValue(QStringLiteral("remoteId"), remoteIdEdit_->text().trimmed());
    settings.setValue(QStringLiteral("mqttHost"), hostEdit_->text().trimmed());
    settings.setValue(QStringLiteral("mqttPort"), mqttPortSpin_->value());
    settings.setValue(QStringLiteral("tcpPort"), tcpPortSpin_->value());
}

void MainWindow::updateTopicLabels()
{
    QString id = remoteIdEdit_->text().trimmed();
    if (id.isEmpty()) {
        id = QStringLiteral("<RemoteID>");
    }
    txTopicValue_->setText(QStringLiteral("p3000m/c2/%1/tx").arg(id));
    rxTopicValue_->setText(QStringLiteral("p3000m/c2/%1/rx").arg(id));
    qgcHintLabel_->setText(
        QStringLiteral("Trên QGroundControl: tạo Comm Link kiểu TCP → 127.0.0.1:%1")
            .arg(tcpPortSpin_->value()));
}

void MainWindow::setInputsEditable(bool editable)
{
    // Khóa các ô nhập khi đang chạy để tránh hiểu nhầm rằng sửa giữa chừng sẽ tự đổi topic/broker.
    usernameEdit_->setEnabled(editable);
    passwordEdit_->setEnabled(editable);
    rememberPassCheck_->setEnabled(editable);
    remoteIdEdit_->setEnabled(editable);
    hostEdit_->setEnabled(editable);
    mqttPortSpin_->setEnabled(editable);
    tcpPortSpin_->setEnabled(editable);
}

void MainWindow::setConnectButtonVisual(bool connectedMode)
{
    if (connectedMode) {
        connectBtn_->setText(QStringLiteral("Ngắt kết nối"));
        connectBtn_->setStyleSheet(
            QStringLiteral("QPushButton { background-color: #c62828; color: white; font-weight: 700; "
                           "border-radius: 6px; padding: 6px 18px; }"
                           "QPushButton:hover { background-color: #8e0000; }"));
    } else {
        connectBtn_->setText(QStringLiteral("Kết nối"));
        connectBtn_->setStyleSheet(
            QStringLiteral("QPushButton { background-color: #1565c0; color: white; font-weight: 700; "
                           "border-radius: 6px; padding: 6px 18px; }"
                           "QPushButton:hover { background-color: #0d47a1; }"));
    }
}

void MainWindow::onToggleConnect()
{
    if (worker_->isRunning()) {
        worker_->stop();
        setInputsEditable(true);
        setConnectButtonVisual(false);
        return;
    }

    const QString remoteId = remoteIdEdit_->text().trimmed();
    const QString host = hostEdit_->text().trimmed();
    if (remoteId.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("Thiếu thông tin"),
                             QStringLiteral("Vui lòng nhập mã Remote ID."));
        remoteIdEdit_->setFocus();
        return;
    }
    if (host.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("Thiếu thông tin"),
                             QStringLiteral("Vui lòng nhập MQTT Host / IP."));
        return;
    }

    saveSettings();
    updateTopicLabels();

    C2BridgeConfig cfg;
    cfg.mqttHost = host;
    cfg.mqttPort = static_cast<uint16_t>(mqttPortSpin_->value());
    cfg.tcpPort = static_cast<uint16_t>(tcpPortSpin_->value());
    cfg.username = usernameEdit_->text().trimmed();
    cfg.password = passwordEdit_->text();
    cfg.remoteId = remoteId;

    setInputsEditable(false);
    setConnectButtonVisual(true);

    worker_->start(cfg);
    if (!worker_->isRunning()) {
        // Trường hợp mở cổng TCP cục bộ thất bại (ví dụ bị trùng cổng 5760).
        setInputsEditable(true);
        setConnectButtonVisual(false);
    }
}

void MainWindow::onRemoteIdEdited(const QString &)
{
    updateTopicLabels();
}

void MainWindow::onToggleAdvanced(bool checked)
{
    advancedPanel_->setVisible(checked);
    advancedToggleBtn_->setText(checked
        ? QStringLiteral("▼ Cài đặt nâng cao (Host, MQTT Port, TCP Port)")
        : QStringLiteral("▶ Cài đặt nâng cao (Host, MQTT Port, TCP Port)"));
    adjustSize();
}

void MainWindow::onTogglePasswordVisibility(bool checked)
{
    passwordEdit_->setEchoMode(checked ? QLineEdit::Normal : QLineEdit::Password);
    showPassBtn_->setText(checked ? QStringLiteral("Ẩn") : QStringLiteral("Hiện"));
}

void MainWindow::onToggleLogVisibility(bool checked)
{
    logView_->setVisible(checked);
    clearLogBtn_->setVisible(checked);
    logToggleBtn_->setText(checked
        ? QStringLiteral("▼ Nhật ký hoạt động")
        : QStringLiteral("▶ Nhật ký hoạt động"));
    adjustSize();
}

void MainWindow::onStatusChanged(MqttConnState mqttState, QgcConnState qgcState, const QString &detail)
{
    switch (mqttState) {
    case MqttConnState::Disconnected:
        mqttStatusBadge_->setText(QStringLiteral("Chưa kết nối"));
        mqttStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#e2e8f0"), QStringLiteral("#334155")));
        break;
    case MqttConnState::Connecting:
        mqttStatusBadge_->setText(QStringLiteral("Đang kết nối..."));
        mqttStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#fef3c7"), QStringLiteral("#92400e")));
        break;
    case MqttConnState::Connected:
        mqttStatusBadge_->setText(QStringLiteral("Đã kết nối"));
        mqttStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#dcfce7"), QStringLiteral("#166534")));
        break;
    case MqttConnState::Reconnecting:
        mqttStatusBadge_->setText(QStringLiteral("Mất kết nối — đang thử lại"));
        mqttStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#fee2e2"), QStringLiteral("#991b1b")));
        break;
    case MqttConnState::AuthError:
        mqttStatusBadge_->setText(detail.isEmpty() ? QStringLiteral("Lỗi xác thực MQTT") : detail);
        mqttStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#fee2e2"), QStringLiteral("#991b1b")));
        setInputsEditable(true);
        setConnectButtonVisual(false);
        break;
    }

    switch (qgcState) {
    case QgcConnState::Stopped:
        qgcStatusBadge_->setText(QStringLiteral("Chưa chạy"));
        qgcStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#e2e8f0"), QStringLiteral("#334155")));
        break;
    case QgcConnState::Waiting:
        qgcStatusBadge_->setText(QStringLiteral("Đang chờ QGC (:%1)").arg(tcpPortSpin_->value()));
        qgcStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#fef3c7"), QStringLiteral("#92400e")));
        break;
    case QgcConnState::Connected:
        qgcStatusBadge_->setText(QStringLiteral("Đã kết nối"));
        qgcStatusBadge_->setStyleSheet(badgeStyle(QStringLiteral("#dcfce7"), QStringLiteral("#166534")));
        break;
    }
}

void MainWindow::onStatsUpdated(const C2BridgeStats &stats)
{
    clientIdValue_->setText(stats.clientId.isEmpty() ? QStringLiteral("-") : stats.clientId);
    if (stats.qgcState == QgcConnState::Connected && !stats.qgcPeer.isEmpty()) {
        qgcStatusBadge_->setText(QStringLiteral("Đã kết nối (%1)").arg(stats.qgcPeer));
    }

    dlRateValue_->setText(QStringLiteral("%1 Hz | %2 B/s")
                              .arg(stats.dlHz, 0, 'f', 1)
                              .arg(stats.dlBytesPerSec, 0, 'f', 0));
    mavFrameValue_->setText(QStringLiteral("%1 frame/s (Tổng: %2)")
                                .arg(stats.dlFramesPerSec, 0, 'f', 1)
                                .arg(stats.totalDlFrames));
    ulRateValue_->setText(QStringLiteral("%1 Hz | %2 B/s (Tổng: %3 B)")
                              .arg(stats.ulHz, 0, 'f', 1)
                              .arg(stats.ulBytesPerSec, 0, 'f', 0)
                              .arg(stats.totalUlBytes));
    maxGapValue_->setText(QStringLiteral("%1 ms").arg(stats.dlMaxGapMs, 0, 'f', 0));
}

void MainWindow::onAppendLog(const QString &line)
{
    const QString ts = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"));
    logView_->appendPlainText(QStringLiteral("[%1] %2").arg(ts, line));
}
