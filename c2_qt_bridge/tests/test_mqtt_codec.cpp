#include <QtTest/QtTest>
#include "MqttCodec.h"
#include "C2BridgeWorker.h"

#include <QTcpServer>
#include <QTcpSocket>

class TestMqttCodec : public QObject {
    Q_OBJECT

private slots:
    void testRemainingLengthAndPublishRoundTrip()
    {
        const QByteArray payload(300, 'A');
        const QByteArray pkt = MqttCodec::buildPublish(QStringLiteral("p3000m/c2/1866ASDD0000VT5/rx"), payload);

        QByteArray stream = pkt;
        bool malformed = false;
        const QVector<MqttPacket> parsed = MqttCodec::parsePackets(stream, &malformed);
        QVERIFY(!malformed);
        QVERIFY(stream.isEmpty());
        QCOMPARE(parsed.size(), 1);
        QCOMPARE(parsed[0].packetType(), uint8_t(3));

        const MqttPublishData pub = MqttCodec::parsePublishBody(parsed[0].headerByte, parsed[0].body);
        QVERIFY(pub.valid);
        QCOMPARE(pub.topic, QStringLiteral("p3000m/c2/1866ASDD0000VT5/rx"));
        QCOMPARE(pub.payload, payload);
    }

    void testFragmentedAndCoalescedPackets()
    {
        const QByteArray p1 = MqttCodec::buildConnect(QStringLiteral("cid_1"), QStringLiteral("pilot"), QStringLiteral("pilot123"));
        const QByteArray p2 = MqttCodec::buildSubscribe(1, QStringLiteral("p3000m/c2/TEST/tx"));
        const QByteArray combined = p1 + p2;

        QByteArray buf;
        bool malformed = false;
        // Feed first 5 bytes only (incomplete packet)
        buf.append(combined.left(5));
        QCOMPARE(MqttCodec::parsePackets(buf, &malformed).size(), 0);
        QVERIFY(!malformed);

        // Feed the rest
        buf.append(combined.mid(5));
        const QVector<MqttPacket> parsed = MqttCodec::parsePackets(buf, &malformed);
        QVERIFY(!malformed);
        QVERIFY(buf.isEmpty());
        QCOMPARE(parsed.size(), 2);
        QCOMPARE(parsed[0].packetType(), uint8_t(1));
        QCOMPARE(parsed[1].packetType(), uint8_t(8));
    }

    void testCountMavlinkFramesConcatenated()
    {
        // MAVLink v2 unsigned frame: STX=0xFD, len=9, incompat=0, total=10+9+2=21 bytes
        QByteArray v2(21, '\0');
        v2[0] = static_cast<char>(0xFD);
        v2[1] = 9;

        // MAVLink v2 signed frame: STX=0xFD, len=5, incompat=1, total=10+5+2+13=30 bytes
        QByteArray v2Signed(30, '\0');
        v2Signed[0] = static_cast<char>(0xFD);
        v2Signed[1] = 5;
        v2Signed[2] = 0x01;

        // MAVLink v1 frame: STX=0xFE, len=4, total=6+4+2=12 bytes
        QByteArray v1(12, '\0');
        v1[0] = static_cast<char>(0xFE);
        v1[1] = 4;

        const QByteArray batch = v2 + v2Signed + v1;
        QCOMPARE(MqttCodec::countMavlinkFrames(batch), 3);
        QCOMPARE(MqttCodec::countMavlinkFrames(batch + QByteArray(3, '\xFD')), 3);
    }

    void testEndToEndLoopbackBridge()
    {
        // Mock MQTT broker
        QTcpServer mockBroker;
        QVERIFY(mockBroker.listen(QHostAddress::LocalHost, 0));

        QTcpSocket *brokerClient = nullptr;
        QByteArray brokerRx;
        bool brokerSawConnect = false;
        bool brokerSawSubscribe = false;
        QByteArray uplinkReceivedAtBroker;

        QObject::connect(&mockBroker, &QTcpServer::newConnection, &mockBroker, [&]() {
            brokerClient = mockBroker.nextPendingConnection();
            QObject::connect(brokerClient, &QTcpSocket::readyRead, brokerClient, [&]() {
                brokerRx.append(brokerClient->readAll());
                bool malformed = false;
                const auto pkts = MqttCodec::parsePackets(brokerRx, &malformed);
                for (const auto &p : pkts) {
                    if (p.packetType() == 1) { // CONNECT -> send CONNACK rc=0
                        brokerSawConnect = true;
                        brokerClient->write(QByteArray("\x20\x02\x00\x00", 4));
                    } else if (p.packetType() == 8) { // SUBSCRIBE -> send SUBACK
                        brokerSawSubscribe = true;
                        brokerClient->write(QByteArray("\x90\x03\x00\x01\x00", 5));
                    } else if (p.packetType() == 3) { // PUBLISH uplink from QGC
                        const auto pub = MqttCodec::parsePublishBody(p.headerByte, p.body);
                        if (pub.valid && pub.topic == QStringLiteral("p3000m/c2/UNITTEST/rx")) {
                            uplinkReceivedAtBroker.append(pub.payload);
                        }
                    }
                }
            });
        });

        // Pick an ephemeral TCP port for QGC server
        QTcpServer probe;
        QVERIFY(probe.listen(QHostAddress::LocalHost, 0));
        const uint16_t qgcPort = probe.serverPort();
        probe.close();

        C2BridgeWorker worker;
        bool workerConnected = false;
        QObject::connect(&worker, &C2BridgeWorker::statusChanged, &worker,
                         [&](MqttConnState m, QgcConnState, const QString &) {
                             if (m == MqttConnState::Connected) {
                                 workerConnected = true;
                             }
                         });

        C2BridgeConfig cfg;
        cfg.mqttHost = QStringLiteral("127.0.0.1");
        cfg.mqttPort = mockBroker.serverPort();
        cfg.tcpPort = qgcPort;
        cfg.username = QStringLiteral("pilot");
        cfg.password = QStringLiteral("pilot123");
        cfg.remoteId = QStringLiteral("UNITTEST");
        worker.start(cfg);

        QTRY_VERIFY_WITH_TIMEOUT(workerConnected, 3000);
        QVERIFY(brokerSawConnect);
        QVERIFY(brokerSawSubscribe);

        // Mock QGC client connects to worker's TCP server
        QTcpSocket mockQgc;
        QByteArray downlinkReceivedAtQgc;
        QObject::connect(&mockQgc, &QTcpSocket::readyRead, &mockQgc, [&]() {
            downlinkReceivedAtQgc.append(mockQgc.readAll());
        });
        mockQgc.connectToHost(QHostAddress::LocalHost, qgcPort);
        QVERIFY(mockQgc.waitForConnected(2000));

        // 1) Broker publishes downlink MAVLink frame to p3000m/c2/UNITTEST/tx -> QGC receives it
        QByteArray dlFrame(21, '\x11');
        dlFrame[0] = static_cast<char>(0xFD);
        dlFrame[1] = 9;
        dlFrame[2] = 0;
        brokerClient->write(MqttCodec::buildPublish(QStringLiteral("p3000m/c2/UNITTEST/tx"), dlFrame));
        QTRY_COMPARE_WITH_TIMEOUT(downlinkReceivedAtQgc, dlFrame, 2000);

        // 2) QGC sends uplink bytes -> Broker receives PUBLISH on p3000m/c2/UNITTEST/rx
        const QByteArray ulFrame("MAVLINK_CMD_TEST");
        mockQgc.write(ulFrame);
        QTRY_COMPARE_WITH_TIMEOUT(uplinkReceivedAtBroker, ulFrame, 2000);

        worker.stop();
    }
};

QTEST_MAIN(TestMqttCodec)
#include "test_mqtt_codec.moc"
