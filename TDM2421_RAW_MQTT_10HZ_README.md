# TDM2421 RAW MQTT 10 Hz

Changes in this build:
- EG800K UART remains 460800 baud.
- Replaces per-message Quectel `QMTPUBEX` with MQTT 3.1.1 packets generated on ESP32.
- Opens persistent TCP socket 0 using `AT+QIOPEN=1,0,"TCP",...,0,1` (direct push mode).
- MQTT CONNECT, QoS0 PUBLISH and optional PINGREQ are serialized directly by ESP32.
- Payload is sent with length-delimited `AT+QISEND=0,<len>`.
- Keeps modem in AT-command mode so GNSS and CSQ still work; transparent mode is intentionally not used.
- Keeps TIME DEBUG and CRLF terminal formatting.
- Stream log reports measured rate and raw TCP send counters.

Expected startup markers:
```
[RAW MQTT] TCP connected
[RAW MQTT] CONNACK accepted
[RAW MQTT] connected
```

Expected streaming marker:
```
[RAW MQTT] stream msgCnt=100 bytes=... rate=... Hz sendOK=... sendErr=0
```
