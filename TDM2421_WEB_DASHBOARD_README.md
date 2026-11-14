# TDM2421 Local Wi-Fi Dashboard + LTE/MQTT configuration

This source tree extends the TDM2421 port with a local ESP32-C3 Wi-Fi AP and Web dashboard while keeping the EG800K on UART0 for LTE/MQTT and Pixhawk MAVLink on UART1.

## Default local Wi-Fi AP

- SSID: `TDM2421_Config`
- Password: `tdm2421@123`
- IP: `192.168.4.1`
- Open in a browser: `http://192.168.4.1/`

The AP uses the same ESP32-C3 AP interface as the OpenDroneID Wi-Fi beacon/vendor IE. It is not an LTE Internet-sharing hotspot.

## Dashboard functions

The dashboard shows:

- Wi-Fi AP SSID, IP, channel and connected-client count
- EG800K SIM ready state
- LTE registration, PDP state, LTE IP and RSSI
- MQTT connection state, broker, topic, message counter, last publish age and last error
- Remote ID/MAVLink data source, MAVLink state, latitude, longitude, altitude, speed and heading

Configuration fields:

- Wi-Fi AP SSID/password/channel
- APN
- MQTT host/port/username/password/client ID/topic
- Device ID, version, bandwidth text
- Publish interval and reconnect interval

Settings are saved in ESP32 NVS. Password fields are not returned by the JSON API. Leaving a password field empty keeps the currently stored password.

## JSON API

- `GET /api/status` : combined TDM2421/LTE/MQTT + Remote ID status
- `GET /api/config` : non-secret configuration
- `GET /ajax/status.json` : original ArduRemoteID status JSON
- `GET /index.html` : original ArduRemoteID page

## Build

Normal firmware:

```bash
cd RemoteIDModule
make tdm2421
```

Bench/fake Remote ID test firmware:

```bash
make tdm2421-test
```

Flash merged firmware:

```bash
make upload-TDM2421
```

## Important

The source package intentionally does not provide a newly-built `.bin` because the build environment used to prepare this patch did not have the ESP32 Arduino core installed and had no network access to install it. Build locally with the repository's Arduino environment before flashing.
