# TDM2421 MQTT publish rate from Makefile

Build-time override now accepts 0 or 1..30 Hz.

Examples:

```bash
make tdm2421 MQTT_HZ=30
make tdm2421 MQTT_HZ=20
make tdm2421 MQTT_HZ=10
make tdm2421 MQTT_HZ=5
make tdm2421 MQTT_HZ=1
make tdm2421 MQTT_HZ=0   # use NVS/Web pub_ms
```

Default remains 10 Hz when MQTT_HZ is omitted.
At 30 Hz the current millisecond scheduler uses a 33 ms interval, so the nominal scheduler rate is about 30.3 Hz.
