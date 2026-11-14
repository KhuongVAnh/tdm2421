# TDM2421 MQTT publish frequency from Makefile

The TDM2421 RAW MQTT firmware can select the publish frequency at build time.

## Fixed build-time rate

From `RemoteIDModule/`:

```bash
make tdm2421 MQTT_HZ=10   # 10 Hz, 100 ms
make tdm2421 MQTT_HZ=5    # 5 Hz, 200 ms
make tdm2421 MQTT_HZ=2    # 2 Hz, 500 ms
make tdm2421 MQTT_HZ=1    # 1 Hz, 1000 ms
```

Valid fixed range: 1..10 Hz.

When `MQTT_HZ` is 1..10, the build-time setting overrides an old `pub_ms` value stored in NVS and also remains authoritative if the Web form posts another `pub_ms` value.

At boot the terminal reports, for example:

```text
[MQTT CONFIG] build rate=5 Hz interval=200 ms (overrides NVS/Web)
```

## Use Web/NVS control instead

```bash
make tdm2421 MQTT_HZ=0
```

Then the firmware uses `publish_interval_ms` / `pub_ms` from NVS and the Web dashboard.

## Default

If omitted:

```bash
make tdm2421
```

is equivalent to:

```bash
make tdm2421 MQTT_HZ=10
```

## Fake RID test build

```bash
make tdm2421-test MQTT_HZ=10
make tdm2421-test MQTT_HZ=5
```
