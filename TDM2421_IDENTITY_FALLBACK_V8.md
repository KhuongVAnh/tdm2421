# TDM2421 identity fallback v8

- Position remains independent: fresh MAVLink position first; after timeout use EG800K GNSS.
- A missing MAVLink identity on a later submit no longer clears the last valid Basic ID or Operator ID.
- Last valid MAVLink Basic ID and Operator ID are persisted in ESP32 NVS (`tdmnet/ridbasic`, `tdmnet/ridoper`).
- On reboot or when MAVLink does not currently provide an Operator ID, MQTT can reuse the persisted last-valid ID.
- If the unit has never received an Operator ID and none exists in UAS_data, operatorId remains empty; the firmware does not invent an Operator ID.
- EG800K GNSS is never used as pilot/operator location.
