# TDM2421 v5 - MAVLink priority for WiFi/BLE Remote ID broadcast

Behavior:
- OPEN_DRONE_ID_BASIC_ID received from MAVLink is placed in UAS_data.BasicID[0] and is therefore the primary ID transmitted by WiFi NAN/beacon and BLE legacy/long-range.
- If no usable MAVLink Basic ID is available, BasicID[0] falls back to the existing local parameter UAS_ID.
- OPEN_DRONE_ID_OPERATOR_ID, SELF_ID, SYSTEM and LOCATION continue to populate UAS_data from MAVLink when present.
- MQTT v4 priority behavior is retained: MAVLink Basic ID/Operator ID are used when present, otherwise local IDs are used.
- Location latch/source arbitration from v3 is retained.

Expected with current test values:
- WiFi/BLE primary UAS ID: 1866ASDD0000BID
- Operator ID: 1866ASDD0000OID
- MQTT objectId/uavSN: 1866ASDD0000BID
- MQTT operatorId: 1866ASDD0000OID
