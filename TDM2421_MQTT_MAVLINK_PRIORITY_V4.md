# TDM2421 MQTT MAVLink Priority v4

MQTT now uses Remote ID identity received over MAVLink when available:
- OPEN_DRONE_ID_BASIC_ID.UASID -> MQTT objectId and extension.uavSN
- OPEN_DRONE_ID_OPERATOR_ID.operator_id -> MQTT extension.operatorId
- OPEN_DRONE_ID_SYSTEM operator lat/lon -> pilotLat/pilotLon (existing behavior)
- selected MAVLink Location -> UAV position (existing v3 latch behavior)

Fallback behavior:
- If no MAVLink Basic ID is available, objectId falls back to local BasicID/deviceId and uavSN falls back to configured UAV Serial.
- If no MAVLink Operator ID is available, operatorId falls back to OperatorID already present in UAS_data, otherwise empty.

The MAVLink strings are copied into the MQTT snapshot under the snapshot mutex so the asynchronous modem task never keeps pointers into MAVLink receive buffers.
