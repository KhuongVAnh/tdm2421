# TDM2421 MAVLink OpenDroneID full debug

Adds unconditional serial debug on BOARD_TDM2421 for:
- RX OPEN_DRONE_ID_BASIC_ID
- RX OPEN_DRONE_ID_LOCATION
- RX OPEN_DRONE_ID_SELF_ID
- RX OPEN_DRONE_ID_SYSTEM
- RX OPEN_DRONE_ID_SYSTEM_UPDATE
- RX OPEN_DRONE_ID_OPERATOR_ID
- TX OPEN_DRONE_ID_ARM_STATUS (1 Hz debug)
- RX GLOBAL_POSITION_INT and GPS_RAW_INT fallbacks
- RID/MQTT selected location source

Location priority remains:
1. OPEN_DRONE_ID_LOCATION
2. GLOBAL_POSITION_INT
3. GPS_RAW_INT
4. EG800K GNSS after MAVLink location timeout

No fake Operator ID or Operator Location is generated. This debug build is intended to show whether those OpenDroneID messages actually arrive from the MAVLink side.
