# TDM2421 RID source/filter fix

Changes:
1. Reject OPEN_DRONE_ID_SYSTEM / SYSTEM_UPDATE operator locations that are unset or invalid (0,0; altitude <= -999; out-of-range coordinates), preserving the last valid SYSTEM.
2. Keep native OPEN_DRONE_ID_LOCATION as highest-priority MAVLink location.
3. Add 6.5 s MAVLink hold/grace period before switching to EG800K GNSS, preventing transient source flapping.
4. Rate-limit [MQTT SOURCE] diagnostics to source changes or 1 Hz.
5. Keep detailed [MAV RID RX] logs and mark SYSTEM packets ACCEPT or REJECT_INVALID_OP_LOC.

Expected examples:
[MAV RID RX] SYSTEM ... ACCEPT
[MAV RID RX] SYSTEM ... REJECT_INVALID_OP_LOC
[RID SOURCE] MAVLink/OPEN_DRONE_ID_LOCATION ...
[MQTT SOURCE] MAVLink/OPEN_DRONE_ID_LOCATION
