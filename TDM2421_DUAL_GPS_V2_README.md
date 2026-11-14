# TDM2421 Dual GPS V2

Location priority:
1. Fresh MAVLink location (<=5 s)
2. Fresh EG800K GNSS fix (<=5 s)
3. NO_LOCATION

GNSS startup is now deterministic:
- AT+QGPS?
- if not running: AT+QGPSEND (ERROR is tolerated), AT+QGPS=1, AT+QGPS? verification
- GNSS is considered started only after +QGPS: 1 is observed.

AT+QGPSLOC=2 is polled once per second. +CME ERROR: 516 is treated as normal NO FIX acquisition state and logged at most once per 5 seconds.

MQTT safety:
- MQTT receives a snapshot only when a real selected location is valid.
- bcast_powerup may still temporarily force OpenDroneID LocationValid for RF power-up behavior, but that forced sentinel state is never submitted to MQTT.
- publishSnapshot() retains a final LocationValid guard.

Build production:
    cd RemoteIDModule
    make tdm2421
    make upload-TDM2421
