# TDM2421 MAVLink Identity Latch v6

Behavior on TDM2421:

- UAV position has independent source arbitration: MAVLink/OpenDroneID first, EG800K GNSS fallback after MAVLink location timeout.
- Basic ID and Operator ID are latched in RAM after the first valid MAVLink OPEN_DRONE_ID_BASIC_ID / OPEN_DRONE_ID_OPERATOR_ID message.
- The accepted OPEN_DRONE_ID_SYSTEM/operator location is also latched in RAM and is never replaced with the onboard EG800K position.
- Losing MAVLink therefore does not change the UAV identity seen over MQTT, BLE, or Wi-Fi.
- If no valid MAVLink identity has ever been received since boot, existing local/configured IDs remain the fallback.
- A reboot clears the RAM latch and starts from local/configured fallback until valid MAVLink RID identity arrives again.
