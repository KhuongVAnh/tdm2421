# TDM2421 RID Location Latch v3

Changes based on 2026-09-17 runtime log:

1. Refresh `now_ms` after `mavlink1.update()` / `dronecan.update()` to prevent unsigned age underflow when a receive handler timestamps a packet a few milliseconds after the loop-start timestamp.
2. Add `last_good_mav_location` snapshot (`ODID_Location_data`) and hold it while MAVLink location age <= 6500 ms.
3. EG800K may replace UAV location only after the MAVLink location is genuinely stale beyond the hold interval.
4. Move `check_parse()` / `arm_status_check()` out of `set_data()` and execute it only after final source arbitration, so `odid_initUasData()` cannot create transient `LOC` ARM_STATUS errors.
5. Preserve the v2 invalid operator-location filter and OpenDroneID source priority/debugging.

Expected normal log while OPEN_DRONE_ID_LOCATION is active:

    [RID SOURCE] MAVLink/OPEN_DRONE_ID_LOCATION ...
    [MQTT SOURCE] MAVLink/OPEN_DRONE_ID_LOCATION
    [MAV RID TX] ARM_STATUS status=0 reason=''

There should be no one-loop EG800K_GNSS/NO_LOCATION transition between healthy OpenDroneID Location packets.
