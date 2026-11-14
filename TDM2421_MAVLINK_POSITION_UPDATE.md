# TDM2421 MAVLink position update

Position-source priority implemented in `RemoteIDModule/mavlink.cpp`:

1. `OPEN_DRONE_ID_LOCATION` — highest priority; preserves original ArduRemoteID RID behaviour and fields.
2. `GLOBAL_POSITION_INT` — used when native OpenDroneID Location has not been received for 2 s; converted to the internal OpenDroneID Location structure.
3. `GPS_RAW_INT` — used only when neither native OpenDroneID Location nor GLOBAL_POSITION_INT is fresh for 2 s.
4. EG800K GNSS — existing fallback in `RemoteIDModule.ino`, selected only after the MAVLink location is stale for 5 s.

The original MAVLink OpenDroneID messages for BasicID, Authentication, SelfID, System/SystemUpdate and OperatorID remain unchanged.

TDM2421 USB debug now reports MAVLink RX byte/frame counters and decoded GLOBAL_POSITION_INT/GPS_RAW_INT. Native OPEN_DRONE_ID_LOCATION detailed printing remains controlled by OPTIONS_PRINT_RID_MAVLINK.

Build command:

    cd RemoteIDModule
    make tdm2421

Upload command (adjust SERDEV to your stable USB by-id path):

    make tdm2421 upload-tdm2421 SERDEV=/dev/serial/by-id/<your-device>

Note: the modified source could not be compiled in the ChatGPT sandbox because the ESP32 Arduino platform is not installed there and outbound access to downloads.arduino.cc is blocked. Compile it in the existing development environment where `make tdm2421` already works.
