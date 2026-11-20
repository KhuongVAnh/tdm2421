/*
  mavlink class for handling OpenDroneID messages
 */
#pragma once
#include "transport.h"
#include "parameters.h"

// Name of the MAVLink message currently supplying RID Location.
// Returns: OPEN_DRONE_ID_LOCATION, GLOBAL_POSITION_INT, GPS_RAW_INT, or NONE.
const char *mavlink_location_source_name(void);

// Trạng thái kết nối FC qua cổng UART MAVLink (Serial1).
bool mavlink_fc_connected(void);
String mavlink_fc_status_json(void);

/*
  abstraction for MAVLink on a serial port
 */
class MAVLinkSerial : public Transport {
public:
    using Transport::Transport;
    MAVLinkSerial(HardwareSerial &serial, mavlink_channel_t chan);
    void init(void) override;
    void update(void) override;

private:
    HardwareSerial &serial;
    mavlink_channel_t chan;
    uint32_t last_hb_ms;
    uint32_t last_hb_warn_ms;
    uint32_t param_request_last_ms;
    const Parameters::Param *param_next;

    void update_receive(void);
    void update_send(void);
    void process_packet(mavlink_status_t &status, mavlink_message_t &msg);
    void mav_printf(uint8_t severity, const char *fmt, ...);
    void handle_secure_command(const mavlink_secure_command_t &pkt);

    void arm_status_send(void);
};
