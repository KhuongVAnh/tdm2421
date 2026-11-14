#include "webinterface.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <WiFiAP.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "parameters.h"
#include "romfs.h"
#include "check_firmware.h"
#include "status.h"
#if defined(BOARD_TDM2421)
#include "tdm2421_mqtt.h"
#endif

static WebServer server(80);

static String htmlEscape(String s)
{
    s.replace("&", "&amp;");
    s.replace("\"", "&quot;");
    s.replace("<", "&lt;");
    s.replace(">", "&gt;");
    return s;
}

static String jsonEscape(String s)
{
    s.replace("\\", "\\\\");
    s.replace("\"", "\\\"");
    s.replace("\n", "\\n");
    s.replace("\r", "");
    return s;
}

/*
  serve files from ROMFS
 */
class ROMFS_Handler : public RequestHandler
{
    bool canHandle(HTTPMethod method, String uri) {
#if defined(BOARD_TDM2421)
        // On TDM2421 the root path is the new local configuration dashboard.
        // The original ArduRemoteID page remains available as /index.html.
        if (uri == "/") {
            return false;
        }
#else
        if (uri == "/") {
            uri = "/index.html";
        }
#endif
        uri = "web" + uri;
        return ROMFS::exists(uri.c_str());
    }

    bool handle(WebServer& server, HTTPMethod requestMethod, String requestUri) {
        if (requestUri == "/") {
            requestUri = "/index.html";
        }
        String uri = "web" + requestUri;
        Serial.printf("handle: '%s'\r\n", requestUri.c_str());

        const char *content_type = "text/html";
        const struct {
            const char *extension;
            const char *content_type;
        } extensions[] = {
            { ".js", "text/javascript" },
            { ".jpg", "image/jpeg" },
            { ".png", "image/png" },
            { ".css", "text/css" },
        };
        for (const auto &e : extensions) {
            if (uri.endsWith(e.extension)) {
                content_type = e.content_type;
                break;
            }
        }

        auto *f = ROMFS::find_stream(uri.c_str());
        if (f != nullptr) {
            server.sendHeader("Content-Encoding", "gzip");
            server.streamFile(*f, content_type);
            delete f;
            return true;
        }
        return false;
    }

} ROMFS_Handler;

/*
  serve legacy ArduRemoteID AJAX status
 */
class AJAX_Handler : public RequestHandler
{
    bool canHandle(HTTPMethod method, String uri) {
        return uri == "/ajax/status.json";
    }

    bool handle(WebServer& server, HTTPMethod requestMethod, String requestUri) {
        if (requestUri != "/ajax/status.json") {
            return false;
        }
        server.send(200, "application/json", status_json());
        return true;
    }

} AJAX_Handler;

#if defined(BOARD_TDM2421)

static String tdmDashboardHtml()
{
    TDM2421Config cfg {};
    tdm2421_mqtt.getConfig(cfg);

    String h;
    h.reserve(12000);
    h += F("<!doctype html><html><head><meta charset='utf-8'>");
    h += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
    h += F("<title>TDM2421 Control</title><style>");
    h += F("body{font-family:Arial,sans-serif;margin:0;background:#f3f5f7;color:#18212b}"
           ".top{background:#172b4d;color:white;padding:16px 20px}.top h1{margin:0;font-size:24px}"
           ".wrap{max-width:980px;margin:auto;padding:16px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px}"
           ".card{background:white;border-radius:10px;padding:16px;box-shadow:0 1px 5px #0002;margin-bottom:14px}"
           ".row{display:flex;justify-content:space-between;gap:12px;border-bottom:1px solid #eee;padding:7px 0}.row:last-child{border:0}"
           ".v{font-family:monospace;text-align:right;word-break:break-all}.ok{color:#16803a;font-weight:bold}.bad{color:#b42318;font-weight:bold}"
           "label{display:block;font-weight:bold;margin-top:10px}input{width:100%;box-sizing:border-box;padding:9px;border:1px solid #bbb;border-radius:6px}"
           "button{margin-top:16px;padding:10px 18px;border:0;border-radius:7px;background:#1769aa;color:white;font-weight:bold}"
           ".note{font-size:13px;color:#59636e}.nav a{color:#dbeafe;margin-right:15px}.two{display:grid;grid-template-columns:1fr 1fr;gap:12px}"
           "@media(max-width:650px){.two{grid-template-columns:1fr}}"
           "</style></head><body>");
    h += F("<div class='top'><h1>TDM2421 Local Dashboard</h1><div class='nav'><a href='/'>Status & Config</a><a href='/index.html'>ArduRemoteID Legacy</a><a href='/api/status'>JSON API</a></div></div>");
    h += F("<div class='wrap'><div class='grid'>");

    h += F("<div class='card'><h3>Wi-Fi AP</h3>"
           "<div class='row'><span>SSID</span><span class='v' id='ap_ssid'></span></div>"
           "<div class='row'><span>IP</span><span class='v' id='ap_ip'></span></div>"
           "<div class='row'><span>Clients</span><span class='v' id='ap_clients'></span></div>"
           "<div class='row'><span>Channel</span><span class='v' id='ap_channel'></span></div></div>");

    h += F("<div class='card'><h3>EG800K / LTE</h3>"
           "<div class='row'><span>SIM</span><span class='v' id='sim'></span></div>"
           "<div class='row'><span>LTE registration</span><span class='v' id='lte_reg'></span></div>"
           "<div class='row'><span>PDP</span><span class='v' id='pdp'></span></div>"
           "<div class='row'><span>LTE IP</span><span class='v' id='lte_ip'></span></div>"
           "<div class='row'><span>RSSI</span><span class='v' id='rssi'></span></div>"
           "<div class='row'><span>GNSS engine</span><span class='v' id='gnss_engine'></span></div>"
           "<div class='row'><span>Location valid</span><span class='v' id='gnss'></span></div>"
           "<div class='row'><span>Fix status</span><span class='v' id='gnss_fix'></span></div>"
           "<div class='row'><span>Satellites reported</span><span class='v' id='gnss_sat'></span></div>"
           "<div class='row'><span>Satellites used (GSA)</span><span class='v' id='gnss_used'></span></div>"
           "<div class='row'><span>HDOP</span><span class='v' id='gnss_hdop'></span></div>"
           "<div class='row'><span>VDOP</span><span class='v' id='gnss_vdop'></span></div>"
           "<div class='row'><span>PDOP</span><span class='v' id='gnss_pdop'></span></div>"
           "<div class='row'><span>GNSS UTC</span><span class='v' id='gnss_utc'></span></div>"
           "<div class='row'><span>GNSS date</span><span class='v' id='gnss_date'></span></div>"
           "<div class='row'><span>Fix age</span><span class='v' id='gnss_age'></span></div>"
           "<div class='row'><span>GNSS latitude</span><span class='v' id='gnss_lat'></span></div>"
           "<div class='row'><span>GNSS longitude</span><span class='v' id='gnss_lon'></span></div>"
           "<div class='row'><span>GNSS altitude</span><span class='v' id='gnss_alt'></span></div>"
           "<div class='row'><span>GNSS speed</span><span class='v' id='gnss_speed'></span></div>"
           "<div class='row'><span>GNSS course</span><span class='v' id='gnss_course'></span></div></div>");

    h += F("<div class='card'><h3>MQTT</h3>"
           "<div class='row'><span>Connected</span><span class='v' id='mqtt'></span></div>"
           "<div class='row'><span>Broker</span><span class='v' id='broker'></span></div>"
           "<div class='row'><span>Topic</span><span class='v' id='topic'></span></div>"
           "<div class='row'><span>Published</span><span class='v' id='msgcnt'></span></div>"
           "<div class='row'><span>Last publish age</span><span class='v' id='pubage'></span></div>"
           "<div class='row'><span>Last error</span><span class='v' id='err'></span></div></div>");

    h += F("<div class='card'><h3>UAV / MAVLink / Remote ID</h3>"
           "<div class='row'><span>Data source</span><span class='v' id='source'></span></div>"
           "<div class='row'><span>MAVLink</span><span class='v' id='mav'></span></div>"
           "<div class='row'><span>Object ID</span><span class='v' id='object_id'></span></div>"
           "<div class='row'><span>Basic ID / UAS ID</span><span class='v' id='basic_id'></span></div>"
           "<div class='row'><span>Basic ID type</span><span class='v' id='basic_type'></span></div>"
           "<div class='row'><span>Basic ID source</span><span class='v' id='basic_source'></span></div>"
           "<div class='row'><span>UAV Serial</span><span class='v' id='uav_serial'></span></div>"
           "<div class='row'><span>Device ID</span><span class='v' id='device_id'></span></div>"
           "<div class='row'><span>Latitude</span><span class='v' id='lat'></span></div>"
           "<div class='row'><span>Longitude</span><span class='v' id='lon'></span></div>"
           "<div class='row'><span>Altitude Geo</span><span class='v' id='alt'></span></div>"
           "<div class='row'><span>Speed</span><span class='v' id='speed'></span></div>"
           "<div class='row'><span>Heading</span><span class='v' id='hdg'></span></div></div>");

    h += F("</div><div class='card'><h2>Configuration</h2><form method='POST' action='/save'>");
    h += F("<h3>Wi-Fi AP</h3><div class='two'><div><label>SSID</label><input name='ap_ssid' maxlength='20' value='");
    h += htmlEscape(g.wifi_ssid);
    h += F("'></div><div><label>Wi-Fi channel</label><input name='ap_channel' type='number' min='1' max='13' value='");
    h += String(g.wifi_channel);
    h += F("'></div></div><label>New AP password</label><input name='ap_pass' type='password' minlength='8' maxlength='20' placeholder='Leave blank to keep current password'>");

    h += F("<h3>Remote ID Identity</h3><div class='two'><div><label>Basic ID / UAS ID</label><input name='basic_id' maxlength='20' value='");
    h += htmlEscape(g.uas_id);
    h += F("'></div><div><label>UAV Serial</label><input name='uav_serial' maxlength='20' value='");
    h += htmlEscape(cfg.uav_serial);
    h += F("'></div></div><p class='note'>Basic ID is the OpenDroneID identity broadcast over Wi-Fi/BLE. Saving a non-empty Basic ID configures UAS_TYPE=multirotor and UAS_ID_TYPE=serial number. MQTT objectId uses Basic ID first and falls back to Device ID. UAV Serial is used for extension.uavSN.</p>");

    h += F("<h3>LTE / MQTT</h3><div class='two'><div><label>APN</label><input name='apn' maxlength='32' value='");
    h += htmlEscape(cfg.apn);
    h += F("'></div><div><label>Publish interval (ms)</label><input name='pub_ms' type='number' min='100' max='60000' value='");
    h += String(cfg.publish_interval_ms);
    h += F("'></div></div><div class='two'><div><label>MQTT host</label><input name='host' maxlength='64' value='");
    h += htmlEscape(cfg.mqtt_host);
    h += F("'></div><div><label>MQTT port</label><input name='port' type='number' min='1' max='65535' value='");
    h += String(cfg.mqtt_port);
    h += F("'></div></div><div class='two'><div><label>Username</label><input name='user' maxlength='32' value='");
    h += htmlEscape(cfg.mqtt_user);
    h += F("'></div><div><label>New MQTT password</label><input name='pass' type='password' maxlength='64' placeholder='Leave blank to keep current password'></div></div>");
    h += F("<label>Client ID</label><input name='client' maxlength='64' value='");
    h += htmlEscape(cfg.mqtt_client);
    h += F("'><label>Topic</label><input name='topic' maxlength='96' value='");
    h += htmlEscape(cfg.mqtt_topic);
    h += F("'><div class='two'><div><label>Device ID</label><input name='device' maxlength='32' value='");
    h += htmlEscape(cfg.device_id);
    h += F("'></div><div><label>Reconnect interval (ms)</label><input name='recon_ms' type='number' min='1000' max='300000' value='");
    h += String(cfg.reconnect_interval_ms);
    h += F("'></div></div><div class='two'><div><label>Version</label><input name='version' maxlength='16' value='");
    h += htmlEscape(cfg.version);
    h += F("'></div><div><label>Bandwidth text</label><input name='bandwidth' maxlength='16' value='");
    h += htmlEscape(cfg.bandwidth);
    h += F("'></div></div><button type='submit'>SAVE & REBOOT</button>"
           "<p class='note'>Passwords are stored in ESP32 NVS and are never returned by the JSON API. Saving restarts the board so Wi-Fi and LTE/MQTT settings are applied cleanly.</p></form></div>");

    h += F("<div class='card'><h3>Notes</h3><p class='note'>The ESP32-C3 AP is also the Wi-Fi interface used by the OpenDroneID beacon/vendor IE. The EG800K continues to provide LTE/MQTT over UART0; this page does not route Internet traffic to connected Wi-Fi clients.</p></div></div>");

    h += F("<script>function yn(v){return v?'<span class=ok>YES</span>':'<span class=bad>NO</span>';}"
           "function set(id,v){document.getElementById(id).innerHTML=(v===undefined||v===null||v==='')?'-':v;}"
           "async function poll(){try{let r=await fetch('/api/status',{cache:'no-store'});let j=await r.json();let t=j.tdm||{},q=j.rid||{},a=j.ap||{};"
           "set('ap_ssid',a.ssid);set('ap_ip',a.ip);set('ap_clients',a.clients);set('ap_channel',a.channel);"
           "set('sim',yn(t.simReady));set('lte_reg',yn(t.lteRegistered));set('pdp',yn(t.pdpActive));set('lte_ip',t.lteIp);set('rssi',t.rssiDbm? t.rssiDbm+' dBm':'-');set('gnss_engine',yn(t.gnssEngineRunning));set('gnss',yn(t.gnssValid));set('gnss_fix',t.gnssFixText);set('gnss_sat',t.gnssSatellites);set('gnss_used',t.gnssDopValid?t.gnssSatellitesUsed:'N/A');set('gnss_hdop',t.gnssHdop);set('gnss_vdop',t.gnssDopValid?t.gnssVdop:'N/A');set('gnss_pdop',t.gnssDopValid?t.gnssPdop:'N/A');set('gnss_utc',t.gnssUtc);set('gnss_date',t.gnssDate);set('gnss_age',t.gnssAgeMs+' ms');set('gnss_lat',t.gnssLat);set('gnss_lon',t.gnssLon);set('gnss_alt',t.gnssAlt+' m');set('gnss_speed',t.gnssSpeed+' m/s');set('gnss_course',t.gnssCourseValid?t.gnssCourse+' deg':'N/A');"
           "set('mqtt',yn(t.mqttConnected));set('broker',t.broker);set('topic',t.topic);set('msgcnt',t.msgCnt);set('pubage',t.lastPublishAgeMs? t.lastPublishAgeMs+' ms':'-');set('err',t.lastError);"
           "set('source',q['STATUS:DATA_SOURCE']);set('mav',q['MAVLINK:Connected']);let bid=q['BASICID:UASID']||'';set('basic_id',bid||'N/A');set('basic_type',q['BASICID:IDType']||'N/A');set('basic_source',bid?'OpenDroneID Basic ID':'Device ID fallback');set('device_id',t.deviceId);set('uav_serial',t.uavSerial||bid||t.deviceId);set('object_id',bid||t.deviceId);set('lat',q['LOCATION:Latitude']);set('lon',q['LOCATION:Longitude']);set('alt',q['LOCATION:AltitudeGeo']+' m');set('speed',q['LOCATION:SpeedHorizontal']+' m/s');set('hdg',q['LOCATION:Direction']+' deg');}catch(e){set('err','dashboard poll failed');}}"
           "poll();setInterval(poll,1000);</script></body></html>");
    return h;
}

static void setupTDM2421Routes()
{
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", tdmDashboardHtml());
    });
    server.on("/tdm2421", HTTP_GET, []() {
        server.send(200, "text/html", tdmDashboardHtml());
    });
    server.on("/api/config", HTTP_GET, []() {
        String s = "{";
        s += "\"apSsid\":\"" + jsonEscape(g.wifi_ssid) + "\",";
        s += "\"apChannel\":" + String(g.wifi_channel) + ",";
        s += "\"mqtt\":" + tdm2421_mqtt.configJson();
        s += "}";
        server.send(200, "application/json", s);
    });
    server.on("/api/status", HTTP_GET, []() {
        String s;
        s.reserve(4096);
        s = "{";
        s += "\"tdm\":" + tdm2421_mqtt.statusJson() + ",";
        s += "\"rid\":" + status_json() + ",";
        s += "\"ap\":{";
        s += "\"ssid\":\"" + jsonEscape(g.wifi_ssid) + "\",";
        s += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
        s += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
        s += "\"channel\":" + String(g.wifi_channel);
        s += "}}";
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", s);
    });
    server.on("/save", HTTP_POST, []() {
        TDM2421Config cfg {};
        tdm2421_mqtt.getConfig(cfg);

        auto argToBuf = [](const String &name, char *dst, size_t len, bool allow_empty=true) {
            if (!server.hasArg(name)) return;
            String v = server.arg(name);
            v.trim();
            if (!allow_empty && v.length() == 0) return;
            memset(dst, 0, len);
            v.substring(0, len - 1).toCharArray(dst, len);
        };

        argToBuf("apn", cfg.apn, sizeof(cfg.apn), false);
        argToBuf("host", cfg.mqtt_host, sizeof(cfg.mqtt_host), false);
        argToBuf("user", cfg.mqtt_user, sizeof(cfg.mqtt_user), true);
        if (server.hasArg("pass") && server.arg("pass").length() > 0) {
            argToBuf("pass", cfg.mqtt_pass, sizeof(cfg.mqtt_pass), true);
        }
        argToBuf("client", cfg.mqtt_client, sizeof(cfg.mqtt_client), false);
        argToBuf("topic", cfg.mqtt_topic, sizeof(cfg.mqtt_topic), false);
        argToBuf("device", cfg.device_id, sizeof(cfg.device_id), false);
        argToBuf("uav_serial", cfg.uav_serial, sizeof(cfg.uav_serial), true);
        argToBuf("version", cfg.version, sizeof(cfg.version), false);
        argToBuf("bandwidth", cfg.bandwidth, sizeof(cfg.bandwidth), false);

        if (server.hasArg("port")) cfg.mqtt_port = (uint16_t)server.arg("port").toInt();
        if (server.hasArg("pub_ms")) cfg.publish_interval_ms = (uint32_t)server.arg("pub_ms").toInt();
        if (server.hasArg("recon_ms")) cfg.reconnect_interval_ms = (uint32_t)server.arg("recon_ms").toInt();

        String ap_ssid = server.hasArg("ap_ssid") ? server.arg("ap_ssid") : String(g.wifi_ssid);
        ap_ssid.trim();
        const int ap_channel = server.hasArg("ap_channel") ? server.arg("ap_channel").toInt() : g.wifi_channel;
        const String ap_pass = server.hasArg("ap_pass") ? server.arg("ap_pass") : String();

        if (ap_ssid.length() == 0 || ap_ssid.length() > 20 || ap_channel < 1 || ap_channel > 13 ||
            (ap_pass.length() > 0 && (ap_pass.length() < 8 || ap_pass.length() > 20))) {
            server.send(400, "text/plain", "Invalid Wi-Fi AP configuration");
            return;
        }
        if (!tdm2421_mqtt.saveConfig(cfg)) {
            server.send(400, "text/plain", "Invalid or unsaved LTE/MQTT configuration");
            return;
        }

        // Configure the actual OpenDroneID Basic ID from the local dashboard.
        // For this TDM2421 build the aircraft is a multirotor and the configured
        // identifier is treated as a serial-number Basic ID.
        if (server.hasArg("basic_id")) {
            String bid = server.arg("basic_id");
            bid.trim();
            if (bid.length() > 20) {
                server.send(400, "text/plain", "Basic ID must be at most 20 characters");
                return;
            }
            if (bid.length() > 0) {
                g.set_by_name_uint8("UAS_TYPE", ODID_UATYPE_HELICOPTER_OR_MULTIROTOR);
                g.set_by_name_uint8("UAS_ID_TYPE", ODID_IDTYPE_SERIAL_NUMBER);
                g.set_by_name_string("UAS_ID", bid.c_str());
            }
        }

        g.set_by_name_string("WIFI_SSID", ap_ssid.c_str());
        g.set_by_name_uint8("WIFI_CHANNEL", (uint8_t)ap_channel);
        if (ap_pass.length() > 0) {
            g.set_by_name_string("WIFI_PASSWORD", ap_pass.c_str());
        }

        server.send(200, "text/html",
                    "<!doctype html><meta name='viewport' content='width=device-width,initial-scale=1'>"
                    "<h2>Configuration saved</h2><p>TDM2421 is rebooting. Reconnect to the configured Wi-Fi AP, then open <b>http://192.168.4.1</b>.</p>");
        delay(800);
        ESP.restart();
    });
}
#endif

/*
  init web server
 */
void WebInterface::init(void)
{
    Serial.printf("WAP start %s %s\r\n", g.wifi_ssid, g.wifi_password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.printf("WAP IP %s\r\n", myIP.toString().c_str());

#if defined(BOARD_TDM2421)
    setupTDM2421Routes();
#endif

    server.addHandler(&AJAX_Handler);
    server.addHandler(&ROMFS_Handler);

    /* handling uploading firmware file */
    server.on("/update", HTTP_POST, []() {
        if (Update.hasError()) {
            server.sendHeader("Connection", "close");
            server.send(500, "text/plain", "FAIL");
            Serial.printf("Update Failed: Update function has errors\r\n");
            delay(5000);
        } else {
            server.sendHeader("Connection", "close");
            server.send(200, "text/plain", "OK");
            Serial.printf("Update Success: \r\nRebooting...\r\n");
            delay(1000);
            ESP.restart();
        }
    }, [this]() {
        HTTPUpload& upload = server.upload();
        static const esp_partition_t* partition_new_firmware = esp_ota_get_next_update_partition(NULL);
        if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("Update: %s\r\n", upload.filename.c_str());
            lead_len = 0;

            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (lead_len < sizeof(lead_bytes)) {
                uint32_t n = sizeof(lead_bytes)-lead_len;
                if (n > upload.currentSize) n = upload.currentSize;
                memcpy(&lead_bytes[lead_len], upload.buf, n);
                lead_len += n;
            }
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            uint32_t extra = SPI_FLASH_SEC_SIZE+1;
            while (extra--) {
                uint8_t ff = 0xff;
                Update.write(&ff, 1);
            }
            if (!CheckFirmware::check_OTA_next(partition_new_firmware, lead_bytes, lead_len)) {
                Serial.printf("Update Failed: firmware checks have errors\r\n");
                server.sendHeader("Connection", "close");
                server.send(500, "text/plain", "FAIL");
                delay(5000);
            } else if (Update.end(true)) {
                Serial.printf("Update Success: %u\r\nRebooting...\r\n", upload.totalSize);
                server.sendHeader("Connection", "close");
                server.send(200, "text/plain", "OK");
            } else {
                Update.printError(Serial);
                Serial.printf("Update Failed: Update.end function has errors\r\n");
                server.sendHeader("Connection", "close");
                server.send(500, "text/plain", "FAIL");
                delay(5000);
            }
        }
    });
    Serial.printf("WAP started\r\n");
    server.begin();
}

void WebInterface::update()
{
    if (!initialised) {
        init();
        initialised = true;
    }
    server.handleClient();
}
