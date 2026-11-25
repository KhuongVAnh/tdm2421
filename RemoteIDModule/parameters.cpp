#include "options.h"
#include <Arduino.h>
#include "parameters.h"
#include <nvs_flash.h>
#if defined(BOARD_TDM2421)
#include <esp_flash.h>
#endif
#include <string.h>
#include "romfs.h"
#include "util.h"

#if defined(BOARD_TDM2421)
// Arduino-ESP32 2.0.3 esp32c3 SDK is compiled with CONFIG_ESPTOOLPY_FLASHMODE_QIO=y,
// so esp_flash_default_chip->read_mode defaults to SPI_FLASH_QIO (5) even when the
// image is built in DIO mode. On TDM2421 (flash 0x464016), QIO reads corrupt data
// via esp_flash_read/nvs_flash, causing NVS to wipe on every boot. Force DIO before
// initArduino() runs nvs_flash_init().
__attribute__((constructor)) static void tdm2421_fix_flash_read_mode(void)
{
    if (esp_flash_default_chip != nullptr && esp_flash_default_chip->read_mode > SPI_FLASH_DIO) {
        esp_flash_default_chip->read_mode = SPI_FLASH_DIO;
        esp_flash_init(esp_flash_default_chip);
    }
}
#endif

Parameters g;
static nvs_handle handle;
static bool nvs_ready = false;

static void commit_nvs_write(esp_err_t err, const char *name)
{
    if (!nvs_ready) {
        Serial.printf("NVS not ready when writing %s\r\n", name);
        return;
    }
    if (err != ESP_OK) {
        Serial.printf("NVS set %s failed: %d\r\n", name, (int)err);
        return;
    }
    const esp_err_t commit_err = nvs_commit(handle);
    if (commit_err != ESP_OK) {
        Serial.printf("NVS commit %s failed: %d\r\n", name, (int)commit_err);
    }
}

const Parameters::Param Parameters::params[] = {
    { "LOCK_LEVEL",        Parameters::ParamType::INT8,  (const void*)&g.lock_level,       0, -1, 2 },
    { "CAN_NODE",          Parameters::ParamType::UINT8,  (const void*)&g.can_node,         0, 0, 127 },
#if defined(PIN_CAN_TERM)
    { "CAN_TERMINATE",     Parameters::ParamType::UINT8,  (const void*)&g.can_term,        0, 0, 1 },
#endif
#if defined(BOARD_TDM2421)
    // TDM2421 is a self-contained Remote ID transmitter.  These defaults must
    // live in the parameter table (not only as C++ member initializers), because
    // Parameters::load_defaults() overwrites all numeric members at boot.
    { "UAS_TYPE",          Parameters::ParamType::UINT8,  (const void*)&g.ua_type,          2, 0, 15 },
    { "UAS_ID_TYPE",       Parameters::ParamType::UINT8,  (const void*)&g.id_type,          1, 0, 4 },
#else
    { "UAS_TYPE",          Parameters::ParamType::UINT8,  (const void*)&g.ua_type,          0, 0, 15 },
    { "UAS_ID_TYPE",       Parameters::ParamType::UINT8,  (const void*)&g.id_type,          0, 0, 4 },
#endif
    { "UAS_ID",            Parameters::ParamType::CHAR20, (const void*)&g.uas_id[0],        0, 0, 0 },
    { "UAS_TYPE_2",        Parameters::ParamType::UINT8,  (const void*)&g.ua_type_2,          0, 0, 15 },
    { "UAS_ID_TYPE_2",     Parameters::ParamType::UINT8,  (const void*)&g.id_type_2,          0, 0, 4 },
    { "UAS_ID_2",          Parameters::ParamType::CHAR20, (const void*)&g.uas_id_2[0],        0, 0, 0 },
    { "BAUDRATE",          Parameters::ParamType::UINT32, (const void*)&g.baudrate,         57600, 9600, 921600 },
    { "WIFI_NAN_RATE",     Parameters::ParamType::FLOAT,  (const void*)&g.wifi_nan_rate,    5, 0, 5 },
    { "WIFI_BCN_RATE",     Parameters::ParamType::FLOAT,  (const void*)&g.wifi_beacon_rate,    5, 0, 5 },
    { "WIFI_POWER",        Parameters::ParamType::FLOAT,  (const void*)&g.wifi_power,       20, 2, 20 },
    { "BT4_RATE",          Parameters::ParamType::FLOAT,  (const void*)&g.bt4_rate,         1, 0, 5 },
    { "BT4_POWER",         Parameters::ParamType::FLOAT,  (const void*)&g.bt4_power,        18, -27, 18 },
    { "BT5_RATE",          Parameters::ParamType::FLOAT,  (const void*)&g.bt5_rate,         1, 0, 5 },
    { "BT5_POWER",         Parameters::ParamType::FLOAT,  (const void*)&g.bt5_power,        18, -27, 18 },
    { "WEBSERVER_EN",      Parameters::ParamType::UINT8,  (const void*)&g.webserver_enable, 1, 0, 1 },
    { "WIFI_SSID",         Parameters::ParamType::CHAR20, (const void*)&g.wifi_ssid, },
    { "WIFI_PASSWORD",     Parameters::ParamType::CHAR20, (const void*)&g.wifi_password,    0, 0, 0, PARAM_FLAG_PASSWORD, 8 },
    { "WIFI_CHANNEL",      Parameters::ParamType::UINT8,  (const void*)&g.wifi_channel,    6, 1, 13 },
    { "BCAST_POWERUP",     Parameters::ParamType::UINT8,  (const void*)&g.bcast_powerup,    1, 0, 1 },
    { "PUBLIC_KEY1",       Parameters::ParamType::CHAR64, (const void*)&g.public_keys[0], },
    { "PUBLIC_KEY2",       Parameters::ParamType::CHAR64, (const void*)&g.public_keys[1], },
    { "PUBLIC_KEY3",       Parameters::ParamType::CHAR64, (const void*)&g.public_keys[2], },
    { "PUBLIC_KEY4",       Parameters::ParamType::CHAR64, (const void*)&g.public_keys[3], },
    { "PUBLIC_KEY5",       Parameters::ParamType::CHAR64, (const void*)&g.public_keys[4], },
    { "MAVLINK_SYSID",     Parameters::ParamType::UINT8,  (const void*)&g.mavlink_sysid,    0, 0, 254 },
    { "OPTIONS",           Parameters::ParamType::UINT8,  (const void*)&g.options,          0, 0, 254 },
    { "TO_DEFAULTS",     Parameters::ParamType::UINT8,  (const void*)&g.to_factory_defaults,    0, 0, 1 }, //if set to 1, reset to factory defaults and make 0.
    { "DONE_INIT",         Parameters::ParamType::UINT8,  (const void*)&g.done_init,        0, 0, 0, PARAM_FLAG_HIDDEN},
    { "",                  Parameters::ParamType::NONE,   nullptr,  },
};

/*
  get count of parameters capable of being converted to load
 */
uint16_t Parameters::param_count_float(void)
{
    uint16_t count = 0;
    for (const auto &p : params) {
        if (p.flags & PARAM_FLAG_HIDDEN) {
            continue;
        }
        switch (p.ptype) {
        case ParamType::UINT8:
        case ParamType::INT8:
        case ParamType::UINT32:
        case ParamType::FLOAT:
            count++;
            break;
        }
    }
    // remove 1 for DONE_INIT
    return count-1;
}

/*
  get index of parameter counting only those capable of representation as float
 */
int16_t Parameters::param_index_float(const Parameters::Param *f)
{
    uint16_t count = 0;
    for (const auto &p : params) {
        if (p.flags & PARAM_FLAG_HIDDEN) {
            continue;
        }
        switch (p.ptype) {
        case ParamType::UINT8:
        case ParamType::INT8:
        case ParamType::UINT32:
        case ParamType::FLOAT:
            if (&p == f) {
                return count;
            }
            count++;
            break;
        }
    }
    return -1;
}

/*
  find by name
 */
const Parameters::Param *Parameters::find(const char *name)
{
    for (const auto &p : params) {
        if (strcmp(name, p.name) == 0) {
            return &p;
        }
    }
    return nullptr;
}

/*
  find by index
 */
const Parameters::Param *Parameters::find_by_index(uint16_t index)
{
    if (index >= ARRAY_SIZE(params)-2) {
        return nullptr;
    }
    return &params[index];
}

/*
  find by index
 */
const Parameters::Param *Parameters::find_by_index_float(uint16_t index)
{
    uint16_t count = 0;
    for (const auto &p : params) {
        if (p.flags & PARAM_FLAG_HIDDEN) {
            continue;
        }
        switch (p.ptype) {
        case ParamType::UINT8:
        case ParamType::INT8:
        case ParamType::UINT32:
        case ParamType::FLOAT:
            if (index == count) {
                return &p;
            }
            count++;
            break;
        }
    }
    return nullptr;
}

void Parameters::Param::set_uint8(uint8_t v) const
{
    auto *p = (uint8_t *)ptr;
    *p = v;
    commit_nvs_write(nvs_set_u8(handle, name, *p), name);
    if (strcmp(name, "TO_DEFAULTS") == 0) {
        if (v == 1) {
            nvs_flash_erase();
            esp_restart();
        }
    }
}

void Parameters::Param::set_int8(int8_t v) const
{
    auto *p = (int8_t *)ptr;
    *p = v;
    commit_nvs_write(nvs_set_i8(handle, name, *p), name);
}

void Parameters::Param::set_uint32(uint32_t v) const
{
    auto *p = (uint32_t *)ptr;
    *p = v;
    commit_nvs_write(nvs_set_u32(handle, name, *p), name);
}

void Parameters::Param::set_float(float v) const
{
    auto *p = (float *)ptr;
    *p = v;
    union {
        float f;
        uint32_t u32;
    } u;
    u.f = v;
    commit_nvs_write(nvs_set_u32(handle, name, u.u32), name);
}

void Parameters::Param::set_char20(const char *v) const
{
    if (min_len > 0 && strlen(v) < min_len) {
        return;
    }
    memset((void*)ptr, 0, 21);
    strncpy((char *)ptr, v, 20);
    commit_nvs_write(nvs_set_str(handle, name, (const char *)ptr), name);
}

void Parameters::Param::set_char64(const char *v) const
{
    if (min_len > 0 && strlen(v) < min_len) {
        return;
    }
    memset((void*)ptr, 0, 65);
    strncpy((char *)ptr, v, 64);
    commit_nvs_write(nvs_set_str(handle, name, (const char *)ptr), name);
}

uint8_t Parameters::Param::get_uint8() const
{
    const auto *p = (const uint8_t *)ptr;
    return *p;
}

int8_t Parameters::Param::get_int8() const
{
    const auto *p = (const int8_t *)ptr;
    return *p;
}

uint32_t Parameters::Param::get_uint32() const
{
    const auto *p = (const uint32_t *)ptr;
    return *p;
}

float Parameters::Param::get_float() const
{
    const auto *p = (const float *)ptr;
    return *p;
}

const char *Parameters::Param::get_char20() const
{
    const char *p = (const char *)ptr;
    return p;
}

const char *Parameters::Param::get_char64() const
{
    const char *p = (const char *)ptr;
    return p;
}

/*
  get parameter as a float
 */
bool Parameters::Param::get_as_float(float &v) const
{
    switch (ptype) {
        case ParamType::UINT8:
            v = float(get_uint8());
            break;
        case ParamType::INT8:
            v = float(get_int8());
            break;
        case ParamType::UINT32:
            v = float(get_uint32());
            break;
        case ParamType::FLOAT:
            v = get_float();
            break;
    default:
        return false;
    }
    return true;
}

/*
  set parameter from a float
 */
void Parameters::Param::set_as_float(float v) const
{
    switch (ptype) {
        case ParamType::UINT8:
            set_uint8(uint8_t(v));
            break;
        case ParamType::INT8:
            set_int8(int8_t(v));
            break;
        case ParamType::UINT32:
            set_uint32(uint32_t(v));
            break;
        case ParamType::FLOAT:
            set_float(v);
            break;
    }
}


/*
  load defaults from parameter table
 */
void Parameters::load_defaults(void)
{
    for (const auto &p : params) {
        switch (p.ptype) {
        case ParamType::UINT8:
            *(uint8_t *)p.ptr = uint8_t(p.default_value);
            break;
        case ParamType::INT8:
            *(int8_t *)p.ptr = int8_t(p.default_value);
            break;
        case ParamType::UINT32:
            *(uint32_t *)p.ptr = uint32_t(p.default_value);
            break;
        case ParamType::FLOAT:
            *(float *)p.ptr = p.default_value;
            break;
        }
    }
}

void Parameters::init(void)
{
    load_defaults();

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.printf("NVS needs erase (%d), erasing...\r\n", (int)nvs_err);
        if (nvs_flash_erase() == ESP_OK) {
            nvs_err = nvs_flash_init();
        }
    }
    const esp_err_t open_err = (nvs_err == ESP_OK) ? nvs_open("storage", NVS_READWRITE, &handle) : nvs_err;
    if (nvs_err != ESP_OK || open_err != ESP_OK) {
        nvs_ready = false;
        Serial.printf("NVS init failed: %d open=%d\r\n", (int)nvs_err, (int)open_err);
    } else {
        nvs_ready = true;
    }
    // load values from NVS
    for (const auto &p : params) {
        switch (p.ptype) {
        case ParamType::UINT8:
            nvs_get_u8(handle, p.name, (uint8_t *)p.ptr);
            break;
        case ParamType::INT8:
            nvs_get_i8(handle, p.name, (int8_t *)p.ptr);
            break;
        case ParamType::UINT32:
            nvs_get_u32(handle, p.name, (uint32_t *)p.ptr);
            break;
        case ParamType::FLOAT:
            nvs_get_u32(handle, p.name, (uint32_t *)p.ptr);
            break;
        case ParamType::CHAR20: {
            size_t len = 21;
            nvs_get_str(handle, p.name, (char *)p.ptr, &len);
            break;
        }
        case ParamType::CHAR64: {
            size_t len = 65;
            nvs_get_str(handle, p.name, (char *)p.ptr, &len);
            break;
        }
        }
    }

#if defined(BOARD_TDM2421)
    // Migration for boards that were flashed with an older TDM2421 build.
    // Old firmware stored UAS_TYPE=0/UAS_ID_TYPE=0 (and sometimes an empty
    // UAS_ID) in NVS.  NVS is intentionally preserved by normal firmware
    // flashing, so those stale values otherwise override the new defaults and
    // BasicIDValid[0] never becomes true.
    if (g.ua_type == 0) {
        set_by_name_uint8("UAS_TYPE", 2); // helicopter / multirotor
    }
    if (g.id_type == 0) {
        set_by_name_uint8("UAS_ID_TYPE", 1); // serial number
    }
    if (strlen(g.uas_id) == 0) {
        set_by_name_string("UAS_ID", "1866ASDD0000VT5");
    }
#endif

    if (strlen(g.wifi_ssid) == 0) {
        uint8_t mac[6] {};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(wifi_ssid, 20, "RID_%02x%02x%02x%02x%02x%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    }
    if (g.to_factory_defaults == 1) {
        //should not happen, but in case the parameter is still set to 1, erase flash and reboot
        nvs_flash_erase();
        esp_restart();
    }

    if (g.done_init == 0) {
        set_by_name_uint8("DONE_INIT", 1);
        // setup public keys
        set_by_name_char64("PUBLIC_KEY1", ROMFS::find_string("public_keys/ArduPilot_public_key1.dat"));
        set_by_name_char64("PUBLIC_KEY2", ROMFS::find_string("public_keys/ArduPilot_public_key2.dat"));
#if defined(BOARD_BLUEMARK_DB200) || defined(BOARD_BLUEMARK_DB110) || defined(BOARD_BLUEMARK_DB202) || defined(BOARD_BLUEMARK_DB210) || defined(BOARD_BLUEMARK_DB203)
        set_by_name_char64("PUBLIC_KEY3", ROMFS::find_string("public_keys/BlueMark_public_key1.dat"));
#elif defined(BOARD_CUAV_RID)
        set_by_name_char64("PUBLIC_KEY3", ROMFS::find_string("public_keys/CUAV_public_key1.dat"));
#else
        set_by_name_char64("PUBLIC_KEY3", ROMFS::find_string("public_keys/ArduPilot_public_key3.dat"));
#endif

    }
}

/*
  check if BasicID info is filled in with parameters
 */
bool Parameters::have_basic_id_info(void) const
{
    return strlen(g.uas_id) > 0 && g.id_type > 0 && g.ua_type > 0;
}

bool Parameters::have_basic_id_2_info(void) const
{
    return strlen(g.uas_id_2) > 0 && g.id_type_2 > 0 && g.ua_type_2 > 0;
}

bool Parameters::set_by_name_uint8(const char *name, uint8_t v)
{
    const auto *f = find(name);
    if (!f) {
        return false;
    }
    f->set_uint8(v);
    return true;
}

bool Parameters::set_by_name_int8(const char *name, int8_t v)
{
    const auto *f = find(name);
    if (!f) {
        return false;
    }
    f->set_int8(v);
    return true;
}

bool Parameters::set_by_name_char64(const char *name, const char *s)
{
    const auto *f = find(name);
    if (!f) {
        return false;
    }
    f->set_char64(s);
    return true;
}

bool Parameters::set_by_name_string(const char *name, const char *s)
{
    const auto *f = find(name);
    if (!f) {
        return false;
    }
    switch (f->ptype) {
        case ParamType::UINT8:
            f->set_uint8(uint8_t(strtoul(s, nullptr, 0)));
            return true;
        case ParamType::INT8:
            f->set_int8(int8_t(strtoul(s, nullptr, 0)));
            return true;
        case ParamType::UINT32:
            f->set_uint32(strtoul(s, nullptr, 0));
            return true;
        case ParamType::FLOAT:
            f->set_float(atof(s));
            return true;
        case ParamType::CHAR20:
            f->set_char20(s);
            return true;
        case ParamType::CHAR64:
            f->set_char64(s);
            return true;
    }
    return false;
}

/*
  return a public key
 */
bool Parameters::get_public_key(uint8_t i, uint8_t key[32]) const
{
    if (i >= MAX_PUBLIC_KEYS) {
        return false;
    }
    const char *b64_key = g.public_keys[i].b64_key;
    const char *ktype = "PUBLIC_KEYV1:";
    if (strncmp(b64_key, ktype, strlen(ktype)) != 0) {
        return false;
    }
    b64_key += strlen(ktype);
    int32_t out_len = base64_decode(b64_key, key, 32);
    if (out_len != 32) {
        return false;
    }
    return true;
}

bool Parameters::no_public_keys(void) const
{
    for (uint8_t i=0; i<MAX_PUBLIC_KEYS; i++) {
        uint8_t key[32];
        if (get_public_key(i, key)) {
            return false;
        }
    }
    return true;
}

bool Parameters::set_public_key(uint8_t i, const uint8_t key[32])
{
    if (i >= MAX_PUBLIC_KEYS) {
        return false;
    }
    char *s = base64_encode(key, PUBLIC_KEY_LEN);
    if (s == nullptr) {
        return false;
    }
    char name[] = "PUBLIC_KEYx";
    name[strlen(name)-2] = '1'+i;
    bool ret = set_by_name_char64(name, s);
    delete[] s;
    return ret;
}

bool Parameters::remove_public_key(uint8_t i)
{
    if (i >= MAX_PUBLIC_KEYS) {
        return false;
    }
    char name[] = "PUBLIC_KEYx";
    name[strlen(name)-2] = '1'+i;
    return set_by_name_char64(name, "");
}
