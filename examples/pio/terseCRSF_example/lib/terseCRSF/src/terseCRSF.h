#include <Arduino.h>
#include <iostream>
#include <string>

#define log             Serial
#define crsfSerial      Serial1
#if defined SEND_SBUS
  #define sbusSerial    Serial2
#endif

//#define RC_BUILD       // ELSE TELEMETRY BUILD

#if defined RC_BUILD
  //#define SEND_SBUS
#endif

#define MAJOR_VER          0
#define MINOR_VER          0
#define PATCH_LEV          2 

//=========  D E M O   M A C R O S  ========
#define DEMO_PWM_VALUES
//#define DEMO_SBUS
//#define DEMO_CRSF_GPS
//#define DEMO_CRSF_BATTERY
//#define DEMO_CRSF_LINK
//#define DEMO_CRSF_ATTITUDE
//#define DEMO_CRSF_FLIGHT_MODE
//#define SHOW_BUFFER
//#define SHOW_LINK_STATS
//#define SHOW_LOOP_PERIOD
//==========================================
#define  RC_INPUT_MAX_CHANNELS 8  //18

#define RADS2DEGS 180 / PI

// Frame id
#define GPS_ID                         0x02
#define CF_VARIO_ID                    0x07
#define BATTERY_ID                     0x08
#define BARO_ALT_ID                    0x09
#define HEARTBEAT_ID                   0x0B  // added
#define LINK_ID                        0x14  // link statistics
#define CHANNELS_ID                    0x16
#define LINK_RX_ID                     0x1C
#define LINK_TX_ID                     0x1D
#define ATTITUDE_ID                    0x1E
#define FLIGHT_MODE_ID                 0x21
#define PING_DEVICES_ID                0x28
#define DEVICE_INFO_ID                 0x29
#define REQUEST_SETTINGS_ID            0x2A
#define COMMAND_ID                     0x32
#define RADIO_ID                       0x3A

#define CRSF_BUFFER_SIZE               64
#define CRSF_SBUS_BUFFER_SIZE          25   
#define CRSF_TEL_SYNC_BYTE             0xC8 
#define CRSF_RC_SYNC_BYTE1             24   // 0x18 these are repeating frame length values
#define CRSF_RC_SYNC_BYTE2             22   // 0x16 byte pair close to unique

#if defined RC_BUILD
    #define crfs_invert     true
    #define crfs_rxPin      13      // YELLOW rx from FC WHITE tx
    #define crfs_txPin      14      // GREEN tx to FC LIGHT BLUE 
    #define sbus_rxPin      -1      // RX1 SBUS not used - don't care 
    #define sbus_txPin      15      // TX1 SBUS out 
    #define sbusFast        false
    #define sbusInvert      true   
#else
    #define crfs_invert     false
    #define crfs_rxPin      27      //16 YELLOW rx from GREEN FC tx
    #define crfs_txPin      17      // GREEN tx to YELLOW FC rx    
#endif

class CRSF
{
  // Data members
public:
// quote "416KBaud that CRSF uses", ELRS uses 420000
#if defined RC_BUILD 
const uint32_t crfs_baud = 416000;  // works for both
#else
const uint32_t crfs_baud = 420000;  // works for both
#endif

#define MAX_RC_BYTES  22
uint8_t   max_rc_bytes = MAX_RC_BYTES;
uint8_t   max_ch = RC_INPUT_MAX_CHANNELS;
uint8_t   frame_lth = 0;
uint16_t  rc_ch_cnt = 0;
uint8_t   crsf_buf[CRSF_BUFFER_SIZE] {};
uint8_t   rc_bytes[22]; // RC_bytes(22) - note: just the RC bytes, not the full sbus
uint8_t   sb_bytes[25]; // Header(1) + RC_bytes(22) + status(1)(los+fs) + footer(1)
uint16_t  pwm_val[RC_INPUT_MAX_CHANNELS] {};  

uint8_t   crsf_id = 0;
uint8_t   crsf_lth = 0;

/* GPS ID:0x02 */
int32_t     gps_lat = 0;              // deg * 1e7
int32_t     gps_lon = 0;
float       gpsF_lat = 0;             // deg
float       gpsF_lon = 0.0;
uint16_t    gps_groundspeed = 0;
float       gpsF_groundspeed = 0.0;   // km/hr
uint16_t    gps_heading = 0;  
float       gpsF_heading = 0.0;       // deg
uint16_t    gps_altitude = 0;         // metres, 1000m offset
uint8_t     gps_sats = 0;

/* Battery ID:0x08 */
uint16_t    bat_voltage = 0;           // mV * 100
float       batF_voltage = 0.0;        // volts
uint16_t    bat_current = 0;           // mA * 100
float       batF_current = 0.0;        // amps
uint32_t    bat_fuel_drawn = 0;        // uint24_t    mAh drawn
float       batF_fuel_drawn = 0.0;     // Ah drawn
uint8_t     bat_remaining = 0;         // percent

/* Link Statistics ID 0x14*/
uint8_t     link_up_rssi_ant_1 = 0;         // dBm * -1
uint8_t     link_up_rssi_ant_2 = 0;         // dBm * -1
uint8_t     link_up_quality = 0;            // packet_success_rate (%)
int8_t      link_up_snr = 0;                // db
uint8_t     link_diversity_active_ant = 0;  //(enum ant_1 = 0, ant_2)
uint8_t     link_rf_mode = 0;               //(enum 4fps = 0, 50fps, 150hz)
uint8_t     link_up_tx_power = 0;           //(enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW)
uint8_t     link_dn_rssi = 0;               // RSSI(dBm * -1)
uint8_t     link_dn_quality = 0;            // packet_success_rate (%)
int8_t      link_dn_snr = 0;                // db

    /* Attitude ID:0x1E */
    int16_t atti_pitch = 0;            // rad / 10000
float       attiF_pitch = 0.0;         // deg
int16_t     atti_roll = 0;             // rad / 10000
float       attiF_roll = 0.0;          // deg
int16_t     atti_yaw = 0;              // rad / 10000)
float       attiF_yaw = 0.0;           // deg

/* Flight Mode ID:0x21*/
uint8_t     flight_mode_lth = 0;
std::string flightMode;

//= (char *)"ACRO";

// Member function prototypes

private:

// own link stats
  uint32_t frames_read = 0;
  uint32_t good_frames = 0;
  uint32_t crc_errors = 0;
  uint32_t frame_errors = 0;
  uint16_t unknown_ids = 0;

public:
  bool initialise();
  bool readCrsfFrame(uint8_t &lth);
  uint8_t decodeTelemetry(uint8_t *_buf);
  void decodeRC();
  void printByte(byte b, char delimiter);
  void printBytes(uint8_t *buf, uint8_t len);
  void printPWM(uint16_t *ch, uint8_t num_of_channels);
  void printLinkStats();
private:
  uint8_t crc8_dvb_s2(uint8_t, unsigned char);
  uint8_t crc8_dvb_s2_update(uint8_t, const void *, uint32_t);
  int32_t bytes2int32(uint8_t *byt);
  uint16_t bytes2uint16(uint8_t *byt);;
  int16_t bytes2int16(uint8_t *byt);
  uint16_t wrap360(int16_t);
  bool fixBadRc(uint8_t *);
  void prepSBUS(uint8_t *rc_buf, uint8_t *sb_buf, bool _los, bool _failsafe);
  bool bytesToPWM(uint8_t *sb_byte, uint16_t *ch_val, uint8_t max_ch);
  void pwmToBytes(uint16_t *in_pwm, uint8_t *rc_byt, uint8_t max_ch);
};  // end of class