#include <Arduino.h>
#include <iostream>
#include <string>

#define log             Serial
#define crsfSerial      Serial2
#if defined SEND_SBUS
  #define sbusSerial    Serial2
#endif

//#define RC_BUILD       // ELSE TELEMETRY BUILD
#if defined RC_BUILD
  //#define SEND_SBUS
#endif

#define MAJOR_VERSION      0
#define MINOR_VERSION      0
#define PATCH_LEVEL        1 

//=========  D E M O   M A C R O S  ========
#define DEMO_PWM_VALUES
#define DEMO_SBUS
#define DEMO_CRSF_GPS
#define DEMO_CRSF_BATTERY
#define DEMO_CRSF_ATTITUDE
#define DEMO_CRSF_FLIGHT_MODE
//#define SHOW_BUFFER
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
#define LINK_ID                        0x14
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
    #define crfs_rxPin      16      // YELLOW rx from FC WHITE tx
    #define crfs_txPin      17      // GREEN tx to FC LIGHT BLUE   
#endif

class CRSF
{
  // Data members
public:
// quote "416KBaud that CRSF uses", ELRS uses 420000
const uint32_t crfs_baud = 420000;  // works for both

uint8_t   max_ch = RC_INPUT_MAX_CHANNELS;  
uint8_t   frame_lth = 0;
uint16_t  rc_ch_cnt = 0;
uint8_t   crsf_buf[CRSF_BUFFER_SIZE] {};

uint8_t   rc_bytes[22]; // RC_bytes(22) - note: just the RC bytes, not the full sbus
uint8_t   sb_bytes[25]; // Header(1) + RC_bytes(22) + status(1)(los+fs) + footer(1)
uint16_t  pwm_val[RC_INPUT_MAX_CHANNELS] {};  

uint8_t   crsf_id = 0;
uint8_t   crsf_lth = 0;

/* GPS ID:02 */
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

/* Battery ID:08 */
uint16_t    bat_voltage = 0;           // mV * 100
float       batF_voltage = 0.0;        // volts
uint16_t    bat_current = 0;           // mA * 100
float       batF_current = 0.0;        // amps
uint32_t    bat_fuel_drawn = 0;        // uint24_t    mAh drawn
float       batF_fuel_drawn = 0.0;     // Ah drawn
uint8_t     bat_remaining = 0;         // percent
                         
/* Attitude ID:1E */
int16_t     atti_pitch = 0;            // rad / 10000
float       attiF_pitch = 0.0;         // deg
int16_t     atti_roll = 0;             // rad / 10000
float       attiF_roll = 0.0;          // deg
int16_t     atti_yaw = 0;              // rad / 10000)
float       attiF_yaw = 0.0;           // deg

/* Flight Mode */
uint8_t     flight_mode_lth = 0;
std::string flightMode;

//= (char *)"ACRO";

// Member function prototypes

private:

public:
  bool initialise();
  bool readCrsfFrame(uint8_t &lth);
  uint8_t decodeTelemetry(uint8_t *_buf);
  void decodeRC();
  void printByte(byte b, char delimiter);
  void printBytes(uint8_t *buf, uint8_t len);
  void printPWM(uint16_t *ch, uint8_t num_of_channels);
  
private:
  int32_t bytes2int32(uint8_t *byt);
  uint16_t bytes2uint16(uint8_t *byt);;
  int16_t bytes2int16(uint8_t *byt);
  void prepSBUS(uint8_t *rc_buf, uint8_t *sb_buf, bool _los, bool _failsafe);
  bool bytesToPWM(uint8_t *sb_byte, uint16_t *ch_val, uint8_t max_ch);
  void pwmToBytes(uint16_t *in_pwm, uint8_t *rc_byt, uint8_t max_ch);

};  // end of class