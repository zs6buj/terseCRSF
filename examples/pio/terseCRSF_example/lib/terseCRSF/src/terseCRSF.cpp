
#include <terseCRSF.h>

//=======================================================
bool CRSF::initialise()
{
  log.print("\nterseCRSF by zs6buj");
  log.printf(" version:%d.%02d.%02d\n", MAJOR_VER, MINOR_VER, PATCH_LEV);
#if defined RC_BUILD
  log.println("RC_BUILD");
#else
  log.println("TELEMETRY_BUILD");
#endif
  crsfSerial.begin(crfs_baud, SERIAL_8N1, crfs_rxPin, crfs_txPin, crfs_invert);
  log.printf("CRFS baud:%u  rxPin:%u  txPin:%u  invert:%u\n", crfs_baud, crfs_rxPin, crfs_txPin, crfs_invert);
  #if defined SEND_SBUS
    if(sbusInvert)
    {
      log.println("SBUS inverted (idle low)");
    } else 
    {
      log.println("SBUS not inverted (idle high) ");
    }
    uint32_t sbus_baud = 0;
    if (sbusFast)
    {
      sbus_baud = 200000;
      log.print("Fast 200000b/s");
    }
    else
    {
      sbus_baud = 100000;  
      log.print("Normal 100000 b/s"); 
    }  
    log.printf(" SBUS sending on pin tx:%d\n", sbus_txPin);
    delay(100);
    sbusSerial.begin(sbus_baud, SERIAL_8E2, sbus_rxPin, sbus_txPin, sbusInvert); 
    delay(100);
  #endif
return true;
}
//=======================================================
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}
//=======================================================
uint8_t crc8_dvb_s2_sbuf_accum(const void *data, uint8_t frm_lth)
{
  uint8_t crc = 0;
  const uint8_t *p = (const uint8_t *)data;
  const uint8_t *pend = p + frm_lth;

  for (; p != pend; p++)
  {
    //log.printf("crc *p 0x%2X\n", *p);
    crc = crc8_dvb_s2(crc, *p);
    }
    return crc;
}
//=======================================================
int32_t CRSF::bytes2int32(uint8_t *byt)
{
  return ((byt[3] << 0) & 0xFF) + ((byt[2] << 8) & 0xFFFF) + ((byt[1] << 16) & 0xFFFFFF) + ((byt[0] << 24) & 0xFFFFFFFF);
}
//=======================================================
uint16_t CRSF::bytes2uint16(uint8_t *byt)
{
  return ((byt[1] << 0) & 0xFF) + ((byt[0] << 8) & 0xFFFF);
}
//=======================================================
int16_t CRSF::bytes2int16(uint8_t *byt)
{
  return ((byt[1] << 0) & 0xFF) + ((byt[0] << 8) & 0xFFFF);
}
//=======================================================
void CRSF::printByte(byte b, char delimiter)
{
  if (b <= 0xf)
    log.print("0");
    log.print(b, HEX);
    log.write(delimiter);
}
//========================================================
void CRSF::printPWM(uint16_t *ch, uint8_t num_of_channels)
{
  log.print("PWM: ");
  for (int i = 0; i < num_of_channels; i++)
  {
    log.printf("%2d:%4d  ", i + 1, *(ch + i));
  }
  // log.printf("  rssi:%d", pwm_rssi );
  log.println();
}
//========================================================
void CRSF::printBytes(uint8_t *buf, uint8_t len)
{
  //log.print("bytes: ");
  for (int i = 0; i < len; i++)
  {
    printByte(buf[i], ' ');
  }
  log.println();
}
//========================================================
void CRSF::printLinkStats()
{
#if defined SHOW_LINK_STATS
  static uint32_t  error_millis = 0;
  if ((millis() - error_millis) > 1.2E5)  // 2 minutes
  {
    error_millis = millis();
    log.printf("frames_read:%u  good_frames:%u  crc_errors:%u  frame_errors:%u  unknown_ids:%u\n",  frames_read, good_frames, crc_errors, frame_errors, unknown_ids);
  }
#endif
}
//===================================================================
uint16_t CRSF::wrap360(int16_t ang)
{
  if (ang < 0)
    ang += 360;
  if (ang > 359)
    ang -= 360;
  return ang;
}
//===================================================================
void CRSF::prepSBUS(uint8_t *rc_buf, uint8_t *sb_buf, bool _los, bool _failsafe)
{
  uint8_t statusByte = 0x00;
  if (_los)
  { // loss of signal
    statusByte |= 0x04;
  }
  if (_failsafe)
  {
    statusByte |= 0x08;
  }

  sb_buf[0] = 0x0F; // sbus byte [0], header byte
  for (int i = 0; i < 22; i++)
  {                              // rc byte   [0] thru [21]
    sb_buf[i + 1] = rc_buf[i]; // sbus byte [1] thru [22]
  }

  sb_buf[23] = statusByte; // sbus byte [23], status byte
  sb_buf[24] = 0x00;       // sbus byte [24], footer byte
}
//==================================================================
bool CRSF::bytesToPWM(uint8_t *sb_byte, uint16_t *ch_val, uint8_t max_ch)  
{ 
  // remember these are SBus PWM values, which range from 192 thru 1792, not 1000 thru 2000
  *ch_val = ((*sb_byte | *(sb_byte + 1) << 8) & 0x07FF);
  *(ch_val + 1) = ((*(sb_byte + 1) >> 3 | *(sb_byte + 2) << 5) & 0x07FF);
  *(ch_val + 2) = ((*(sb_byte + 2) >> 6 | *(sb_byte + 3) << 2 | *(sb_byte + 4) << 10) & 0x07FF);
  *(ch_val + 3) = ((*(sb_byte + 4) >> 1 | *(sb_byte + 5) << 7) & 0x07FF);
  *(ch_val + 4) = ((*(sb_byte + 5) >> 4 | *(sb_byte + 6) << 4) & 0x07FF);
  *(ch_val + 5) = ((*(sb_byte + 6) >> 7 | *(sb_byte + 7) << 1 | *(sb_byte + 8) << 9) & 0x07FF);
  *(ch_val + 6) = ((*(sb_byte + 8) >> 2 | *(sb_byte + 9) << 6) & 0x07FF);
  *(ch_val + 7) = ((*(sb_byte + 9) >> 5 | *(sb_byte + 10) << 3) & 0x07FF);

  if ((max_ch == 16) || (max_ch == 24))
  {
    *(ch_val + 8) = ((*(sb_byte + 11) | *(sb_byte + 12) << 8) & 0x07FF);
    *(ch_val + 9) = ((*(sb_byte + 12) >> 3 | *(sb_byte + 13) << 5) & 0x07FF);
    *(ch_val + 10) = ((*(sb_byte + 13) >> 6 | *(sb_byte + 14) << 2 | *(sb_byte + 15) << 10) & 0x07FF);
    *(ch_val + 11) = ((*(sb_byte + 15) >> 1 | *(sb_byte + 16) << 7) & 0x07FF);
    *(ch_val + 12) = ((*(sb_byte + 16) >> 4 | *(sb_byte + 17) << 4) & 0x07FF);
    *(ch_val + 13) = ((*(sb_byte + 17) >> 7 | *(sb_byte + 18) << 1 | *(sb_byte + 19) << 9) & 0x07FF);
    *(ch_val + 14) = ((*(sb_byte + 19) >> 2 | *(sb_byte + 20) << 6) & 0x07FF);
    *(ch_val + 15) = ((*(sb_byte + 20) >> 5 | *(sb_byte + 21) << 3) & 0x07FF);
  }

  if (max_ch == 24)
  {
    *(ch_val + 16) = ((*(sb_byte + 22) | *(sb_byte + 23) << 8) & 0x07FF);
    *(ch_val + 17) = ((*(sb_byte + 23) >> 3 | *(sb_byte + 24) << 5) & 0x07FF);
    *(ch_val + 18) = ((*(sb_byte + 24) >> 6 | *(sb_byte + 25) << 2 | *(sb_byte + 26) << 10) & 0x07FF);
    *(ch_val + 19) = ((*(sb_byte + 26) >> 1 | *(sb_byte + 27) << 7) & 0x07FF);
    *(ch_val + 20) = ((*(sb_byte + 27) >> 4 | *(sb_byte + 28) << 4) & 0x07FF);
    *(ch_val + 21) = ((*(sb_byte + 28) >> 7 | *(sb_byte + 29) << 1 | *(sb_byte + 30) << 9) & 0x07FF);
    *(ch_val + 22) = ((*(sb_byte + 30) >> 2 | *(sb_byte + 31) << 6) & 0x07FF);
    *(ch_val + 23) = ((*(sb_byte + 31) >> 5 | *(sb_byte + 32) << 3) & 0x07FF);
  }

  for (int i = max_ch; i < 25; i++)
  { // pad out pwm to 24 channels with 1500
    *(ch_val + i) = 1500;
  }
  // pwm_rssi = *(sb_byte+23);  // if we have a designated rssi channel
  // remap SBUS pwm values in the range of 192 - 1792 (0x00 thu 0xFF) to regular pwm values
  for (int i = 0; i < max_ch; i++)
  {
    if (*(ch_val + i) > 0)
    {
      *(ch_val + i) = map(*(ch_val + i), 192, 1792, 1000, 2000); // regular PWM uS limits
    }
  }
  if (*ch_val > 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}
//===============================================================
void CRSF::pwmToBytes(uint16_t *in_pwm, uint8_t *rc_byt, uint8_t max_ch)
{
  uint16_t ch_pwm[RC_INPUT_MAX_CHANNELS] {};
  // remap PWM values to sbus byte values in the range of 192 - 1792 (0x00 thu 0xFF)
  for (int i = 0; i < max_ch; i++)
  {
    if (*(in_pwm + i) > 0)
    {
    *(ch_pwm + i) = map(*(in_pwm + i), 1000, 2000, 192, 1792); // new SBUS uS limits
    //log.printf("in_pwm:%u  ch_pwm:%u", *(in_pwm + i), *(ch_pwm + i));
    }
  }
  //log.println();
  uint8_t byte_cnt = 0;
  uint8_t ch_cnt = 0;
  uint8_t rc_bit = 0;
  uint8_t ch_bit = 0;

  ch_bit = 0;
  byte_cnt = 0; // no header
  rc_bit = 0;

  uint16_t max_bits = max_ch * 11; // each pwm ch = 11 bits   8 x 11 = 88 or 8 x 22 = 176 or 8 x 33 = 264
  for (int i = 0; i < max_bits; i++)
  {
    if (*(ch_pwm + ch_cnt) & (1 << ch_bit))
    {
      *(rc_byt + byte_cnt) |= (1 << rc_bit);
    }
    rc_bit++;
    ch_bit++;
    if (rc_bit == 8)
    { // byte full, so increment and do next
      rc_bit = 0;
      // log.printf("byt:%2X ", rc_byt[byte_cnt]);
      byte_cnt++;
    }
    if (ch_bit == 11)
    { // pwm ch overflow, so increment ch_no and do next
      // log.printf("ch_pwm:%u ", ch_pwm[ch_cnt]);
      ch_bit = 0;
      ch_cnt++;
    }
  }
  // log.println();
}
//========================================================
bool CRSF::readCrsfFrame(uint8_t &frm_lth)
{
  static uint8_t b = 0;
  static uint8_t idx = 0;
  static uint8_t embed_lth = 0;
  static uint8_t crsf_crc = 0;
  if (crsfSerial.available())
  {
    uint8_t numch = crsfSerial.available();
    while (numch)
    {
        if (idx == 0)
        {
          memset(crsf_buf, 0, CRSF_BUFFER_SIZE); // flush the crsf_buf
 #if defined RC_BUILD
          if (b == CRSF_RC_SYNC_BYTE2)               // prev read byte    
          {
            crsf_buf[0] = b;                         // to front of buf
          }
        }
#else   // TELEM BUILD
        if (b == CRSF_TEL_SYNC_BYTE)               // prev read byte    
          {
            crsf_buf[0] = b;                         // to front of buf
          }
        }
#endif
        static uint8_t prev_b = 0;
        prev_b = b;
        b = crsfSerial.read();
        numch--;
        //printByte(b, ' ');
#if defined RC_BUILD
        if ((b == CRSF_RC_SYNC_BYTE2) && (prev_b == CRSF_RC_SYNC_BYTE1))
#else
        if (b == CRSF_TEL_SYNC_BYTE)         // new frame, so now process prev buffer     
#endif
        {
            frames_read++;     
            frm_lth = idx;    
            idx = 0;
#ifndef RC_BUILD    // TELEMETRY BUILD  
            crsf_crc = crc8_dvb_s2_sbuf_accum(&crsf_buf[2], frm_lth-2); // lth -start 
            if (crsf_buf[frm_lth] != crsf_crc)
            {
              crc_errors++;
              //log.printf("frm_lth:%u  buf_crc:0x%2X calc_crc:0x%2X   mismatch\n", frm_lth, crsf_buf[frm_lth], crsf_crc);
              return false;
            } 
            if (frm_lth != embed_lth)  // embedded frm lth byte + CRC
            {
              frame_errors++;   
              //log.printf("frm_lth:%u  embed_lth:%u mismatch\n", frm_lth, embed_lth);
              return false;
            }

#endif
            good_frames++;
            return true;
        }
        // prevent array overflow
        if (idx < CRSF_BUFFER_SIZE-1) idx++;
        crsf_buf[idx] = b; 
        if (idx == 1) 
        {
          embed_lth = b+1; // 2nd byte, + 1
        }
    }
  }
   
  return false;  // drop thru and loop
}
//===================================================================
#if defined SEND_SBUS
void CRSF::sendSBUS(uint8_t *sb_buf)
{
  /*
   A single SBUS message is 25 bytes long and therefore takes 3ms to be transmitted.
   It can be sent every 15mS, and consists of the following bytes:

   1 Header byte 00001111b (0x0F)
   16 * 11 bit channels -> 22 bytes
   1 status byte for frame-lost (los) and failsafe
   1 Footer byte 00000000b (0x00)
   */

  sbusSerial.write(sb_buf, 25);
}
#endif
//========================================================
uint8_t CRSF::decodeTelemetry(uint8_t *_buf)
{
  uint8_t crsf_frm_lth = _buf[1];   
  uint8_t crsf_id = _buf[2];
  if (crsf_id == 0)
    {
      return 0;
    }
#if defined SHOW_BUFFER
  log.print("CRSF_BUF:");
  printBytes(&*_buf, crsf_frm_lth+2); // plus header and crc bytes
#endif
    switch (crsf_id)
    {
    case GPS_ID:
      gps_lat = bytes2int32(&_buf[3]);  // offset (&*(_buf+3))
      gps_lon = bytes2int32(&_buf[7]);
      gpsF_lat = (float)(gps_lat / 1e7);                     // degrees+decimals
      gpsF_lon = (float)(gps_lon / 1e7);
      gps_groundspeed = bytes2uint16(&_buf[11]);
      gpsF_groundspeed = (float)(gps_groundspeed * 0.1);     // km\hr
      gps_heading = bytes2uint16(&_buf[13]);
      gpsF_heading = (float)(gps_heading * 0.01);            // degrees+decimals 
      gps_altitude = bytes2uint16(&_buf[15]);                // metres, ­1000m offset
      gps_altitude = gps_altitude > 100 ? gps_altitude - 1000: gps_altitude;
      gps_sats = (uint8_t)_buf[17];
      break;
    case CF_VARIO_ID:
#if defined DEMO_CRSF_CF_VARIO 
      log.print("CF_VARIO:");
#endif 
      break;
    case BATTERY_ID:
      bat_voltage = bytes2uint16(&_buf[3]);           // mV * 100
      batF_voltage = (float)bat_voltage * 10;         // volts
      bat_current = bytes2uint16(&_buf[5]);           // mA * 100
      batF_current = bat_current * 10;                // amps
      bat_fuel_drawn = bytes2int32(&_buf[7]);         // uint24_t    mAh drawn
      batF_fuel_drawn = bat_fuel_drawn * 1e3;         // Ah drawn
      bat_remaining = (uint8_t)_buf[10];              // percent
      break;
    case BARO_ALT_ID:
#if defined DEMO_CRSF_BARO     
      log.print("BARO_ALT:");
#endif
       break; 
    case HEARTBEAT_ID:
#if defined DEMO_CRSF_HEARTBEAT 
      log.print("HEARTBEAT:");
#endif
       break;     
    case LINK_ID:  // 0x14 Link statistics
      link_up_rssi_ant_1 = (uint8_t)_buf[3];          // dBm * -1
      link_up_rssi_ant_2 = (uint8_t)_buf[4];          // dBm * -1
      link_up_quality = (uint8_t)_buf[5];             // packet_success_rate (%)
      link_up_snr = (int8_t)_buf[6];                  // db
      link_diversity_active_ant = (uint8_t)_buf[7];   // (enum ant_1 = 0, ant_2)
      link_rf_mode = (uint8_t)_buf[8];                // (enum 4fps = 0, 50fps, 150hz)
      link_up_tx_power = (uint8_t)_buf[9];            // (enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW)
      link_dn_rssi = (uint8_t)_buf[10];               // RSSI(dBm * -1)
      link_dn_quality = (uint8_t)_buf[11];            // packet_success_rate (%)
      link_dn_snr = (int8_t)_buf[12];                 // db
      break;
    case CHANNELS_ID:
#if defined DEMO_CRSF_CHANNELS 
      log.print("CHANNELS:");
#endif
      break;
    case LINK_RX_ID:
#if defined DEMO_CRSF_LINK_RX 
      log.print("LINK_RX:");
#endif
      break;
    case LINK_TX_ID:
#if defined DEMO_CRSF_LINK_TX 
      log.print("LINK_TX:");
#endif
      break;
    case ATTITUDE_ID:
      atti_pitch = bytes2int16(&_buf[3]);                         // rad / 10000  
      atti_roll = bytes2int16(&_buf[5]);                          // rad / 10000
      atti_yaw = bytes2int16(&_buf[7]);                           // rad / 10000
      attiF_pitch = (float)(atti_pitch * RADS2DEGS * 0.0001);     // deg
      attiF_roll = (float)(atti_roll * RADS2DEGS * 0.0001);       // deg    
      atti_yaw = (int16_t)(atti_yaw * RADS2DEGS * 0.0001);        // deg
      atti_yaw = wrap360(atti_yaw);
      attiF_yaw = (float)atti_yaw;
      break;
    case FLIGHT_MODE_ID:
      /* HUH! Flight mode is a string*/
      flight_mode_lth = crsf_frm_lth - 4;
      memcpy(&flightMode, &_buf[4], flight_mode_lth);
      //printBytes(&_buf[4], flight_mode_lth);                  
#if defined DEMO_CRSF_FLIGHT_MODE
      log.print("FLIGHT_MODE id:");
      printByte(crsf_id, ' ');
      printf("lth:%u %s\n",flight_mode_lth, &flightMode);
#endif
      break;
    case PING_DEVICES_ID:
#if defined DEMO_CRSF_GPS_PING_DEVICES 
      log.print("PING_DEVICES:");
#endif
      break;
    case DEVICE_INFO_ID:
#if defined DEMO_CRSF_DEVIDE_INFO 
      log.print("DEVICE_INFO:");
#endif
      break;
    case REQUEST_SETTINGS_ID:
#if defined DEMO_CRSF_REQUEST_SETTINGS 
      log.print("REQUEST_SETTINGS:");
#endif
      break;
    case COMMAND_ID:
#if defined DEMO_CRSF_COMMAND 
      log.print("COMMAND:");
#endif
      break; 
    case RADIO_ID:
#if defined DEMO_CRSF_RADIO 
      log.print("RADIO id:");
#endif
      break; 
    default:
      //log.print("crsf_id:");
      //printByte(crsf_id, ' ');
      //log.print("UNKNOWN  ");
      //printBytes(&*_buf, crsf_frm_lth+2); // plus header and CRC bytes
      unknown_ids++;
      return 0;
    }
    return crsf_id;
}
//========================================================
void CRSF::decodeRC()
{
#if defined SHOW_BUFFER
  log.print("CRSF_BUF:");
  printBytes(&*crsf_buf, 24); // sync (lth) byte + 22 RC bytes
#endif
  bytesToPWM(&*(crsf_buf+1), &*pwm_val, max_ch);
#if defined SEND_SBUS || defined DEMO_SBUS
  memcpy(&*rc_bytes, &*(crsf_buf + 1), 22);
  prepSBUS(&*rc_bytes, &*sb_bytes, false, false);
#endif
#if defined SEND_SBUS
  sendSBUS(&*sb_bytes);
#endif
 
}