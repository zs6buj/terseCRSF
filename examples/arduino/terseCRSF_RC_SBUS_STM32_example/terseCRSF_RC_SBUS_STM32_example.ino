//version 0.0.8
#include <terseCRSF.h>  
// Select RC or telemetry, telem source-type and any debug macros in terseCRSF.h

/* STM32
  Serial       // USART1 (PA9/PA10)
  Serial1      // USART2 (PA2/PA3)
  Serial2      // USART3 (PB10/PB11)

  HardwareSerial(0) -> USART1
  HardwareSerial(1) -> USART2
  HardwareSerial(2) -> USART3
*/

#if defined RC_BUILD
  #define crsf_rxPin          PA3     // USART2 RX
  #define crsf_txPin          PA2     // USART2 TX
  #define crsf_invert         true
    #if defined SUPPORT_SBUS_OUT
    #define sbus_rxPin        PB11    // USART3 RX
    #define sbus_txPin        PB10    // USART3 TX
      #define sbus_invert     false
      #define sbus_baud       100000
    #endif
#else
  #define crsf_rxPin           P3      // USART2 RX
  #define crsf_txPin           PA9     // USART2 TX
  #define crsf_invert          true
#endif

#define crsf_baud              420000

#define log   Serial

#define crsf_uart            USART2                 //Serial rx2=PA3  tx2=PA2  
#if (TELEMETRY_SOURCE  == 1)                // Telemetry from BetaFlight/CF
  #define crsf_baud          420000
#elif (TELEMETRY_SOURCE  == 2)              // EdgeTX/OpenTx
  #define crsf_baud          115200         // Telemetry from RadioMaster TX16S AUX2
#endif

HardwareSerial crsfSerial(USART2);

#if defined(SUPPORT_SBUS_OUT)
  HardwareSerial sbusSerial(2);  // Use '2' for USART3
#endif

CRSF crsf;            // instantiate CRSF object

void printLoop1(bool newline)
{
  static uint32_t prev_lp1_millis = 0;
  uint32_t now_millis = millis();
  uint32_t period = now_millis - prev_lp1_millis;
  log.printf("Loop1 %3dmS ", period);
  if (newline)
    log.println();
  prev_lp1_millis = now_millis;
}

void setupCRSF() 
{
  crsfSerial.begin(crsf_baud);
  //if (crsf_invert) 
  //{
  //  USART2->CR2 |= USART_CR2_RXINV;  // Invert RX for CRSF
  //  USART2->CR2 |= USART_CR2_TXINV; //  Invert TX for CRSF (PIN not used in this application)
  //}
  log.printf("\nCRFS uart:%u  baud:%u  rxPin:%u  txPin:%u  invert:%u\n", crsf_baud, crsf_rxPin, crsf_txPin, crsf_invert);
  crsf.initialise(crsfSerial);
}

void setupSBUS() 
{
#if defined SUPPORT_SBUS_OUT
  sbusSerial.begin(sbus_baud, SERIAL_8E2); // 100000 baud, 8E2
  //if (sbus_invert) 
  //{
    //USART3->CR2 |= USART_CR2_RXINV; // Invert RX for sbus (noyt used in this application)
    //USART3->CR2 |= USART_CR2_TXINV; // Invert TX for SBUS (SBUS not normally inverted)
  //}
  log.printf("SBUS baud:%u  rxPin:%u  txPin:%u  invert:%u\n", sbus_baud, sbus_rxPin, sbus_txPin, sbus_invert);
  crsf.sbus_initialise(sbusSerial);
#endif
}

void setup() {
  log.begin(115200);
  delay(2000);
  setupCRSF(); 
  #if defined SUPPORT_SBUS_OUT
    delay(100);
    setupSBUS();
  #endif
}

void loop() 
{
  if (crsf.readCrsfFrame(crsf.frame_lth))  // exposes frame_lth
  {
  #if defined SHOW_LOOP_PERIOD
    printLoop1(true);
  #endif

  #if defined RC_BUILD      
    crsf.decodeRC(&*(crsf.crsf_buf + 3));   // sync byte(0xEE) + 2 + 22 RC bytes + CRC
    /* here just to illustate where to find the values
    #if defined DEMO_PWM_VALUES
      crsf.printPWM(&*crsf.pwm_val, crsf.max_ch);
    #endif
    #if defined DEMO_SBUS
      log.print("SBUS:");
      crsf.printBytes(&*crsf.sb_bytes, 25);
    #endif 
    */
  #else   // TELEMETRY BUILD
      uint16_t len = crsf.frame_lth;
      uint8_t crsf_id = crsf.decodeTelemetry(&*crsf.crsf_buf, len);

      if (crsf_id == GPS_ID)   
      {
    #if defined DEMO_CRSF_GPS        
          log.print("GPS id:");
          crsf.printByte(crsf_id, ' ');
          log.printf("lat:%2.7f  lon:%2.7f", crsf.gpsF_lat, crsf.gpsF_lon);
          log.printf("  ground_spd:%.1fkm/hr", crsf.gpsF_groundspeed);
          log.printf("  hdg:%.2fdeg", crsf.gpsF_heading);
          log.printf("  alt:%dm", crsf.gps_altitude);
          log.printf("  sats:%d\n", crsf.gps_sats);
    #endif          
      }      
    
      if (crsf_id == BATTERY_ID) 
      {     
    #if defined DEMO_CRSF_BATTERY            
        log.print("BATTERY id:");
        crsf.printByte(crsf_id, ' ');
        log.printf("volts:%2.1f", crsf.batF_voltage);
        log.printf("  amps:%3.1f", crsf.batF_current);
        log.printf("  Ah_drawn:%3.1f", crsf.batF_fuel_drawn);
        log.printf("  remaining:%3u%%\n", crsf.bat_remaining);
    #endif       
      }      

      if (crsf_id == ATTITUDE_ID)
      {
    #if defined DEMO_CRSF_ATTITUDE 
        log.print("ATTITUDE id:");
        crsf.printByte(crsf_id, ' '); 
        log.printf("pitch:%3.1fdeg", crsf.attiF_pitch);
        log.printf("  roll:%3.1fdeg", crsf.attiF_roll);
        log.printf("  yaw:%3.1fdeg\n", crsf.attiF_yaw);  
    #endif          
      }    

      if (crsf_id == FLIGHT_MODE_ID)
      {
    #if defined DEMO_CRSF_FLIGHT_MODE 
        log.print("FLIGHT_MODE id:");
        crsf.printByte(crsf_id, ' ');
        log.printf("lth:%u %s\n", crsf.flight_mode_lth, crsf.flightMode.c_str());
    #endif
      }
  #endif // end of Telemetry Build
  }
}
