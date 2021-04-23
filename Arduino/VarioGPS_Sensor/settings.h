/*
  -----------------------------------------------------------
            Settings 
  -----------------------------------------------------------
  General settings for the VarioGPS sensor.
*/

//#define UNIT_US                             // uncomment to enable US units

#define V_REF                     5000        // set supply voltage from 1800 to 5500mV
#define JETI_SEND_CYCLE 75


// #define CFG_DEFAULT
// #define TEST_HARDWARE
// #define CFG_AIRSPEEDTEST
// #define CFG_FFSWIFT32
// #define CFG_FWSWIFT38
// #define CFG_RADICAL
// #define CFG_RS_ID17_XSWIFT_SPITFIRE
// #define CFG_RS_FW_LIBELLE
// #define CFG_RS_TOXIC_GPS
// #define CFG_MEM_TEST
// #define CFG_3_3V_MS5611_ONLY
// #define CFG_GPS_ONLY
// #define CFG_RXQ_ONLY
// #define CFG_RS_TEST
// #define CFG_TEST_SENSOR
#define CFG_CFSB14_60
// #define CFG_RS_XSWIFT32
// #define CFG_FF_MACKA35
// #define CFG_PRESSURE_SENSOR_TEST
// #define CFG_KLIPPENECK
//
//#define SUPPORT_EX_BUS                        // enable bidirectional EX Bus with 125kbaud, otherwise the old EX protocol with 9.6kbaud is enabled
//                                              // not compatible with SUPPORT_GPS !!!
// supported devices
#if defined(CFG_FFSWIFT32)
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SUPPORT_GPS
  #define SUPPORT_RX_VOLTAGE
  #define SUPPORT_RXQ
  #define RXQ_SIGNAL_PIN 3
  #define ANALOG_R_DIVIDER_20_20
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_PRESSURE_SENSOR_TEST)
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
#elif defined(CFG_FF_MACKA35)
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SUPPORT_GPS
  #define SUPPORT_RXQ
  #define RXQ_SIGNAL_PIN_PULLUP
  #define RXQ_SIGNAL_PIN 3
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_GPS_ONLY)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_GPS
  // #define SUPPORT_RX_VOLTAGE
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_3_3V_MS5611_ONLY)
  #define V_REF               3300        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_RXQ_ONLY)
  #define V_REF               3300        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_RXQ
  // #define RXQ_SIGNAL_PIN_PULLUP
  // #define RXQ_SIGNAL_PIN 3
  // #define SUPPORT_RX_VOLTAGE
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_RS_XSWIFT32)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_BMx280                        // comment to disable devices
  #define SUPPORT_RXQ
  #define SUPPORT_GPS
  #define SUPPORT_RX_VOLTAGE
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_TEST_SENSOR)
  #define V_REF               3300        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_RXQ                    // 40kOhm Resistor to SS_PIN
  #define RXQ_SIGNAL_PIN 3
  #define RXQ_SIGNAL_PIN_PULLUP // for direct connection without resistor
  #define SUPPORT_MS5611
  #define SUPPORT_JETIBOX_MENU
  // #define SUPPORT_GPS
  // #define SPEEDVARIO
  // #define SUPPORT_GY86
  // #define SUPPORT_RX_VOLTAGE
  // #define JETI_EX_SERIAL_OUT
#elif defined(CFG_RS_TEST)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SERVO_SIGNAL_AVG
  // #define SUPPORT_GY86
  #define SUPPORT_RXQ                    // 40kOhm Resistor to SS_PIN
  #define SPEEDVARIO
  #define SUPPORT_GPS
  #define SUPPORT_RX_VOLTAGE
  #define RXQ_SIGNAL_PIN 3
  #define SUPPORT_JETIBOX_MENU
  #define JETI_EX_SERIAL_OUT
  #define DATA_ANALYSIS
#elif defined(CFG_RS_ID17_XSWIFT_SPITFIRE)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SUPPORT_RXQ                    // 40kOhm Resistor to SS_PIN
  #define RXQ_SIGNAL_PIN 6
  #define SPEEDVARIO
  #define SUPPORT_GPS
  #define SUPPORT_RX_VOLTAGE
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_RS_FW_LIBELLE)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SUPPORT_RXQ                    // 40kOhm Resistor to SS_PIN
  #define SPEEDVARIO
  #define SUPPORT_GPS
  #define SUPPORT_RX_VOLTAGE
  #define RXQ_SIGNAL_PIN 3
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_MEM_TEST)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SUPPORT_RXQ
  #define RXQ_SIGNAL_PIN_PULLUP
  #define SUPPORT_GPS
  #define SUPPORT_GPS_EXTENDED
  #define SUPPORT_JETIBOX_MENU
  #define SUPPORT_SYS_INTERNALS
#elif defined(CFG_RS_TOXIC_GPS)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SUPPORT_RXQ
  #define RXQ_SIGNAL_PIN_PULLUP
  #define SUPPORT_GPS
  #define SUPPORT_GPS_EXTENDED
  #define SUPPORT_JETIBOX_MENU
  // #define SUPPORT_SYS_INTERNALS
#elif defined(CFG_RADICAL)
  #define V_REF              3300        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_RXQ
  #define SUPPORT_MS5611
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_CFSB14_60)
  #define DEBUG_SENSOR // provide debug telemetry values
  #define V_REF              4990        // set supply voltage from 1800 to 5500mV
  #define JETI_DEV_ID_LOW 0x76           // to support SpeedVario parallel to the MEZON speed controller
  #define JETI_DEV_ID_HIGH 0x33
  #define SUPPORT_MS5611
  #define SUPPORT_GPS
  #define SUPPORT_RXQ
  #define RXQ_SIGNAL_PIN_PULLUP
  #define RXQ_SIGNAL_PIN 2
  #define SUPPORT_MPXV7002_MPXV5004
  #define SUPPORT_ADA_ADS1115
  #define SUPPORT_TEC
  #define SUPPORT_RX_VOLTAGE
  #define ANALOG_R_DIVIDER_20_20
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_FWSWIFT38)
  #define JETI_SEND_CYCLE 75
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SUPPORT_GPS
  #define SUPPORT_GPS_EXTENDED
  #define SUPPORT_RXQ
  #define SUPPORT_MPXV7002_MPXV5004
  #define SUPPORT_TEC
  #define SUPPORT_JETIBOX_MENU
  // #define SUPPORT_SYS_INTERNALS
#elif defined(CFG_JR_ASW17)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611
  #define SUPPORT_GPS
  #define SPEEDVARIO
  #define SUPPORT_MPXV7002_MPXV5004
  #define SUPPORT_TEC
  #define SUPPORT_JETIBOX_MENU
#elif defined(CFG_KLIPPENECK)
  #define V_REF               3300
  #define SUPPORT_MS5611
  #define SUPPORT_GPS
  #define SUPPORT_JETIBOX_MENU
  #undef JETI_SEND_CYCLE
  #define JETI_SEND_CYCLE 150
#else
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_BMx280                        // comment to disable devices
  #define SUPPORT_GPS
  // #define SUPPORT_MAIN_DRIVE
  // #define SUPPORT_RX_VOLTAGE
  // #define SUPPORT_EXT_TEMP
  #define SPEEDVARIO
  // #define ANALOG_R_DIVIDER_20_20
  #define SUPPORT_JETIBOX_MENU
#endif
