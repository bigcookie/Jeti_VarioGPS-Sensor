/*
  -----------------------------------------------------------
            Settings & defaults
  -----------------------------------------------------------
*/

// **** General settings ****************

//#define UNIT_US                             //uncomment to enable US units

// #define JETI_DEV_ID_LOW 0x76
// #define JETI_DEV_ID_HIGH 0x32
#define DEFAULT_PITOT_TUBE_CORRECTION 1.00f
#define DEFAULT_SM_TUBE_CORRECTION 1.25f
#define DEFAULT_TUBE_CORRECTION DEFAULT_SM_TUBE_CORRECTION
// #define CFG_DEFAULT
//#define TEST_HARDWARE
// #define CFG_AIRSPEEDTEST
// #define CFG_FFSWIFT32
// #define CFG_FWSWIFT38
// #define CFG_RS_TOXIC
// #define CFG_RS_TOXIC_GPS
//  #define CFG_GPS_ONLY
#define CFG_RXQ_ONLY
// #define CFG_RADICAL
// #define CFG_CFSB14_60
// #define CFG_FF_MACKA35
// #define CFG_PRESSURE_SENSOR_TEST
// supported devices
#if defined(CFG_FFSWIFT32)
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611_LPS
  #define SUPPORT_GPS
  #define SUPPORT_RX_VOLTAGE
  #define SERVOSIGNAL
  #define SERVOSIGNAL_PIN 3
  #define ANALOG_R_DIVIDER_20_20
  #elif defined(CFG_PRESSURE_SENSOR_TEST)
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611_LPS
#elif defined(CFG_FF_MACKA35)
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611_LPS
  #define SUPPORT_GPS
  #define SERVOSIGNAL
  #define SERVOSIGNAL_PIN_PULLUP
  #define SERVOSIGNAL_PIN 3
#elif defined(CFG_GPS_ONLY)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_GPS
  // #define SUPPORT_RX_VOLTAGE
#elif defined(CFG_RXQ_ONLY)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SERVOSIGNAL
  #define SERVOSIGNAL_PIN_PULLUP
  #define SERVOSIGNAL_PIN 3
  // #define SUPPORT_RX_VOLTAGE
#elif defined(CFG_RS_TOXIC)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_BMx280                        // comment to disable devices
  #define SERVOSIGNAL
  #define SERVOSIGNAL_PIN_PULLUP
  #define SPEEDVARIO
  #define SUPPORT_RX_VOLTAGE
#elif defined(CFG_RS_TOXIC_GPS)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611_LPS                        // comment to disable devices
  #define SERVOSIGNAL
  #define SERVOSIGNAL_PIN_PULLUP
  #define SPEEDVARIO
  #define SUPPORT_GPS
  #define SUPPORT_RX_VOLTAGE
#elif defined(CFG_RADICAL)
  #define V_REF              3300        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611_LPS
#elif defined(CFG_CFSB14_60)
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define JETI_DEV_ID_LOW 0x76           // to support SpeedVario parallel to the MEZON speed controller
  #define JETI_DEV_ID_HIGH 0x33
  #define SUPPORT_MS5611_LPS
  #define SUPPORT_GPS
  #define SPEEDVARIO
  #define SERVOSIGNAL
  #define SUPPORT_MPXV7002_MPXV5004
  #define SUPPORT_AIRSPEED_JETIBOX
  #define SUPPORT_TEC
  #define SUPPORT_RX_VOLTAGE
  #define ANALOG_R_DIVIDER_20_20
  // #define TEST_TELEMETRY_VALUE
#elif defined(CFG_FWSWIFT38)
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611_LPS
  #define SUPPORT_GPS
  #define SPEEDVARIO
  #define SERVOSIGNAL
  #define SUPPORT_MPXV7002_MPXV5004
  #define SUPPORT_AIRSPEED_JETIBOX
  #define SUPPORT_TEC
  // #define SAW_TOOTH
#elif defined(CFG_JR_ASW17)
  #define V_REF               5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_MS5611_LPS
  #define SUPPORT_GPS
  #define SPEEDVARIO
  #define SUPPORT_MPXV7002_MPXV5004
  #define SUPPORT_TEC
#else
  #define V_REF              5000        // set supply voltage from 1800 to 5500mV
  #define SUPPORT_BMx280                        // comment to disable devices
  #define SUPPORT_GPS
  // #define SUPPORT_MAIN_DRIVE
  // #define SUPPORT_RX_VOLTAGE
  // #define SUPPORT_EXT_TEMP
  #define SPEEDVARIO
  // #define ANALOG_R_DIVIDER_20_20
#endif

// support JetiBox Menu
#define SUPPORT_JETIBOX_MENU

// **************************************


// EEprom parameter addresses
enum
{
  P_GPS_MODE =              1,
  P_GPS_3D =                2,
  P_CURRENT_SENSOR =        3,
  P_CURRENT_CALIBRATION =   4,
  P_CAPACITY_MODE =         5,
  P_ENABLE_RX1 =            6,
  P_ENABLE_RX2 =            7,
  P_ENABLE_TEMP =           8,
  P_VARIO_SMOOTHING =      10,
  P_VARIO_DEADZONE =       12,
  P_AIRSPEED_SENSOR =      16,
  P_TEC_MODE =             17,
  P_AIRSPEED_SMOOTHING =   18,
  P_AIRSPEED_TUBE_CORR =   19,
  P_CAPACITY_VALUE =       20,
  P_VOLTAGE_VALUE =        P_CAPACITY_VALUE+sizeof(float)
};

// Sensor IDs
enum
{
  ID_GPSLAT = 1,
  ID_GPSLON,
  ID_GPSSPEED,
  ID_ALTREL,
  ID_ALTABS,
  ID_VARIO,
#if defined(SPEEDVARIO) || defined(SERVOSIGNAL)
#ifdef SUPPORT_TEC
  ID_TEC_VARIO,
#endif
  ID_SV_SIG_LOSS_CNT,
  ID_SV_SIGNAL_DURATION,
  ID_SV_SIGNAL_DURATION_MAX,
#ifdef SIGNAL_FREQ
  ID_SV_SIGNAL_FRQ,
  ID_SV_SIGNAL_FRQ_RETARDED,
#endif
#endif
  ID_DIST,
  ID_TRIP,
  ID_HEADING,
  ID_COURSE,
  ID_SATS,
  ID_HDOP,
  ID_PRESSURE,
  ID_TEMPERATURE,
  ID_HUMIDITY,
  ID_VOLTAGE,
  ID_CURRENT,
  ID_CAPACITY,
  ID_POWER,
  ID_RX1_VOLTAGE,
  ID_RX2_VOLTAGE,
  ID_EXT_TEMP,
#ifdef SUPPORT_MPXV7002_MPXV5004
  ID_AIRSPEED,
#endif
#ifdef SAW_TOOTH
  ID_SAWTOOTH,
#endif
#ifdef TEST_TELEMETRY_VALUE
  ID_TESTVALUE,
#endif
  ID_LAST
};

/*
TYPE_6b   int6_t    Data type 6b (-31 to 31)
TYPE_14b  int14_t   Data type 14b (-8191 to 8191)
TYPE_22b  int22_t   Data type 22b (-2097151 to 2097151)
TYPE_DT   int22_t   Special data type for time and date
TYPE_30b  int30_t   Data type 30b (-536870911 to 536870911)
TYPE_GPS  int30_t   Special data type for GPS coordinates:  lo/hi minute - lo/hi degree.
*/

// Sensor names and unit[EU]
#ifndef UNIT_US
#ifdef SERIAL_TEST
JETISENSOR_CONST sensors[] PROGMEM =
#else
JETISENSOR_CONST sensors[] PROGMEM =
#endif
{
  // id             name          unit          data type           precision
  { ID_GPSLAT,      "Latitude",   " ",          JetiSensor::TYPE_GPS, 0 },
  { ID_GPSLON,      "Longitude",  " ",          JetiSensor::TYPE_GPS, 0 },
  { ID_GPSSPEED,    "GPS Speed",  "km/h",       JetiSensor::TYPE_14b, 0 },
  { ID_ALTREL,      "Rel. Altit", "m",          JetiSensor::TYPE_22b, 1 },
  { ID_ALTABS,      "Altitude",   "m",          JetiSensor::TYPE_22b, 0 },
  { ID_VARIO,       "Vario",      "m/s",        JetiSensor::TYPE_22b, 2 },
#if defined(SPEEDVARIO) || defined(SERVOSIGNAL)
#ifdef SUPPORT_TEC
  { ID_TEC_VARIO,    "TEC Vario",      "m/s",   JetiSensor::TYPE_22b, 2 },
#endif
  { ID_SV_SIG_LOSS_CNT,         "SigLossCnt", "#",  JetiSensor::TYPE_14b, 0 },
  { ID_SV_SIGNAL_DURATION_MAX,  "SigDuraMax","ms", JetiSensor::TYPE_22b, 0 },
  { ID_SV_SIGNAL_DURATION,  "SigDura","ms", JetiSensor::TYPE_22b, 0 },
#ifdef SIGNAL_FREQ
  { ID_SV_SIGNAL_FRQ,    "Signal Freq",      "Hz",  JetiSensor::TYPE_14b, 0 },
  { ID_SV_SIGNAL_FRQ_RETARDED,    "SigFreqRet",      "Hz",  JetiSensor::TYPE_14b, 0 },
#endif
#endif
  { ID_DIST,        "Distance",   "m",          JetiSensor::TYPE_22b, 0 },
  { ID_TRIP,        "Trip",       "km",         JetiSensor::TYPE_22b, 2 },
  { ID_HEADING,     "Heading",    "\xB0",       JetiSensor::TYPE_14b, 0 },
  { ID_COURSE,      "Course",     "\xB0",       JetiSensor::TYPE_14b, 0 },
  { ID_SATS,        "Satellites", " ",          JetiSensor::TYPE_6b,  0 },
  { ID_HDOP,        "HDOP",       " ",          JetiSensor::TYPE_14b, 2 },
  { ID_PRESSURE,    "Pressure",   "hPa",        JetiSensor::TYPE_22b, 2 },
  { ID_TEMPERATURE, "Temperature","\xB0\x43",   JetiSensor::TYPE_14b, 1 },
  { ID_HUMIDITY,    "Humidity",   "%rH",        JetiSensor::TYPE_14b, 1 },
  { ID_VOLTAGE,     "Voltage",    "V",          JetiSensor::TYPE_14b, 1 },
  { ID_CURRENT,     "Current",    "A",          JetiSensor::TYPE_14b, 1 },
  { ID_CAPACITY,    "Capacity",   "mAh",        JetiSensor::TYPE_22b, 0 },
  { ID_POWER,       "Power",      "W",          JetiSensor::TYPE_22b, 0 },
  { ID_RX1_VOLTAGE, "Rx1 Voltage","V",          JetiSensor::TYPE_14b, 2 },
  { ID_RX2_VOLTAGE, "Rx2 Voltage","V",          JetiSensor::TYPE_14b, 2 },
  { ID_EXT_TEMP,    "Ext. Temp",  "\xB0\x43",   JetiSensor::TYPE_14b, 1 },
#ifdef SUPPORT_MPXV7002_MPXV5004
  { ID_AIRSPEED,    "Air Speed",   "km/h",      JetiSensor::TYPE_14b, 0 },
#endif
#ifdef SAW_TOOTH
  { ID_SAWTOOTH,    "Sawtooth",   "#",          JetiSensor::TYPE_14b, 0 },
#endif
#ifdef TEST_TELEMETRY_VALUE
  { ID_TESTVALUE,    "TestVal",   "#",          JetiSensor::TYPE_14b, 0 },
#endif
  { 0 }
};
#endif

// TEC mode
enum {
  TEC_disabled,
  TEC_airSpeed,
  TEC_GPS
};

#define PRESSURE_SEALEVEL         101325   // Pa

// Sensor names and unit[US]
#ifdef UNIT_US
JETISENSOR_CONST sensors[] PROGMEM =
{
  // id             name          unit          data type           precision
  { ID_GPSLAT,      "Latitude",   " ",          JetiSensor::TYPE_GPS, 0 },
  { ID_GPSLON,      "Longitude",  " ",          JetiSensor::TYPE_GPS, 0 },
  { ID_GPSSPEED,    "Speed",      "mph",        JetiSensor::TYPE_14b, 0 },
  { ID_ALTREL,      "Rel. Altit", "ft",         JetiSensor::TYPE_22b, 1 },
  { ID_ALTABS,      "Altitude",   "ft",         JetiSensor::TYPE_22b, 0 },
  { ID_VARIO,       "Vario",      "ft/s",       JetiSensor::TYPE_22b, 2 },
#ifdef SPEEDVARIO
  { ID_SPEEDVARIO,  "SpeedVario",      "??",   JetiSensor::TYPE_22b, 2 },
#endif
  { ID_DIST,        "Distance",   "ft.",        JetiSensor::TYPE_22b, 0 },
  { ID_TRIP,        "Trip",       "mi",         JetiSensor::TYPE_22b, 2 },
  { ID_HEADING,     "Heading",    "\xB0",       JetiSensor::TYPE_14b, 0 },
  { ID_COURSE,      "Course",     "\xB0",       JetiSensor::TYPE_14b, 0 },
  { ID_SATS,        "Satellites", " ",          JetiSensor::TYPE_6b,  0 },
  { ID_HDOP,        "HDOP",       " ",          JetiSensor::TYPE_14b, 2 },
  { ID_PRESSURE,    "Pressure",   "inHG",       JetiSensor::TYPE_22b, 2 },
  { ID_TEMPERATURE, "Temperature","\xB0\x46",   JetiSensor::TYPE_14b, 1 },
  { ID_HUMIDITY,    "Humidity",   "%rH",        JetiSensor::TYPE_14b, 1 },
  { ID_VOLTAGE,     "Voltage",    "V",          JetiSensor::TYPE_14b, 1 },
  { ID_CURRENT,     "Current",    "A",          JetiSensor::TYPE_14b, 1 },
  { ID_CAPACITY,    "Capacity",   "mAh",        JetiSensor::TYPE_22b, 0 },
  { ID_POWER,       "Power",      "W",          JetiSensor::TYPE_22b, 0 },
  { ID_RX1_VOLTAGE, "Rx1 Voltage","V",          JetiSensor::TYPE_14b, 2 },
  { ID_RX2_VOLTAGE, "Rx2 Voltage","V",          JetiSensor::TYPE_14b, 2 },
  { ID_EXT_TEMP,    "Ext. Temp",  "\xB0\x46",   JetiSensor::TYPE_14b, 1 },
  { 0 }
};
#endif



// **** Vario settings ****

// Pressure Sensors
enum {
  unknown,
  BMP280,
  BME280,
  MS5611_,
  LPS_
};

// Vario lowpass filter and
// dead zone filter in centimeter (Even if you use US-units!)

// BMP280/BME280
#define BMx280_SMOOTHING 0.85
#define BMx280_DEADZONE 5

// MS5611
#define MS5611_SMOOTHING 0.80
#define MS5611_DEADZONE 0

// LPS (LPS311)
#define LPS_SMOOTHING 0.80
#define LPS_DEADZONE 0

// **** Air speed settings ****

#define AIRSPEED_PIN              A7
#define DEFAULT_AIRSPEED_SMOOTHING        0.15

#define UNIVERSAL_GAS_CONSTANT    8.3144621f
#define DRY_AIR_MOLAR_MASS        0.0289644f

// Air speed sensors
enum {
  airSpeed_disabled,
  MPXV7002_MPXV5004
};
// **** GPS settings ****

#define GPSBaud 9600

// GPS mode
enum {
  GPS_disabled,
  GPS_basic,
  GPS_extended
};

#ifdef SPEEDVARIO
#define SPEEDVARIO_SIG_FREQ 50
#endif

// **** Voltage measurement settings ****

// analog input pin
#define VOLTAGE_PIN         A1
#define RX1_VOLTAGE_PIN     A2
#define RX2_VOLTAGE_PIN     A3


// suported voltage sensors
enum {
  AttoPilot_45V,
  AttoPilot_90V,
  AttoPilot_180V,
  APM25_V,
  ACS712_voltage,
  ACS758_voltage,
  Rx1_voltage,
  Rx2_voltage
};

#ifdef ANALOG_R_DIVIDER_20_20
// max. voltage @5.0V vref             13.6V           51.8V           51.8V           33.4V           36.3V           62.7V           10.0V           10.0V
const uint16_t voltageInputR1[] = {   14700,          14700,          14700,          13700,          10000,          18000,          20000,          20000,  };   //Resistor R1 in Ohms
const uint16_t voltageInputR2[] = {    4700,           1000,           1000,           1500,           1000,           1000,          20000,          20000,  };   //Resistor R2 in Ohms
#else
//                                  AttoPilot_45    AttoPilot_90    AttoPilot_180     APM25           ACS712          ACS758        Rx1 Voltage     Rx2 Voltage
// max. voltage @3.3V vref             13.6V           51.8V           51.8V           33.4V           36.3V           62.7V            9.9V            9.9V
const uint16_t voltageInputR1[] = {   14700,          14700,          14700,          13700,          10000,          18000,          20000,          20000,  };   //Resistor R1 in Ohms
const uint16_t voltageInputR2[] = {    4700,           1000,           1000,           1500,           1000,           1000,          10000,          10000,  };   //Resistor R2 in Ohms
#endif

/*
                  voltage input
                     |
                     |
                    | |
                    | |  R1
                    | |
                     |
  analog Pin  <------+
                     |
                    | |
                    | |  R2
                    | |
                     |
                     |
                    GND
*/



// **** Current measurement settings ****

// analog input pin
#define CURRENT_PIN     A0

// capacity reset mode
enum {
  startup,
  automatic,
  manual
};

// save capacity in automatic mode
#define CAPACITY_SAVE_INTERVAL        10000         // ms
#define MAX_CUR_TO_SAVE_CAPACITY      2             // A

// voltage difference to reset
#define VOLTAGE_DIFFERENCE            2             // %

// suported current sensors
enum {
  mainDrive_disabled,
  AttoPilot_45A,
  AttoPilot_90A,
  AttoPilot_180A,
  APM25_A,
  #if V_REF >= 4500
  ACS712_05,
  ACS712_20,
  ACS712_30,
  #endif
  ACS758_50B,
  ACS758_100B,
  ACS758_150B,
  ACS758_200B,
  ACS758_50U,
  ACS758_100U,
  ACS758_150U,
  ACS758_200U
};

// Offset for AttoPilot and APM sensor
const uint16_t Atto_APM_offset = 0;

// Offset for ACS sensor
const uint16_t ACS_B_offset = V_REF/2; //bi-directional offset in mV ( V_REF / 2)
const uint16_t ACS_U_offset = V_REF/8.33;  //uni-directional offset in mV ( V_REF / 8.33)

                         //   mV/Amp @5V            sensor type
const uint8_t mVperAmp[] =  {
                              73,               // AttoPilot 45A
                              37,               // AttoPilot 90A
                              18,               // AttoPilot 180A
                              73,               // APM 2.5 90A
                              #if V_REF >= 4500
                              185,              // ACS712-05
                              100,              // ACS712-20
                               66,              // ACS712-30
                              #endif
                               40,              // ACS758-50B
                               20,              // ACS758-100B
                               13,              // ACS758-150B
                               10,              // ACS758-200B
                               60,              // ACS758-50U
                               40,              // ACS758-100U
                               27,              // ACS758-150U
                               20               // ACS758-200U
                             };



// **** Temperature measurement settings ****

// analog input pin
#define TEMPERATURE_PIN     A6

// Thermistor nominal resistance at 25ºC
#define THERMISTORNOMINAL   10000

// temp. for nominal resistance (almost always 25ºC)
#define TEMPERATURENOMINAL  25

// thermistor's beta coefficient
#define BCOEFFICIENT        3950

// Self-heat compensation (K)
#define SELF_HEAT           1.2

// Value of the series resistor
#define SERIESRESISTOR      10000

/*
                    vcc
                     |
                     |
                    | |
                    | |  Resistor
                    | |
                     |
  analog Pin  <------+
                     |
                    | |
                    | |  NTC
                    | |
                     |
                     |
                    GND
*/




// **** Default settings ****

#define DEFAULT_GPS_MODE          GPS_disabled          //GPS_disabled, GPS_basic, GPS_extended
#define DEFAULT_GPS_3D_DISTANCE   true

#define DEFAULT_CURRENT_SENSOR    mainDrive_disabled
#define DEFAULT_CAPACITY_MODE     automatic             //startup, automatic, manual

#define DEFAULT_ENABLE_Rx1        false
#define DEFAULT_ENABLE_Rx2        false

#define DEFAULT_ENABLE_EXT_TEMP   false

#define DEFAULT_AIRSPEED_SENSOR   airSpeed_disabled
#define DEFAULT_TEC_MODE          TEC_disabled

#ifndef SERVOSIGNAL_PIN
#define SERVOSIGNAL_PIN 2
#endif

#ifndef JETI_DEV_ID_LOW
#define JETI_DEV_ID_LOW 0x76
#endif
#ifndef JETI_DEV_ID_HIGH
#define JETI_DEV_ID_HIGH 0x32
#endif
