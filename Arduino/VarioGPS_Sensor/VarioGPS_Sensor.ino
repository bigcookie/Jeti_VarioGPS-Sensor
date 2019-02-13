/*
  ------------------------------------------------------------------
                    Jeti VarioGPS Sensor
  ------------------------------------------------------------------
            *** Universeller Jeti Telemetrie Sensor ***
  Vario, GPS, Strom/Spannung, Empfängerspannungen, Temperaturmessung

*/
#define VARIOGPS_VERSION "Vers: V3.2.3.15d"
/*

  ******************************************************************
  Versionen:
  V3.2.3.15 01.11.18  enhanced handling of the MS5611 pressure sensor (beta)
  V3.2.3.14 04.09.18  Cleanup von Telemetrie und Jetibox Menü
  V3.2.3.13 30.08.18  AirSpeed smoothing und tube correction modifyable by JetiBox
  V3.2.3.12 25.08.18  AirSpeed / TEK Support von Nightflyer added
                      für MPXV7002/MPXV5004 Sensor an A7
  V3.2.3.11 11.04.18  SigFreqRet ohne 0-Werte beim Einschalten
  V3.2.3.10 04.04.18  RC-Signal Auswertung SigDura, SigDuraMax
  V3.2.3.9 28.03.18 BugFix: RC-Signal an D2 Behandlung auch ohne Drucksensor
  V3.2.3.8 26.03.18 Bugfix bei float<->int casting Smoothing Factor (merge von master)
  V3.2.3.7 21.03.18 Retardierte SignalFrequenz "SigFreqRet" BugFix Buffer
                    HW Signalmessung an D2 mit 40k Widerstand
  V3.2.3.6 20.03.18 Retardierte SignalFrequenz "SigFreqRet" hinzugefügt
                    RX1/2 Widerstandsteiler 20k/20k für Messung bis 10V bei 5V Arduino
  V3.2.3.5 10.02.18 Konfiguration SpeedVario mit JetiBox
  V3.2.3.4 09.02.18 BugFix TEK V und dV Detektion, Debugging mittels JetiExTest
  V3.2.3.3 03.02.18 "Qualitätswert" für RC-Signal-Frequenz jetzt durch IRQ unabhänige Zählung der Impulse
  V3.2.3.2 02.02.18 Problem mit Interferenz IRQ Behandlung RC-Signal und Behandlung anderer serieller IF behoben.
                    Frequenz des RC-Signals wird jetzt automatisch erkannt.
  V3.2.3.1 01.02.18 Telemetriewert Signal Frequenz des Umschalter Servo-Signals hinzugefügt, um einen "Qualitätswert" im Log zu haben
                    Hierfür sollte im RX die Signalfrequenz auf AUTO stehen, um Frequenzänderungen durch Signalqualität gemessen werden kann.
                    Einführung von Telemetriewert
  V3.2.2  01.02.18  Refactoring und Fehlerorrektur des Vario LowPass Filters Codes
                    siehe: https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
                    jetzt nur noch 1 Dämpfungs-Faktor (JetiBox "Vario Smoth Fact" ~ 0.12)
  V3.2.1  01.02.18  Auswertung des Umschalter Servo-Signals auf Signal-Verlust mit Telemetrie-Ausgabe
  V3.2.0  29.01.18  Vario mit Umschalter auf D2: >+50% : normales Vario
                                                     sonst : Speed-Anzeige als Landehilfe mit Trimmbereich +-49% für Landespeed
                                                     <-50% : Energie kompensiertes Vario (GPS Speed, Versuch ob es was bringt)
                    in der JetiBox kann die Landespeed als Wert "Normal Speed:" in m/s gespeichert werden.
  V2.2    15.02.18  Vario Tiefpass mit nur einem Smoothing Factor (by RS)
                    Jeder Sensor kann mit #define deaktiviert werden
  V2.1.1  13.01.18  kleine Fehlerbehebung mit libraries
  V2.1    23.12.17  Analog Messeingänge stark überarbeitet, NTC-Temperaturmessung hinzugefügt,
                    startup-/auto-/maual-reset für Kapazitätsanzeige, SRAM-Speicheroptimierung
  V2.0.2  03.12.17  Fehler in GPS Trip behoben
  V2.0.1  21.11.17  Fehler bei Spannungsmessung behoben
  V2.0    20.11.17  GPS Trip[km] und verbrauchte Kapazität[mAh] eingebaut, Stromsensoren ACS712 eingebaut
  V1.9    17.11.17  ACSxxx Stromsensoren eingebaut
  V1.8    17.11.17  Luftdrucksensor MS5611/LPS werden unterstützt
  V1.7.2  15.11.17  Fehlerbehebung mit BME280
  V1.7.1  15.11.17  Speicheroptimierung, kleinere Fehler behoben
  V1.7    12.11.17  Empfängerspannungen können gemessen werden
  V1.6    05.11.17  Luftdrucksensoren BMP085/BMP180/BMP280/BME280 eingebaut, und zu VarioGPS umbenannt
  V1.5    05.11.17  Code von RC-Thoughts(Jeti_GPS-Sensor) übernommen




  ******************************************************************
  Unterstützte Hardware:

  - Arduino Pro Mini 3.3V-8Mhz/5V-16Mhz
  - GPS-Modul mit NMEA Protokoll und UART@9600baud
  - Luftdrucksensoren: BMP280, BME280, MS5611, LPS
  - Stromsensoren @3.3V/5V Betriebsspannung:        AttoPilot Module @3.3V: 45A/13.6V - 90A/50V - 180A/50V (@5V: 45A/20.6V - 90A/60V - 180A/60V)
                                                    APM2.5 PowerModul @5V: 90A/50V (@3.3V: 58A/33.4V)
                                                    ACS758_50B, ACS758_100B, ACS758_150B, ACS758_200B, ACS758_50U, ACS758_100U, ACS758_150U, ACS758_200U
  - zusätzliche Stromsensoren @5V Betriebsspannung: ACS712_05, ACS712_20, ACS712_30



  ******************************************************************
  Anzeige:

  Nur mit Luftdrucksensor werden die Werte angezeigt:
  - Rel. und Abs. Höhe
  - Vario
  - Luftdruck
  - Temperatur
  - Luftfeuchte (nur mit BME280)

  Im GPS Basic Mode werden die Werte angezeigt:
  - Position
  - Geschwindigkeit
  - Rel. und Abs. Höhe
  - Vario

  Im GPS Extended Mode werden zusätzlich die Werte angezeigt:
  - Distanz vom Modell zum Startpunkt (2D oder 3D)
  - zurückgelegte Strecke (Trip)
  - Flugrichtung (Heading)
  - Kurs vom Modell zum Startpunkt
  - Anzahl Satelliten
  - HDOP (Horizontaler Faktor der Positionsgenauigkeit)
  - Luftdruck
  - Temperatur
  - Luftfeuchtigkeit

  An den Analogeingängen sind folgende Messungen möglich:
  - Strom- und Spannung für Hauptantrieb mit verbrauchter Kapazität[mAh] und Leistung[W]
  - 2x Empfängerspannung
  - Temperatur mit NTC-Wiederstand von -55 bis +155°C

  Folgende Einstellungen können per Jetibox vorgenommen werden:
  - GPS: deaktiviert, Basic oder Extended
  - GPS Distanz: 2D oder 3D
  - Vario Filterparameter X, Y und Deadzone
  - Stromsensor für Hauptantrieb
  - Einstellung Reset der Kapazität:
        STARTUP(Wert ist nach jedem Einschalten auf 0)
        AUTO(Wert wird gespeichert und erst zurückgesetzt wenn ein geladener Akku angeschlossen wird)
        MANUAL(Wert muss manuell per Jetibox zurückgesetzt werden mit RESET OFFSET)
  - Rx1, Rx2 Empfängerspannungsmessung aktivieren
  - Temperaturmessung aktivieren

*/

// uncomment this, to get debug Serial.prints instead of JetiExBus protocol for debugging
// #define JETI_EX_SERIAL_OUT

#ifdef JETI_EX_SERIAL_OUT
#include "JetiExTest.h"
#include <Wire.h>
#else
#include <JetiExSerial.h>
#include <JetiExProtocol.h>
#include <Wire.h>
#endif

#include <EEPROM.h>
#include "defaults.h"

JetiExProtocol jetiEx;

#ifdef SUPPORT_GPS
  #include <TinyGPS++.h>
  #include <AltSoftSerial.h>
  TinyGPSPlus gps;
  AltSoftSerial gpsSerial;
#endif

#ifdef SUPPORT_BMx280
  #include "BMx_Sensor.h"
  BMx_Sensor boschPressureSensor;
#endif

#ifdef SUPPORT_MS5611_LPS
#include <MS5611.h>
#include <LPS.h>
MS5611 ms5611;
LPS lps;
#endif

#if V_REF < 1800 || V_REF > 5500
#error unsupported supply voltage
#endif

#define MEASURING_INTERVAL        180         //ms
#define EEPROM_ADRESS_CAPACITY    20

struct {
  uint8_t mode = DEFAULT_GPS_MODE;
  bool distance3D = DEFAULT_GPS_3D_DISTANCE;
} gpsSettings;

struct {
  uint8_t type = unknown;
  float smoothingValue;
  long deadzone;
} pressureSensor;


#ifdef SAW_TOOTH
  uint8_t sawtoothVal = 0;
#endif // SAW_TOOTH
unsigned long lastTime = 0;
unsigned long lastTimeCapacity = 0;
uint8_t currentSensor = DEFAULT_CURRENT_SENSOR;
uint8_t capacityMode = DEFAULT_CAPACITY_MODE;
bool enableRx1 = DEFAULT_ENABLE_Rx1;
bool enableRx2 = DEFAULT_ENABLE_Rx2;
bool enableExtTemp = DEFAULT_ENABLE_EXT_TEMP;
int32_t gpsSpeed = 0;

struct {
  uint8_t state = DEFAULT_AIRSPEED_SENSOR;
  uint8_t TECmode = DEFAULT_TEC_MODE;
  int refPressure;
  float smoothingValue = DEFAULT_AIRSPEED_SMOOTHING;
  float tubeCorrection = DEFAULT_TUBE_CORRECTION;
} airSpeedSensor;

#if defined(SPEEDVARIO) || defined(SERVOSIGNAL)

  volatile int sv_PulseWidth = 0;
  volatile int sv_PauseWidth = 0;
  volatile int sv_RisingT = 0;
  volatile int sv_FallingT = 0;
  volatile int sv_PulsStartT = 0;
  volatile int sv_SignalPeriod = 0;
  #ifdef SIGNAL_FREQ
    volatile int sv_SignalFreq = 0;
    const uint8_t SIGNAL_FREQ_BUF_SIZE = 100;
    uint8_t sv_SignalFreqArray[SIGNAL_FREQ_BUF_SIZE];
  #endif
  int sv_sigPeriodSum = 0;
  int sv_PulsCnt = 0;
  volatile static int sv_SignalLossCnt = 0;
  volatile static int sv_SignalDurationMax = 0;
  volatile static int sv_SignalDuration = 0;
  static int sv_LastFreqMeasureTime = 0;
  static int sv_lastPulsCnt = 0;
  float sv_VarioEnergyCompensationValue = 0.0;
  float sv_SpeedinMPS = 0.0;
#endif

const float factorVoltageDivider[] = { float(voltageInputR1[0] + voltageInputR2[0]) / voltageInputR2[0],
                                       float(voltageInputR1[1] + voltageInputR2[1]) / voltageInputR2[1],
                                       float(voltageInputR1[2] + voltageInputR2[2]) / voltageInputR2[2],
                                       float(voltageInputR1[3] + voltageInputR2[3]) / voltageInputR2[3],
                                       float(voltageInputR1[4] + voltageInputR2[4]) / voltageInputR2[4],
                                       float(voltageInputR1[5] + voltageInputR2[5]) / voltageInputR2[5],
                                       float(voltageInputR1[6] + voltageInputR2[6]) / voltageInputR2[6],
                                       float(voltageInputR1[7] + voltageInputR2[7]) / voltageInputR2[7]
                                     };

const float timefactorCapacityConsumption = (1000.0 / MEASURING_INTERVAL * 60 * 60) / 1000;
float capacityConsumption = 0;

// Restart CPU
void(* restartCPU) (void) = 0;
#include "HandleMenu.h"

long readAnalog_mV(uint8_t sensorVolt, uint8_t pin) {
  return (analogRead(pin) / 1023.0) * V_REF * factorVoltageDivider[sensorVolt];
}

uint8_t getVoltageSensorTyp() {
  if (currentSensor <= APM25_A) {
    return currentSensor - 1;
#if V_REF >= 4500
  } else if (currentSensor >= ACS712_05 && currentSensor <= ACS712_30) {
    return ACS712_voltage;
#endif
  } else if (currentSensor >= ACS758_50B) {
    return ACS758_voltage;
  }
}


#if defined(SPEEDVARIO) || defined(SERVOSIGNAL)

void sv_rising() {
  sv_SignalDuration = millis() - sv_PulsStartT;
  sv_PulsStartT = millis();
  sv_RisingT = micros();
  // attachInterrupt(0, sv_falling, FALLING);
  attachInterrupt(digitalPinToInterrupt(SERVO_SIGNAL_PIN), sv_falling, FALLING);
  sv_PauseWidth = sv_RisingT - sv_FallingT;
  #ifdef JETI_EX_SERIAL_OUT
    Serial.print("servosignal rising at: ");
    Serial.print(sv_RisingT);
    Serial.println();
  #endif
}

#define MAXLOOP 20
volatile int sigCnt = 0;
void sv_falling() {
  volatile int now = micros();
  sv_PulsCnt++;
  sv_FallingT = now;
  sv_PulseWidth = sv_FallingT - sv_RisingT;

  // attachInterrupt(0, sv_rising, RISING);
  attachInterrupt(digitalPinToInterrupt(SERVO_SIGNAL_PIN), sv_rising, RISING);
}

int getRCServoPulse() {
  // RX output : sv_PulseWidth:~ 1000 - 2000
  int retVal;
  //noInterrupts();
  retVal = sv_PulseWidth;
  // interrupts();
  return retVal;
}

bool checkRCServoSignal() {
  static bool sv_SignalLossState = false;
  int fallingPulseTime = sv_PulsStartT;
  int sigDuration = (int) millis() - fallingPulseTime;
  sv_SignalDuration = max(sigDuration, sv_SignalDuration);

  if (sv_SignalDurationMax < sv_SignalDuration ) {
    sv_SignalDurationMax = sv_SignalDuration;
  }
  if (sv_SignalDuration > 100) {
    if (!sv_SignalLossState) {
      sv_SignalLossCnt++;
    }
    sv_SignalLossState = true;
    return false;
  }
  sv_SignalLossState = false;
  return true;
}

#endif

void setup()
{
  #ifdef JETI_EX_SERIAL_OUT
    Serial.begin(115200);//sets the baud rate
    Serial.println("== SpeedVario in TEST Mode ==");
  #endif
  #if defined(SPEEDVARIO) || defined(SERVOSIGNAL)
    // when pin D2 goes high, call the rising function
    pinMode(SERVO_SIGNAL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SERVO_SIGNAL_PIN), sv_rising, RISING);
  #ifdef JETI_EX_SERIAL_OUT
    Serial.print("servosignal on pin: ");
    Serial.print(SERVO_SIGNAL_PIN);
    Serial.println();
  #endif
  #endif

  // identify sensor
  #ifdef SUPPORT_BMx280
    pressureSensor.type = boschPressureSensor.begin(0x76);
    if(pressureSensor.type == unknown){
      pressureSensor.type = boschPressureSensor.begin(0x77);
    }
  #endif
  #ifdef SUPPORT_MS5611_LPS
    if(pressureSensor.type == unknown){
      if (lps.init()) {
        Wire.begin();
        lps.enableDefault();
        pressureSensor.type = LPS_;
      } else {
        Wire.beginTransmission(MS5611_ADDRESS); // if no Bosch sensor, check if return an ACK on MS5611 address
        if (Wire.endTransmission() == 0) {
          ms5611.begin(MS5611_ULTRA_HIGH_RES);
          pressureSensor.type = MS5611_;
        }
      }
    }
  #endif

  #if defined(SUPPORT_BMx280) || defined(SUPPORT_MS5611_LPS)
  switch (pressureSensor.type){
    case BMP280:
      pressureSensor.smoothingValue = BMx280_SMOOTHING;
      pressureSensor.deadzone = BMx280_DEADZONE;
      break;
    case BME280:
      pressureSensor.smoothingValue = BMx280_SMOOTHING;
      pressureSensor.deadzone = BMx280_DEADZONE;
      break;
    case MS5611_ :
      pressureSensor.smoothingValue = MS5611_SMOOTHING;
      pressureSensor.deadzone = MS5611_DEADZONE;
      break;
    case LPS_ :
      pressureSensor.smoothingValue = LPS_SMOOTHING;
      pressureSensor.deadzone = LPS_DEADZONE;
      break;
  }
  #endif
  #if defined(SPEEDVARIO) || defined(SERVOSIGNAL)
  // default settings, if there are no presets
  #ifdef SIGNAL_FREQ
    for (int i = 0 ; i < SIGNAL_FREQ_BUF_SIZE; i++) {
      sv_SignalFreqArray[i] = -1;
    }
  #endif // SIGNAL_FREQ

  #endif  // SPEEDVARIO

  // read settings from eeprom
  #ifdef SUPPORT_GPS
  if (EEPROM.read(P_GPS_MODE) != 0xFF) {
    gpsSettings.mode = EEPROM.read(P_GPS_MODE);
  }
  if (EEPROM.read(P_GPS_3D) != 0xFF) {
    gpsSettings.distance3D = EEPROM.read(P_GPS_3D);
  }
  #endif

  #ifdef SUPPORT_MAIN_DRIVE
  if (EEPROM.read(P_CURRENT_SENSOR) != 0xFF) {
    currentSensor = EEPROM.read(P_CURRENT_SENSOR);
  }
  if (EEPROM.read(P_CAPACITY_MODE) != 0xFF) {
    capacityMode = EEPROM.read(P_CAPACITY_MODE);
  }
  #endif

  #ifdef SUPPORT_RX_VOLTAGE
  if (EEPROM.read(P_ENABLE_RX1) != 0xFF) {
    enableRx1 = EEPROM.read(P_ENABLE_RX1);
  }
  if (EEPROM.read(P_ENABLE_RX2) != 0xFF) {
    enableRx2 = EEPROM.read(P_ENABLE_RX2);
  }
  #endif

  #ifdef SUPPORT_EXT_TEMP
  if (EEPROM.read(P_ENABLE_TEMP) != 0xFF) {
    enableExtTemp = EEPROM.read(P_ENABLE_TEMP);
  }
  #endif

  if (EEPROM.read(P_VARIO_SMOOTHING) != 0xFF) {
    pressureSensor.smoothingValue = (float)EEPROM.read(P_VARIO_SMOOTHING) / 100;
  }
  if (EEPROM.read(P_VARIO_DEADZONE) != 0xFF) {
    pressureSensor.deadzone = EEPROM.read(P_VARIO_DEADZONE);
  }

  #ifdef SUPPORT_MPXV7002_MPXV5004
  if (EEPROM.read(P_AIRSPEED_SENSOR) != 0xFF) {
    airSpeedSensor.state = EEPROM.read(P_AIRSPEED_SENSOR);
  }

  if (EEPROM.read(P_AIRSPEED_TUBE_CORR) != 0xFF) {
    airSpeedSensor.tubeCorrection = (float)EEPROM.read(P_AIRSPEED_TUBE_CORR) / 100;
  }

  if (EEPROM.read(P_AIRSPEED_SMOOTHING) != 0xFF) {
    airSpeedSensor.smoothingValue = (float)EEPROM.read(P_AIRSPEED_SMOOTHING) / 100;
  }
  #endif

  #ifdef SUPPORT_TEC
  if (EEPROM.read(P_TEC_MODE) != 0xFF) {
    airSpeedSensor.TECmode = EEPROM.read(P_TEC_MODE);
  }
  #endif

  #ifdef SUPPORT_GPS
    // init GPS
    gpsSerial.begin(GPSBaud);
  #endif

  #ifdef SUPPORT_MPXV7002_MPXV5004
  // init airspeed sensor
  if(airSpeedSensor.state){
    delay(50);
    airSpeedSensor.refPressure = analogRead(AIRSPEED_PIN);
  }
  #endif
  // Setup sensors
  if (pressureSensor.type == unknown) {
    jetiEx.SetSensorActive( ID_VARIO, false, sensors );
    #ifdef SPEEDVARIO
    jetiEx.SetSensorActive( ID_TEC_VARIO, false, sensors );
    #endif
  }

  if (gpsSettings.mode == GPS_basic || pressureSensor.type != BME280) {
    jetiEx.SetSensorActive( ID_HUMIDITY, false, sensors );
  }
  if (gpsSettings.mode == GPS_basic || pressureSensor.type == unknown) {
    jetiEx.SetSensorActive( ID_PRESSURE, false, sensors );
    jetiEx.SetSensorActive( ID_TEMPERATURE, false, sensors );
  }

  if (gpsSettings.mode == GPS_disabled) {
    jetiEx.SetSensorActive( ID_GPSLAT, false, sensors );
    jetiEx.SetSensorActive( ID_GPSLON, false, sensors );
    jetiEx.SetSensorActive( ID_GPSSPEED, false, sensors );
  }

  if (gpsSettings.mode == GPS_disabled && pressureSensor.type == unknown) {
    jetiEx.SetSensorActive( ID_ALTREL, false, sensors );
    jetiEx.SetSensorActive( ID_ALTABS, false, sensors );
  }

  if (gpsSettings.mode != GPS_extended) {
    jetiEx.SetSensorActive( ID_DIST, false, sensors );
    jetiEx.SetSensorActive( ID_TRIP, false, sensors );
    jetiEx.SetSensorActive( ID_HEADING, false, sensors );
    jetiEx.SetSensorActive( ID_COURSE, false, sensors );
    jetiEx.SetSensorActive( ID_SATS, false, sensors );
    jetiEx.SetSensorActive( ID_HDOP, false, sensors );
  }

  if (currentSensor == mainDrive_disabled) {
    jetiEx.SetSensorActive( ID_VOLTAGE, false, sensors );
    jetiEx.SetSensorActive( ID_CURRENT, false, sensors );
    jetiEx.SetSensorActive( ID_CAPACITY, false, sensors );
    jetiEx.SetSensorActive( ID_POWER, false, sensors );
    #ifdef SUPPORT_MAIN_DRIVE
  }else{
    if(capacityMode > startup){
      // read capacity from eeprom
      int eeAddress = EEPROM_ADRESS_CAPACITY;
      EEPROM.get(eeAddress, capacityConsumption);
      if (capacityMode == automatic) {
        eeAddress += sizeof(float);
        float cuVolt = readAnalog_mV(getVoltageSensorTyp(), VOLTAGE_PIN) / 1000.0;
        float oldVolt;
        EEPROM.get(eeAddress, oldVolt);
        if (cuVolt >= oldVolt * ((100.0 + VOLTAGE_DIFFERENCE) / 100.0)) {
          capacityConsumption = 0;
        }
      }
    }
    #endif
  }

  if (!enableRx1) {
    jetiEx.SetSensorActive( ID_RX1_VOLTAGE, false, sensors );
  }

  if (!enableRx2) {
    jetiEx.SetSensorActive( ID_RX2_VOLTAGE, false, sensors );
  }

  if (!enableExtTemp) {
    jetiEx.SetSensorActive( ID_EXT_TEMP, false, sensors );
  }

  // init Jeti EX Bus
  jetiEx.SetDeviceId( JETI_DEV_ID_LOW, JETI_DEV_ID_HIGH );
  jetiEx.Start( "VarioGPS", sensors, JetiExProtocol::SERIAL2 );

}

static double ourMS5611Pressure = 0;
static bool ourStartupPhase = true;

void pollSensors() {
  #ifdef SUPPORT_MS5611_LPS
  // GY63 new handling
  static unsigned long pollTimeLast = 0;
  static bool isPollSensorsFirst = true;
  unsigned long pollTimeNow = millis();
  double pressure;
  if ( ( pollTimeNow - pollTimeLast) > 25)  {
      pressure = ms5611.readPressure(true); // In Pascal (100 Pa = 1 hPa = 1 mbar)
      if (isPollSensorsFirst) {
        isPollSensorsFirst = false;
        ourMS5611Pressure = pressure;
      }
      // Vario Filter
      // IIR Low Pass Filter
      // y[i] := α * x[i] + (1-α) * y[i-1]
      //      := α * x[i] + (1 * y[i-1]) - (α * y[i-1])
      //      := α * x[i] +  y[i-1] - α * y[i-1]
      //      := α * ( x[i] - y[i-1]) + y[i-1]
      //      := y[i-1] + α * (x[i] - y[i-1])
      // mit α = 1- β
      //      := y[i-1] + (1-ß) * (x[i] - y[i-1])
      //      := y[i-1] + 1 * (x[i] - y[i-1]) - ß * (x[i] - y[i-1])
      //      := y[i-1] + x[i] - y[i-1] - ß * x[i] + ß * y[i-1]
      //      := x[i] - ß * x[i] + ß * y[i-1]
      //      := x[i] + ß * y[i-1] - ß * x[i]
      //      := x[i] + ß * (y[i-1] - x[i])

      // float f = 0.984;
      float f = 0.9f + 0.1f * pressureSensor.smoothingValue;
      ourMS5611Pressure = (double) pressure + f * (double)(ourMS5611Pressure - pressure);
      pollTimeLast = pollTimeNow;
        #ifdef JETI_EX_SERIAL_OUT_NO
          Serial.print("pressure: ");
          Serial.print(pressure);
          Serial.print("smooth pressure: ");
          Serial.print(ourMS5611Pressure);
          Serial.println();
        #endif
    #endif
  }
}

void loop()
{
  static long startAltitude = 0;
  static long uRelAltitude = 0;
  static long uAbsAltitude = 0;

  long uPressure = PRESSURE_SEALEVEL;
  int uTemperature = 20;
  #ifdef SUPPORT_TEC
  static unsigned long dT = 0;
  static float dV;
  #endif

  #if defined(SPEEDVARIO) || defined(SERVOSIGNAL)
    bool checkControlServo = checkRCServoSignal();
  #endif

  unsigned long dTmeasure = millis() - lastTime;     // delta time in ms
  static int measureCnt = 0;
  if(dTmeasure > MEASURING_INTERVAL) {
    #ifdef TEST_TELEMETRY_VALUE
    jetiEx.SetSensorValue( ID_TESTVALUE, dTmeasure);
    #endif
    measureCnt++;
    #ifdef SUPPORT_MPXV7002_MPXV5004
    if(airSpeedSensor.state){
      static int uAirSpeed = 0;
      static float sv_AirSpeedPressure = 0;
      static int lastAirSpeed = 0;
      static float lastAirSpeedPressure = 0;

      // get air speed from MPXV7002/MPXV5004
      // based on code from johnlenfr, http://johnlenfr.1s.fr
      int airSpeedPressure = analogRead(AIRSPEED_PIN);
      if (airSpeedPressure < airSpeedSensor.refPressure) airSpeedPressure = airSpeedSensor.refPressure;

      float pitotpressure =
        5000.0 * ((airSpeedPressure - airSpeedSensor.refPressure) / 1024.0f);   // differential pressure in Pa, 1 V/kPa, max 3920 Pa
      float density = (uPressure * DRY_AIR_MOLAR_MASS) / ((uTemperature + 273.16) * UNIVERSAL_GAS_CONSTANT);
      uAirSpeed = sqrt ((2 * pitotpressure) / density) * airSpeedSensor.tubeCorrection;
      // uAirSpeed = sqrt ((2 * (pitotpressure - uPressure)) / density) * airSpeedSensor.tubeCorrection;

      // IIR Low Pass Filter
      uAirSpeed = uAirSpeed + airSpeedSensor.smoothingValue * (lastAirSpeed - uAirSpeed);

      #ifdef SUPPORT_TEC
      if(airSpeedSensor.TECmode == TEC_airSpeed){
        // TEC Vario: total energy compensated vario value
        // H: Height, t: time, V: Velocity, g: gravity
        // dH/dt = -(V/g) * dV/dt
        // see: http://www.how2soar.de/images/H2S_media/02_pdf/TE-Vario_im_Stroemungsfeld.pdf
        float dVairspeed = uAirSpeed - lastAirSpeed; // dV in m/s
        if (dTmeasure != 0 && dTmeasure < 3000) { // avoid divison by zero
          // = 100 * ( sv_SpeedinMPS / 9.81 ) * dV *  1000 / dT;
          // = (sv_SpeedinMPS / 9.81 ) * 100000 * dV / dT;
          sv_VarioEnergyCompensationValue = uAirSpeed * 10194 * dVairspeed / dTmeasure;
        }
      }
      #endif

      lastAirSpeed = uAirSpeed;

      #ifdef UNIT_US
        jetiEx.SetSensorValue( ID_AIRSPEED, uAirSpeed*2.23694 );    // speed in mph
      #else
        jetiEx.SetSensorValue( ID_AIRSPEED, uAirSpeed*3.6 );        // speed in km/h
      #endif
    }
    #endif // SUPPORT_MPXV7002_MPXV5004

    #if defined(SUPPORT_MS5611_LPS) || defined(SUPPORT_BMx280)
      if(pressureSensor.type != unknown) {
        static bool setStartAltitude = false;
        static long lastAltitude = 0;
        long curAltitude;
        long uPressure = 0;
        int uTemperature;
        long uVario = 0;
        int uHumidity;

        // Read sensormodule values
        switch (pressureSensor.type){
          #ifdef SUPPORT_MS5611_LPS
          case MS5611_:
            // uPressure = ms5611.readPressure(true); // In Pascal (100 Pa = 1 hPa = 1 mbar)
            curAltitude = ms5611.getAltitude(ourMS5611Pressure, 101325) * 100; // In Centimeter
            uTemperature = ms5611.readTemperature(true) * 10; // In Celsius ( x10 for one decimal)
            uPressure = ourMS5611Pressure;
            break;
          case LPS_:
            uPressure = lps.readPressureMillibars(); // In Pascal (100 Pa = 1 hPa = 1 mbar)
            curAltitude = lps.pressureToAltitudeMeters(uPressure) * 100; // In Centimeter
            uTemperature = lps.readTemperatureC() * 10; // In Celsius ( x10 for one decimal)
            break;
         #endif
         #ifdef SUPPORT_BMx280
          default:
            uPressure = boschPressureSensor.readPressure(); // In Pascal (100 Pa = 1 hPa = 1 mbar)
            curAltitude = boschPressureSensor.readAltitude() * 100; // In Centimeter
            uTemperature = boschPressureSensor.readTemperature() * 10; // In Celsius ( x10 for one decimal)
            if (pressureSensor.type == BME280) {
              jetiEx.SetSensorValue( ID_HUMIDITY, boschPressureSensor.readHumidity() * 10 ); // In %rH
            }
            break;
        #endif
      }

      if (!setStartAltitude && measureCnt > 30) {
        // Set start-altitude in sensor-start
        setStartAltitude = true;
        startAltitude = curAltitude;
        lastAltitude = curAltitude;
      } else {
        // Altitud  e
        uRelAltitude = (curAltitude - startAltitude) / 10;
        uAbsAltitude = curAltitude / 100;
      }

      // uVario in cm/s (curAltitude and lastAltitude in cm)

      uVario = (curAltitude - lastAltitude) * (1000 / float(dTmeasure));
      lastAltitude = curAltitude;

      // Vario Filter
      // IIR Low Pass Filter
      // y[i] := α * x[i] + (1-α) * y[i-1]
      //      := α * x[i] + (1 * y[i-1]) - (α * y[i-1])
      //      := α * x[i] +  y[i-1] - α * y[i-1]
      //      := α * ( x[i] - y[i-1]) + y[i-1]
      //      := y[i-1] + α * (x[i] - y[i-1])
      // mit α = 1- β
      //      := y[i-1] + (1-ß) * (x[i] - y[i-1])
      //      := y[i-1] + 1 * (x[i] - y[i-1]) - ß * (x[i] - y[i-1])
      //      := y[i-1] + x[i] - y[i-1] - ß * x[i] + ß * y[i-1]
      //      := x[i] - ß * x[i] + ß * y[i-1]
      //      := x[i] + ß * y[i-1] - ß * x[i]
      //      := x[i] + ß * (y[i-1] - x[i])
      // see: https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter

      #ifdef SUPPORT_MS5611_LPS
      // GY63 new handling
       // do nothing the pressure Value is smoothed instead
      #else
      static float lastVariofilter = 0;
      uVario = uVario + pressureSensor.smoothingValue * (lastVariofilter - uVario);
      lastVariofilter = uVario;
      #endif
      #ifdef JETI_EX_SERIAL_OUT
        Serial.print(" ourMS5611Pressure: ");
        Serial.print(ourMS5611Pressure);
        Serial.print(" curAltitude: ");
        Serial.print(curAltitude);
        Serial.print(" uVario: ");
        Serial.print(uVario);
        Serial.println();
      #endif

      // Dead zone filter
      if (uVario > pressureSensor.deadzone) {
        uVario -= pressureSensor.deadzone;
      } else if (uVario < (pressureSensor.deadzone * -1)) {
        uVario -= (pressureSensor.deadzone * -1);
      } else {
        uVario = 0;
      }

      #ifdef UNIT_US
        // EU to US conversions
        // ft/s = m/s / 0.3048
        // inHG = hPa * 0,029529983071445
        // ft = m / 0.3048
        uVario /= 0.3048;
        uPressure *= 0.029529983071445;
        uTemperature = uTemperature * 1.8 + 320;
      #endif
      jetiEx.SetSensorValue( ID_VARIO, uVario );
      #ifdef SPEEDVARIO
      jetiEx.SetSensorValue( ID_TEC_VARIO, uVario + sv_VarioEnergyCompensationValue);
      #endif

      jetiEx.SetSensorValue( ID_PRESSURE, uPressure );
      jetiEx.SetSensorValue( ID_TEMPERATURE, uTemperature );

    }

    #if defined(SPEEDVARIO) || defined(SERVOSIGNAL)
      static unsigned int loopCnt = 0;
      jetiEx.SetSensorValue( ID_SV_SIG_LOSS_CNT, sv_SignalLossCnt);
      jetiEx.SetSensorValue( ID_SV_SIGNAL_DURATION_MAX, sv_SignalDurationMax);
      jetiEx.SetSensorValue( ID_SV_SIGNAL_DURATION, sv_SignalDuration);
      int timeSinceLastMeasure = millis() - sv_LastFreqMeasureTime;
      sv_LastFreqMeasureTime = millis();
      #ifdef SIGNAL_FREQ
        sv_SignalFreq = (sv_PulsCnt - sv_lastPulsCnt) * 1000 / timeSinceLastMeasure;
      #endif // SIGNAL_FREQ
      sv_lastPulsCnt = sv_PulsCnt;

      #ifdef SIGNAL_FREQ
        static uint8_t sigFreqArrayPtr = 0;
        static uint8_t sigFreqArrayRetardedPtr = 0;
        sigFreqArrayPtr++;
        if (sigFreqArrayPtr == SIGNAL_FREQ_BUF_SIZE) {
          sigFreqArrayPtr = 0;
        }
        sigFreqArrayRetardedPtr = sigFreqArrayPtr+1;
        if (sigFreqArrayRetardedPtr == SIGNAL_FREQ_BUF_SIZE) {
          sigFreqArrayRetardedPtr = 0;
        }
        sv_SignalFreqArray[sigFreqArrayPtr] = sv_SignalFreq;
        jetiEx.SetSensorValue( ID_SV_SIGNAL_FRQ, sv_SignalFreqArray[sigFreqArrayPtr]);
        if (sv_SignalFreqArray[ID_SV_SIGNAL_FRQ_RETARDED] != -1) {
          jetiEx.SetSensorValue( ID_SV_SIGNAL_FRQ_RETARDED, sv_SignalFreqArray[sigFreqArrayRetardedPtr]);
        }
      #endif // SIGNAL_FREQ
      #ifdef SAW_TOOTH
        sawtoothVal++;
        if (sawtoothVal > 50) {
         sawtoothVal = 0;
        }
        jetiEx.SetSensorValue( ID_SAWTOOTH, sawtoothVal);
      #endif // SAW_TOOTH

        #ifdef JETI_EX_SERIAL_OUT
          Serial.print("  === SIGNAL_ANALYSIS: ");
          Serial.print(millis());
          Serial.print(" : SIG PIN = ");
          Serial.print(digitalRead(SERVO_SIGNAL_PIN));
          Serial.print(" : SIG_LOSS_CNT = ");
          Serial.print(sv_SignalLossCnt);
          Serial.print(" : SIGNAL_DURATION_MAX = ");
          Serial.print(sv_SignalDurationMax);
          Serial.print(" : SIGNAL_DURATION = ");
          Serial.print(sv_SignalDuration);
          Serial.print("ms");
          Serial.println();
        #endif
    #endif // SPEEDVARIO | SERVOSIGNAL


  #endif // SUPPORT_MS5611_LPS | SUPPORT_BMx280

    lastTime = millis();

    // analog input
    #ifdef SUPPORT_MAIN_DRIVE
    if(currentSensor){
      // Voltage
      float cuVolt = readAnalog_mV(getVoltageSensorTyp(), VOLTAGE_PIN) / 1000.0;
      jetiEx.SetSensorValue( ID_VOLTAGE, cuVolt * 10);

      // Current
      uint16_t ampOffset;

      if (currentSensor <= APM25_A){
        ampOffset = Atto_APM_offset;
      } else if (currentSensor > ACS758_200B) {
        ampOffset = ACS_U_offset;
      } else {
        ampOffset = ACS_B_offset;
      }

      float mVanalogIn = (analogRead(CURRENT_PIN) / 1023.0) * V_REF; // mV
      float cuAmp = (mVanalogIn - ampOffset) / mVperAmp[currentSensor-1];
      if (currentSensor > APM25_A){
        cuAmp *= 5000.0/V_REF;
      }

      jetiEx.SetSensorValue( ID_CURRENT, cuAmp * 10);

      // Capacity consumption
      capacityConsumption += cuAmp / timefactorCapacityConsumption;
      jetiEx.SetSensorValue( ID_CAPACITY, capacityConsumption);

      // save capacity and voltage to eeprom
      if (capacityMode > startup && (millis() - lastTimeCapacity) > CAPACITY_SAVE_INTERVAL) {
        if (cuAmp <= MAX_CUR_TO_SAVE_CAPACITY) {
          int eeAddress = EEPROM_ADRESS_CAPACITY;
          EEPROM.put(eeAddress, capacityConsumption);
          eeAddress += sizeof(float);
          EEPROM.put(eeAddress, cuVolt);
        }
        lastTimeCapacity = millis();
      }

      // Power
      jetiEx.SetSensorValue( ID_POWER, cuAmp * cuVolt);
    }
    #endif

    #ifdef SUPPORT_RX_VOLTAGE
    if(enableRx1){
      jetiEx.SetSensorValue( ID_RX1_VOLTAGE, readAnalog_mV(Rx1_voltage,RX1_VOLTAGE_PIN)/10);
    }

    if (enableRx2) {
      jetiEx.SetSensorValue( ID_RX2_VOLTAGE, readAnalog_mV(Rx2_voltage, RX2_VOLTAGE_PIN) / 10);
    }
    #endif

    #ifdef SUPPORT_EXT_TEMP
    if(enableExtTemp){
      // convert the value to resistance
      float aIn = 1023.0 / analogRead(TEMPERATURE_PIN) - 1.0;
      aIn = SERIESRESISTOR / aIn;

      // convert resistance to temperature
      float steinhart;
      steinhart = aIn / THERMISTORNOMINAL;                // (R/Ro)
      steinhart = log(steinhart);                         // ln(R/Ro)
      steinhart /= BCOEFFICIENT;                          // 1/B * ln(R/Ro)
      steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);   // + (1/To)
      steinhart = 1.0 / steinhart;                        // Invert
      steinhart -= 273.15 - SELF_HEAT;                    // convert to °C and self-heat compensation

      #ifdef UNIT_US
        // EU to US conversions
        steinhart = steinhart * 1.8 + 320;
      #endif

      jetiEx.SetSensorValue( ID_EXT_TEMP, steinhart*10);
    }
    #endif
  }

#ifdef SUPPORT_GPS
  if (gpsSettings.mode != GPS_disabled) {

    static int homeSetCount = 0;
    static float home_lat;
    static float home_lon;
    static float last_lat;
    static float last_lon;
    static long lastAbsAltitude = 0;
    static unsigned long tripDist;
    unsigned long distToHome;

    // read data from GPS
    while (gpsSerial.available() )
    {
      char c = gpsSerial.read();
      if (gps.encode(c)) {
        break;
      } else {
        return;
      }
    }


    if (gps.location.isValid() && gps.location.age() < 2000) { // if Fix

      jetiEx.SetSensorValueGPS( ID_GPSLAT, false, gps.location.lat() );
      jetiEx.SetSensorValueGPS( ID_GPSLON, true, gps.location.lng() );

      // Altitude
      static long lastGPSAltitude = 0;
      static unsigned long lastGPSAltitudeTime = 0;
      uAbsAltitude = gps.altitude.meters();
      int gpsAltitudeDiff = uAbsAltitude - lastGPSAltitude;
      int gpsTimeDiff = lastGPSAltitudeTime - millis();
      int gpsVarioValue = (gpsAltitudeDiff * 10) / gpsTimeDiff;
      // jetiEx.SetSensorValue( ID_GPSVARIO, gpsVarioValue );

      lastGPSAltitudeTime = millis();


      if (gps.speed.isUpdated() && airSpeedSensor.TECmode == TEC_GPS) {
#ifdef SPEEDVARIO
        static uint32_t timeSpeedActinMS, timeSpeedLastinMS;
        static double lastSpeedinMPS = 0.0;
        gpsSpeed = gps.speed.value(); // in Centi Knots
        sv_SpeedinMPS = _GPS_MPS_PER_KNOT * gpsSpeed / 100.0; // speed in m/S
        timeSpeedActinMS = millis() - gps.speed.age();
        // TEC Vario: total energy compensated vario value
        // H: Height, t: time, V: Velocity, g: gravity
        // dH/dt = -(V/g) * dV/dt
        // see: http://www.how2soar.de/images/H2S_media/02_pdf/TE-Vario_im_Stroemungsfeld.pdf
        double dV = sv_SpeedinMPS - lastSpeedinMPS; // dV in m/s
        uint32_t dT = timeSpeedActinMS - timeSpeedLastinMS; // dT in ms
        if (dT != 0 && dT < 3000) { // avoid divison by zero
          // = 100 * ( sv_SpeedinMPS / 9.81 ) * dV *  1000 / dT;
          // = (sv_SpeedinMPS / 9.81 ) * 100000 * dV / dT;
          sv_VarioEnergyCompensationValue = sv_SpeedinMPS  * 10194 * dV / dT;
        }
        lastSpeedinMPS = sv_SpeedinMPS;
        timeSpeedLastinMS = timeSpeedActinMS;
#endif

      }

      #ifdef UNIT_US
        jetiEx.SetSensorValue( ID_GPSSPEED, gps.speed.mph() );
      #else
        jetiEx.SetSensorValue( ID_GPSSPEED, gps.speed.kmph() );
      #endif

      jetiEx.SetSensorValue( ID_HEADING, gps.course.deg() );

      if (homeSetCount < 3000) {  // set home position
        ++homeSetCount;
        home_lat = gps.location.lat();
        home_lon = gps.location.lng();
        last_lat = home_lat;
        last_lon = home_lon;
        lastAbsAltitude = gps.altitude.meters();
        tripDist = 0;
        if (pressureSensor.type == unknown) {
          startAltitude = gps.altitude.meters();
        }

      } else {

        // Rel. Altitude
        if (pressureSensor.type == unknown) {
          uRelAltitude = (uAbsAltitude - startAltitude) * 10;
        }

        // Distance to model
        distToHome = gps.distanceBetween(
                       gps.location.lat(),
                       gps.location.lng(),
                       home_lat,
                       home_lon);
        if (gpsSettings.distance3D) {
          distToHome = sqrt(pow(uRelAltitude / 10, 2) + pow(distToHome, 2));
        }

        // Course from home to model
        jetiEx.SetSensorValue( ID_COURSE, gps.courseTo(home_lat, home_lon, gps.location.lat(), gps.location.lng()));

        // Trip distance
        float distLast = gps.distanceBetween(
                           gps.location.lat(),
                           gps.location.lng(),
                           last_lat,
                           last_lon);
        if (gpsSettings.distance3D) {
          distLast = sqrt(pow(uAbsAltitude - lastAbsAltitude, 2) + pow(distLast, 2));
          lastAbsAltitude = uAbsAltitude;
        }
        tripDist += distLast;
        last_lat = gps.location.lat();
        last_lon = gps.location.lng();
      }

    } else { // If Fix end
      #ifdef SPEEDVARIO
      if (airSpeedSensor.TECmode == TEC_GPS) {
        sv_VarioEnergyCompensationValue = 0.0; // reset TEC value, in case of Fix end
      }
      #endif
      jetiEx.SetSensorValueGPS( ID_GPSLAT, false, 0 );
      jetiEx.SetSensorValueGPS( ID_GPSLON, true, 0 );
      if (pressureSensor.type == unknown) {
        uRelAltitude = 0;
      }
      uAbsAltitude = 0;
      distToHome = 0;
      jetiEx.SetSensorValue( ID_COURSE, 0 );
      jetiEx.SetSensorValue( ID_GPSSPEED, 0 );
      jetiEx.SetSensorValue( ID_HEADING, 0 );
    }

    jetiEx.SetSensorValue( ID_SATS, gps.satellites.value() );
    jetiEx.SetSensorValue( ID_HDOP, gps.hdop.value());
    #ifndef UNIT_US
      //EU units
      jetiEx.SetSensorValue( ID_TRIP, tripDist/10 );
      jetiEx.SetSensorValue( ID_DIST, distToHome );
    #endif
    #ifdef UNIT_US
      //US units
      jetiEx.SetSensorValue( ID_TRIP, tripDist*0.06213 );
      jetiEx.SetSensorValue( ID_DIST, distToHome*3.2808399);
    #endif

  }
  #endif

  #ifdef UNIT_US
    // EU to US conversions
    // ft/s = m/s / 0.3048
    // inHG = hPa * 0,029529983071445
    // ft = m / 0.3048
    uRelAltitude /= 0.3048;
    uAbsAltitude /= 0.3048;
  #endif

  jetiEx.SetSensorValue( ID_ALTREL, uRelAltitude );
  jetiEx.SetSensorValue( ID_ALTABS, uAbsAltitude );

  pollSensors();

  #ifndef JETI_EX_SERIAL_OUT
  #ifdef SUPPORT_JETIBOX_MENU
  HandleMenu();
  #endif
  jetiEx.DoJetiSend();
  #endif
}
