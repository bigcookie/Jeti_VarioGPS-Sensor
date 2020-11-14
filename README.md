# VarioGPS-Sensor

Universeller Jeti Telemetrie Sensor mit vielen Möglichkeiten: Vario, GPS, Strom/Spannung/Kapazität/Leistung für Hauptantrieb, Empfängerspannung, Temperaturmessung und Empfangsqualität (RXQ). Der Sensor ist total einfach nachbaubar, und sollte auch von Elektronik-Anfängern problemlos zu bewerkstelligen sein. 

## Telemetrie

![EX-vario](https://raw.githubusercontent.com/Pulsar07/Jeti_VarioGPS-Sensor/SpeedVario/Doc/img/EX_vario.bmp)
![EX-GPS](https://raw.githubusercontent.com/Pulsar07/Jeti_VarioGPS-Sensor/SpeedVario/Doc/img/EX_gps.bmp)
![EX-GPS2](https://raw.githubusercontent.com/Pulsar07/Jeti_VarioGPS-Sensor/SpeedVario/Doc/img/EX_gps2.bmp)
![EX-VA](https://raw.githubusercontent.com/Pulsar07/Jeti_VarioGPS-Sensor/SpeedVario/Doc/img/EX_volt_amp.bmp)

Mit Luftdrucksensor werden die Werte angezeigt:
- Rel. und Abs. Höhe
- Vario
- Luftdruck
- Temperatur
- Luftfeuchte (nur mit BME280)
  
Im GPS Basic Mode sind folgende Werte verfügbar:
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

Mit einem Strom/Spannungssensor können folgende Werte angezeigt werden:
- Strom
- Spannung
- Leistung
- Kapazität (wird gespeichert und automatisch bei vollem Akku zurückgesetzt -> es können dadurch mehrere Flüge mit dem gleichen Akku gemacht werden)

Für die Messung der Empfängerspannung stehen die Werte zur Verfügung:
- Rx1 voltage
- Rx2 voltage

Zur Messung von zB. der Motorentemperatur kann zusätzlich ein NTC-Temperaturwiederstand (-55 bis +155°C) angeschlossen werden.
  
### RXQ - Empfangsqualität
Für die Messung der Empfangsqualität (RXQ) muss ein Kabel vom Arduino Pin 2 mit einem 47kOhm Vorwiderstand zu einem freien Servoausgang hergestellt werden. Am Sender wird z.B. 100Hz Sendefrequenz eingestellt, auf dem Empfänger muss dann die Servo-Output Period auf "AUTO" oder "by transmitter" eingestellt werden und der benutzte Servo-Ausgang muss in den Fail-Safe-Einstellungen auf "AUS" gesetzt werden. 
Der Arduino misst nun die Zeitabstände zwischen den am Servoausgang ausgegebenen Servosignale (PWM) (und zwar alle !!!), was bei 100Hz 10ms sein sollte.
Beim Empfänger muss die Servo-Output-Period auf "AUTO" eingestellt sein, damit die PWM Signaldauer nicht vom Empfänger erzeugt wird. Zudem muss beim benutzten Empfängerausgang die Fail-Safe-Einstellung auf "AUS" eingestellt sein, damit im Problemfall, das Servosignal nicht vom RX erzeugt wird.

Mehr Details und Erläuterungen sind unter: http://www.so-fa.de/nh/JetiSensorRXQ zu finden.
Damit sind
- Signal Duration (SigDura) : letzter Abtastwert des Abstand zweier PWM Impulse in ms 
- Maximum Signal Duration (MaxSigDura) : maximaler Abtastwert des Abstands zweier PWM Impulse (lückenlos gezählt !!!)
- Anzahl Signalverlust (SigLossCnt) : werden Abstände > 100ms dedektiert, wird der SigLossCnt um 1 erhöht (sollte immer 0 sein !)

Zum Aktivieren dieses Features Code sind folgende #defines in der Datei defaults.h gedacht:
- #define SERVOSIGNAL : schaltet Feature im Code ein
- #define SERVOSIGNAL_PIN : (default: 2) definiert den zu nutzenden Arduino Eingangs-Pin
- #define SERVOSIGNAL_PIN_PULLUP : (default: "not defined") dies muss definiert sein, wenn der RX ohne Vorwiderstand angeschlossen wird

Die Erfahrung der letzten 2 Jahre, ca. 15 Modellen, Abstand zum Modell bis 1500m, bei gut verlegten Antennen, zeigt, dass selbst bei Sender-Signalverlustmeldungen, die SigDuraMax immer unter 100ms bleibt, was zeigt, dass die Jeti-Signalverlustmeldung eben meist am Verlust des Rückkanals liegt!


## JetiBox Einstellungen

![JetiBox](https://raw.githubusercontent.com/Pulsar07/Jeti_VarioGPS-Sensor/SpeedVario/Doc/img/JetiBox_settings.png)
Folgende Einstellungen können per Jetibox vorgenommen werden:
- GPS: deaktiviert, Basic oder Extended
- GPS Distanz: 2D oder 3D
- Vario Filterparameter X, Y und Deadzone
- Stromsensor für Hauptantrieb 
- Einstellung Reset der Kapazität:
    - STARTUP(Wert ist nach jedem Einschalten auf 0)
    - AUTO(Wert wird gespeichert und erst zurückgesetzt wenn ein geladener Akku angeschlossen wird)
    - MANUAL(Wert muss manuell per Jetibox zurückgesetzt werden mit RESET OFFSET)
- Rx1, Rx2 Empfängerspannungsmessung aktivieren
- Temperaturmessung aktivieren

## Hardware

- Arduino Pro Mini 3.3V-8Mhz oder 5V-16Mhz
- GPS-Modul mit NMEA Protokoll und UART@9600baud
- Luftdrucksensoren: BMP280, BME280, MS5611, LPS 
- Stromsensoren @3.3V/5V Betriebsspannung:
    - AttoPilot Module @3.3V: 45A/13.6V - 90A/50V - 180A/50V (@5V: 45A/20.6V - 90A/60V - 180A/60V)
    - APM2.5 PowerModul @5V: 90A/50V (@3.3V: 58A/33.4V)
    - ACS758_50B, ACS758_100B, ACS758_150B, ACS758_200B, ACS758_50U, ACS758_100U, ACS758_150U, ACS758_200U
- zusätzliche Stromsensoren @5V Betriebsspannung: ACS712_05, ACS712_20, ACS712_30

## Aufbau

Der VarioGPS Sensor kann individuell nach seinen eigenen Wünschen zusammengestellt werden. Es ist möglich den Sensor als reines Vario zu betreiben, nur zur Vermessung des Antriebs, oder als Überwachung der Empfängerstromversorgung. Die benötigten Sensoren werden einfach am Arduino angelötet, und per Jetibox aktiviert.

![schematic](https://raw.githubusercontent.com/Pulsar07/Jeti_VarioGPS-Sensor/SpeedVario/Doc/img/VarioGPS_schematic.png)

## Firmware laden

Je nach verwendetem Arduino Board wird die Firmware für 3.3V oder 5V benötigt. Im Ordner Arduino/Firmware sind die entsprechenden hex-files die direkt auf das Arduino Board geladen werden können. Für fortgeschrittene Benutzer steht auch der komplette Code zur Verfügung, der mit der Arduino IDE auf seine eigene Bedürfnisse angepasst werden kann.

Für den Firmware upload wird ein USB<>serial Adapter benötigt, und ein entsprechendes Upload-Programm. Danach hat man's schon geschaft, und kann den Sensor im Modell einbauen.

Treten nach einem Firmwareupdate Probleme auf, sollten per JetiBox mit "Load defaults" die Einstellungen zurückgesetzt, und danach neu eingestellt werden.

### MAC

Für MAC User gibt es das Programm HexUploader und kann hier heruntergeladen werden: https://github.com/Pulsar07/Jeti_VarioGPS-Sensor/tree/SpeedVario simples Programm zum laden der Firmware, einfach das Arduino Board per USB anschliessen, Arduino Type "Pro mini" und den USB-Serial Port auswählen. Nun das entsprechende hex-file auswählen und der upload startet automatisch.

![hexuploader2](https://raw.githubusercontent.com/Pulsar07/Jeti_VarioGPS-Sensor/SpeedVario/Doc/img/HexUploader.png)

### Windows

Für Windows User kann der XLoader verwendet werden: http://russemotto.com/xloader/XLoader.zip

Der Xloader ist ebenfalls ein simples Programm, einfach das hex-file, device und COM-port auswählen und auf "Upload" drücken.

![xloader](https://raw.githubusercontent.com/Pulsar07/Jeti_VarioGPS-Sensor/SpeedVario/Doc/img/xloader.png)
