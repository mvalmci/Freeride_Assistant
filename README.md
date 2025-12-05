# Freeride Assistant

![Project Banner](https://raw.githubusercontent.com/mvalmci/Freeride_Assistant/main/assets/banner.png)

**Freeride Assistant** ist ein Open‑Source Projekt zur Datenerfassung und Unterstützung beim Freeriden. Ziel ist es, valide Sensordaten (Lage, Beschleunigung, GPS‑Position, Tempo) in Echtzeit zu erfassen, zu verarbeiten und zu protokollieren. Dieses Repository enthält Code, Schaltpläne und Beispielkonfigurationen für den Einsatz mit dem DFRobot FireBeetle ESP32, dem Adafruit BNO055 IMU und dem u‑blox NEO‑6M GPS Modul.

---

## Inhaltsverzeichnis

1. [Kurzüberblick](#kurzüberblick)
2. [Verwendete Hardware](#verwendete-hardware)
3. [Elektrischer Aufbau & Pinout](#elektrischer-aufbau--pinout)
4. [Software & Libraries](#software--libraries)
5. [Installation & Upload](#installation--upload)
6. [Konfiguration & Kalibrierung](#konfiguration--kalibrierung)
7. [Datenformat & Speicherung](#datenformat--speicherung)
8. [Live‑Übertragung / Telemetrie](#live-%E2%80%90%C3%BCbertragung--telemetrie)
9. [Fehlerbehebung](#fehlerbehebung)
10. [Wartung & Tipps](#wartung--tipps)
11. [Lizenz & Mitwirkende](#lizenz--mitwirkende)

---

## Kurzüberblick

Freeride Assistant liest die IMU‑Daten (Orientierung, Beschleunigung, Gyro) vom BNO055, GPS‑Daten vom NEO‑6M und verarbeitet/bezieht diese auf einem ESP32 (DFRobot FireBeetle). Das System ist so ausgelegt, dass es:

* Sensorfusion‑Daten (Euler, Quaternionen) vom BNO055 nutzt
* GPS‑Positionsdaten & Zeitstempel (UTC) vom NEO‑6M erfasst
* Die Daten lokal speichert (microSD optional) und per Bluetooth streamt
* Kalibrierungsroutinen für die IMU bereitstellt

---

## Verwendete Hardware

* **DFRobot FireBeetle ESP32‑E** (MCU, WLAN/Bluetooth, USB‑Programmierinterface)

  * Kompakte IoT‑Plattform mit integriertem LiPo‑Lade‑Schaltkreis (falls verwendet).
* **Adafruit BNO055 9‑DOF Absolute Orientation IMU**

  * Integrierte Sensorfusion (IMU liefert direkte Orientierung als Euler/Quaternionen).
* **u‑blox NEO‑6M GPS (GY‑GPS6M) mit Antenne**

  * GPS/GLONASS‑Empfänger, TTL‑Schnittstelle, liefert NMEA oder UBX.

*Bevor Du beginnst: Achte auf die Betriebsspannungen. Alle genannten Module sind 3.3V‑kompatibel; der FireBeetle hat jedoch Vin/3.3V‑Pins — prüfe die Dokumentation.*

---

## Elektrischer Aufbau & Pinout

> Hinweis: Die hier gezeigten Anschlüsse sind typische Vorschläge. Prüfe die Pins auf deinem konkreten Board‑Silkscreen / Datenblatt.

### Vorschlag: I2C‑IMU (BNO055) + UART‑GPS (NEO‑6M) an FireBeetle

**BNO055 (I2C)**

* VIN (VCC) → 3V3 (FireBeetle 3.3V)
* GND → GND
* SDA → SDA (IO‑Pin für I2C; z. B. `GPIO21` auf vielen ESP32‑Boards)
* SCL → SCL (z. B. `GPIO22`)
* RST/PS0/PS1 → offen oder gemäß Adafruit‑Board

**NEO‑6M (UART TTL)**

* VCC → 3V3 (oder VIN wenn Modul 5V tolerant; prüfe Beschriftung)
* GND → GND
* TX (vom GPS) → RX des FireBeetle (z. B. `GPIO3` / `RX0` oder ein SoftSerial‑Pin)
* RX (vom GPS) → TX des FireBeetle (z. B. `GPIO1` / `TX0`)

**Stromversorgung / LiPo**

* Wenn Du eine LiPo‑Zelle verwendest, verbinde diese an den LiPo‑Eingang des FireBeetle (sofern vorhanden). Achte auf Ladestrom‑Limits.

**MicroSD (optional)**

* Falls Du SD‑Logging möchtest, verwende einen SPI‑MicroSD‑Adapter (MOSI, MISO, SCLK, CS) an freien SPI‑Pins.

**Mechanische Hinweise**

* Montiere die IMU so steif wie möglich am Gehäuse — IMU‑Messwerte sind empfindlich gegenüber Spiel.
* GPS‑Antenne sollte freie Sicht zum Himmel haben für beste Ergebnisse.

---

## Software & Libraries

Empfohlene Entwicklungsumgebung: **Arduino IDE** (oder PlatformIO).

**Wichtige Libraries**

* `Adafruit_BNO055` (Adafruit Unified Sensor + spezielle BNO055 Funktionen)
* `Adafruit_Sensor` (Unified Sensor Framework)
* `TinyGPSPlus` oder `TinyGPS++` (NMEA‑Parser für GPS)
* `HardwareSerial` / `SoftwareSerial` (UART‑Kommunikation falls benötigt)
* `WiFi.h` / `AsyncTCP` / `ESPAsyncWebServer` (für Telemetrie / Web‑UI)
* `SPI.h` + `SD.h` (falls microSD Logging)

**Beispiel: Bibliotheken via Library Manager**

1. Adafruit BNO055
2. Adafruit Unified Sensor
3. TinyGPSPlus
4. (optional) ESPAsyncWebServer + AsyncTCP (für Websocket / HTTP)

---

## Installation & Upload

1. Öffne Arduino IDE.
2. Boards → Board Manager → ESP32 installieren (falls noch nicht vorhanden).
3. Board auswählen: `DFRobot FireBeetle ESP32` (oder generisches `ESP32 Dev Module` mit passenden Einstellungen).
4. Bibliotheken installieren (siehe Abschnitt oben).
5. Beispielsketch hochladen: `examples/FreerideAssistant/main.ino` (im Repo).

**Voreinstellungen (Boardoptionen)**

* Flash Frequency: 80 MHz
* Upload Speed: 115200
* Flash Size: 4M (oder je nach Board)

---

## Konfiguration & Kalibrierung

### BNO055 Kalibrierung

Der BNO055 bietet eingebaute Kalibrierungszustände (IMU, Gyro, Accel, Magnetometer). Empfohlenes Verfahren:

1. Öffne das Serial‑Logging Beispiel (mit `Adafruit_BNO055`) und beobachte den Kalibrierungs‑Status.
2. Drehe das Gerät ruhig und stetig in alle Achsen, bis alle Werte vollständig kalibriert sind (Status = 3 für alle Subsensoren).
3. Optional: Speichere die Kalibrierungs‑Daten in EEPROM/Flash und lade sie nach Reset.

### GPS Hinweise

* GPS braucht freie Sicht zum Himmel. Indoor‑Tests sind möglich, jedoch langsam/ungenau.
* Warte 30–60 Sekunden beim ersten Fix (A‑GPS kann den Fix beschleunigen).

---

## Datenformat & Speicherung

Standard‑Datenframe (JSON, eine Zeile pro Messung):

```json
{
  "ts_utc": "2025-12-05T12:34:56Z",
  "lat": 47.123456,
  "lon": 11.123456,
  "alt": 123.4,
  "speed_m_s": 7.12,
  "orientation": {"qw": 0.98, "qx": 0.01, "qy": 0.05, "qz": 0.02},
  "euler": {"heading": 123.4, "roll": 1.2, "pitch": -3.4},
  "accel": {"ax": 0.12, "ay": -0.02, "az": 9.81}
}
```

* Speicherung: Protokolliere jede Messung als Zeitstempel + JSON‑Zeile (newline delimited). Das erleichtert späteres Parsing.
* Samplingraten: BNO055 kann orientierungsdaten bis 100 Hz liefern; GPS typ. 1 Hz–10 Hz (abhängig von Modul).

---

## Live‑Übertragung / Telemetrie

Optionen:

* **Websocket / HTTP**: ESP32 liefert Websocket‑Server; Smartphone/Notebook können Live‑Daten empfangen.
* **Bluetooth (BLE)**: Telemetrie mittels GATT, geeignet für kurze Reichweite.
* **MQTT**: ESP verbindet sich mit Broker (z. B. `mqtt://broker.example`) und veröffentlicht Sensordaten.

Beispiel: MQTT‑Topic `freeride/<device_id>/telemetry`.

---

## Beispielcode (Kern‑Loop)

> Im Repo findest Du vollständige Beispiele. Unten ein stark vereinfachtes Pseudocode‑Fragment:

```cpp
// setup: init BNO, init GPS UART, init WiFi, ggf. SD
// loop:
//  - read imu (quaternion, euler, accel)
//  - read gps (lat, lon, speed)
//  - build json
//  - write to SD
//  - publish via MQTT / websocket
```

---

## Fehlerbehebung

* **Keine IMU‑Daten:** Prüfe I2C‑Verkabelung, Adressen (`0x28` oder `0x29`), Pull‑ups.
* **GPS bekommt keinen Fix:** Antenne prüfen, Outdoor testen, Stromversorgung prüfen.
* **ESP32 stürzt ab / reboot loop:** Geringe Stromversorgung oder Boot‑GPIO Konflikte prüfen.

---

## Wartung & Tipps

* Regelmäßige Kalibrierung der IMU vor kritischen Messungen.
* Mechanische Dämpfung: Weiche Polsterung reduziert Vibrationseinflüsse auf Sensoren.
* Zeitstempel: Nutze GPS‑Zeit (UTC) wenn möglich, um Drift zu vermeiden.

---

## Weiterentwicklung & Ideen

* Fusion mit Barometer (Höhenstabilisierung)
* Echtzeit‑Event‑Erkennung (Sturz, Sprung) mit automatischer Markierung
* Mobile App für Visualisierung & Analyse

---

## Quellen & Bezugsquellen

* Adafruit BNO055 — smart 9‑DOF IMU (Sensorfusion)
* u‑blox NEO‑6M — GPS Modul (TTL)
* DFRobot FireBeetle ESP32‑E — ESP32‑basierte MCU

(Links zu Datenblättern und Bezugsquellen im Repo unter `/docs/links.md`)

---

## Lizenz

Dieses Projekt steht unter der MIT‑Lizenz. Siehe `LICENSE`.

---

## Mitwirkende

* Projektinitiator: mvalmci
* Contributors: Du? Pull Requests willkommen!

---

*Viel Erfolg beim Bauen — schreib mir gern, wenn Du eine spezifische Schaltung, ein PCB‑Layout oder die WebUI möchtest.*

