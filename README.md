# Freeride Assistant

**Freeride Assistant** ist ein Open‑Source Projekt zur Datenerfassung und Unterstützung beim Freeriden. Ziel ist es, valide Sensordaten (Lage, Beschleunigung, GPS‑Position, Tempo) in Echtzeit zu erfassen, zu verarbeiten und zu protokollieren. Dieses Repository enthält Code, Schaltpläne und Beispielkonfigurationen für den Einsatz mit dem DFRobot FireBeetle ESP32, dem Adafruit BNO055 IMU und dem u‑blox NEO‑6M GPS Modul.

---

## Inhaltsverzeichnis

1. [Kurzüberblick](#kurzüberblick)

2. [Verwendete Hardware](#verwendete-hardware)

3. [Elektrischer Aufbau & Pinout](#elektrischer-aufbau--pinout)

4. [Software & Libraries](#software--libraries)

5. [Installation & Upload

6. FireBeetle ESP32-E mit MicroPython flashen (z. B. mittels `esptool.py`).

7. Projektdateien aus dem Repo (`main.py`, `/lib/bno055.py`, `/lib/micropyGPS.py`) auf das Board kopieren.

8. Mit Thonny oder mpremote Konsole öffnen und prüfen, ob alle Module geladen werden.

**Hinweis:** Die MicroPython-Version sollte **ESP32-kompatibel, stable** und möglichst aktuell sein.2 installieren (falls noch nicht vorhanden).
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
#### Datasheet: [Datasheet](-ADA2472.pdf)
Der BNO055 bietet eingebaute Kalibrierungszustände (IMU, Gyro, Accel, Magnetometer). Empfohlenes Verfahren:

1. Öffne das Serial‑Logging Beispiel (mit `Adafruit_BNO055`) und beobachte den Kalibrierungs‑Status.
2. Drehe das Gerät ruhig und stetig in alle Achsen, bis alle Werte vollständig kalibriert sind (Status = 3 für alle Subsensoren).
3. Optional: Speichere die Kalibrierungs‑Daten in EEPROM/Flash und lade sie nach Reset.

### GPS Hinweise
#### Datasheet: [Datasheet](-GY-GPS6MV2.pdf)
#### [NAchlesewerk GPS Modul](https://randomnerdtutorials.com/micropython-esp32-neo-6m-gps/)
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

## Beispielcode (MicroPython)

````python
from machine import I2C, UART, Pin
import ujson, utime
from bno055 import BNO055
from gps import GPS

i2c = I2C(1, scl=Pin(22), sda=Pin(21))
imu = BNO055(iEmpfohlene Entwicklungsumgebung: **MicroPython** (z. B. Thonny, VS Code + Pymakr, mpremote).S(**Wichtige MicroPython-Module**

- `machine` (I2C, UART, PWM)
- `ujson` (JSON-Serialisierung)
- `utime` (Zeit)
- `uos` (Dateien)
- `bno055.py` – MicroPython-Treiber (liegt im Repo unter `/lib`)
- `gps.py` – einfacher NMEA-Parser
- Optional: `umqtt.simple` für MQTT
- Optional: MicroPython-Webserver (z. B. einfache Socket-Implementierung)der `0x29`), Pull‑ups.
- **GPS bekommt keinen Fix:** Antenne prüfen, Outdoor testen, Stromversorgung prüfen.
- **ESP32 stürzt ab / reboot loop:** Geringe Stromversorgung oder Boot‑GPI## Installation & Upload

1. FireBeetle ESP32-E mit **MicroPython-Firmware** flashen (z. B. mittels `esptool.py`).
2. Projektdateien (`main.py`, `/lib/bno055.py`, `/lib/gps.py`) auf das Board kopieren.
3. Mit Thonny oder mpremote verbinden.
4. `main.py` ausführen oder beim Boot automatisch starten lassen.

---afruit BNO055 — smart 9‑DOF IMU (Sensorfusion)
- u‑blox NEO‑6M — GPS Modul (TTL)
- DFRobot FireBeetle ESP32‑E — ESP32‑basierte MCU

(Links zu Datenblättern und Bezugsquellen im Repo unter `/docs/links.md`)

---

## Lizenz

Dieses Projekt steht unter der MIT‑Lizenz. Siehe `LICENSE`.

---

## Mitwirkende

- Projektinitiator: mvalmci
- Contributors: Du? Pull Requests willkommen!


---

*Viel Erfolg beim Bauen — schreib mir gern, wenn Du eine spezifische Schaltung, ein PCB‑Layout oder die WebUI möchtest.*

## Beispielcode (MicroPython)

```python
from machine import I2C, UART, Pin
import ujson, utime
from bno055 import BNO055
from gps import GPS

i2c = I2C(1, scl=Pin(22), sda=Pin(21))
imu = BNO055(i2c)

uart = UART(1, baudrate=9600, tx=Pin(17), rx=Pin(16))
gps = GPS(uart)

while True:
    imu_data = imu.euler()
    accel = imu.accel()
    gps.update()

    data = {
        "ts": utime.time(),
        "lat": gps.lat,
        "lon": gps.lon,
        "speed": gps.speed,
        "euler": imu_data,
        "accel": accel
    }

    print(ujson.dumps(data))
````
# Welche Daten werden benutzt?
## Data-Fusion
➡️ IMU-Daten als kontinuierliche, hochfrequente Geschwindigkeitsschätzung verwenden
➡️ und diese Schätzung regelmäßig mit den GPS-Daten korrigieren, sobald ein gültiger Fix vorliegt.
Das ist das Standardprinzip von „Dead Reckoning + GPS Correction“.
Damit du ein Gefühl bekommst, hier die klare Struktur, wie du es in deinem Projekt aufbauen solltest:
#### ✅ 1. IMU als Basis („Prediction“)
Die IMU liefert:
Orientierung (Quaternion oder Euler)
Lineare Beschleunigung (idealerweise schon gravity-free, z. B. beim BNO055 verfügbar)
Aus der IMU machst du:
Beschleunigung → Geschwindigkeit integrieren
So bekommst du eine vorhergesagte Geschwindigkeit, die sehr reaktionsschnell ist.
Problem: Diese Integration driftet immer → daher brauchst du GPS.
#### ✅ 2. GPS als Korrektur („Correction“)
Das GPS liefert:
Geschwindigkeit (über NMEA-Satz VTG oder aus RMC)
Richtung
Absolutwerte ohne drift (aber langsam und manchmal noisy)
Du nutzt GPS nur, wenn:
Fix vorhanden (fix = True)
Satelliten ≥ 5
HDOP akzeptabel (z. B. < 2.0)
Dann korrigierst du die IMU-Geschwindigkeit regelmäßig.
#### ✅ 3. Fusion: IMU + GPS (einfacher Ansatz)
Die Grundformel:
v_est = v_est + a_imu * dt      # IMU-Vorhersage

if gps_ok:
    v_est = (1 - k) * v_est + k * v_gps   # GPS-Korrektur
Mit:
k = 0.05–0.2
kleiner k → IMU dominiert
großer k → GPS dominiert
#### 🟦 FESTE EMPFEHLUNG für dein Projekt
👉 Ja: Rechne die Geschwindigkeit aus den IMU-Dateien / IMU-Daten selbst aus.
👉 Ja: Korrigiere diese IMU–Geschwindigkeit dann kontinuierlich mit GPS-Daten.
Das ist der beste Kompromiss:
IMU = schön glatt + schnell
GPS = absolut + driftkorrektur
Du bekommst damit:
sofortige Reaktion
keinen Drift
stabilen Wert auch bei schlechten GPS-Signalen
