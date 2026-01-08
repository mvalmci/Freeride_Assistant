# Ski Jump Tracker (ESP32 + BNO055 + GPS)

Dieses Projekt ist ein autonomer Tracker für Skispringer, der auf einem ESP32 basiert. Er misst Flugzeit (Airtime), Sprungweite, maximale Höhe und die Falltiefe mithilfe eines BNO055 IMU-Sensors und eines GPS-Moduls.

## 🚀 Funktionen
- **Automatische Sprungerkennung:** Nutzt die Beschleunigungsdaten der IMU, um Schwerelosigkeit (Takeoff) und Aufprall (Landing) zu erkennen.
- **Präzise Physik-Berechnung:** Berechnet unter Einbeziehung von Absprungwinkel (Pitch) und GPS-Geschwindigkeit die reale Flugbahn (parabolischer Wurf).
- **Daten-Logging:** Speichert Sprungdaten (Weite, Höhe, Zeit) direkt im Flash-Speicher des ESP32 als `.csv` oder `.jsonl`.
- **Stand-alone Betrieb:** Startet automatisch beim Anschluss einer Powerbank (via `main.py`).

## 🛠 Hardware-Komponenten
- **Mikrocontroller:** ESP32
- **IMU:** Bosch BNO055 (9-Achsen-Sensor) über I2C
- **GPS:** u-blox (oder kompatibel) über UART
- **Stromversorgung:** Mobile Powerbank

### Verkabelung
| Komponente | Pin ESP32 | Funktion |
| :--- | :--- | :--- |
| BNO055 | Pin 21 / 22 | SDA / SCL (I2C) |
| GPS | Pin 25 / 26 | TX / RX (UART) |

## 📊 Funktionsweise der State Machine
Das Programm durchläuft vier Zustände, um Fehlmessungen zu vermeiden:
1. **GROUND:** Sensor misst Erdbeschleunigung (~9.81 m/s²).
2. **TAKEOFF:** Beschleunigung sinkt unter 1.5 m/s² (Filterung für 150ms).
3. **AIR:** Aufzeichnung der Flugzeit und des Absprungwinkels.
4. **LANDING:** Impact-Erkennung (> 15.0 m/s²) und Berechnung der Ergebnisse.

## 📝 Berechnete Werte
Das System nutzt folgende physikalische Formeln:
- **Horizontale Weite:** $s = v_x \cdot t$
- **Höhendifferenz:** $\Delta h = v_{y0} \cdot t - 0.5 \cdot g \cdot t^2$
- **Max. Höhe (Scheitelpunkt):** Berechnung über $t_{peak} = v_{y0} / g$
- **Echte Weite:** Diagonale Distanz via Pythagoras $\sqrt{s_{horiz}^2 + \Delta h^2}$

## 📁 Daten auslesen
Die Daten werden im internen Flash-Speicher abgelegt.
1. ESP32 an PC anschließen.
2. In **Thonny** das Dateifenster öffnen.
3. Rechtsklick auf `log.csv` (oder `data_log.jsonl`) -> *Download*.

## 🛠 Installation
1. MicroPython auf den ESP32 flashen.
2. Bibliotheken `bno055.py` und `micropyGPS.py` auf den ESP32 hochladen.
3. Den Code als `main.py` speichern, damit er ohne PC startet.

---
*Entwickelt für die Messung von Skisprüngen direkt am Skischuh.*
