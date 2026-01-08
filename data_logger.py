# Muss als main.py auf dem Esp32 gespeichert werden
# Kombinierter GPS + BNO055 Logger -> schreibt NDJSON in data_log.jsonl
import time
import ujson as json
import os

from machine import UART, I2C, Pin
from micropyGPS import MicropyGPS
from bno055 import BNO055

# --------------------------
# Einstellungen
# --------------------------
UART_ID = 1
UART_BAUD = 9600
UART_TX = 25
UART_RX = 26
I2C_ID = 0
I2C_SCL = 22
I2C_SDA = 21

FILENAME = "data_log.jsonl"
SAMPLE_INTERVAL_MS = 100   # Abtastintervall (ms) -> 10 Hz
WINDOW_MS = 3000           # 3s Fenster für Durchschnittsgeschwindigkeit
MAX_FILE_BYTES = 1024 * 1024  # einfache Rotation bei >1MB

# --------------------------
# Hardware Setup
# --------------------------
uart = UART(UART_ID, baudrate=UART_BAUD, tx=UART_TX, rx=UART_RX, timeout=10)
gps = MicropyGPS(local_offset=1)

i2c = I2C(I2C_ID, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
imu = BNO055(i2c)

# --------------------------
# Hilfsfunktionen
# --------------------------
def to_json_safe(x):
    # rekursiv Tupel -> Liste, sonst primitive oder str()
    if isinstance(x, (tuple, list)):
        return [to_json_safe(i) for i in x]
    try:
        # ujson kann manche Typen nicht serialisieren; int/float/str/dict/list ok
        json.dumps(x)
        return x
    except Exception:
        return str(x)

def collect_gps_data(gps):
    d = {}
    try:
        d["valid"] = bool(gps.valid)
        d["fix_stat"] = getattr(gps, "fix_stat", None)
        d["satellites_in_use"] = getattr(gps, "satellites_in_use", None)
        # speed is usually a tuple in micropyGPS; safe konvertierung
        speed = getattr(gps, "speed", None)
        d["speed"] = to_json_safe(speed)
        # latitude/longitude/altitude falls vorhanden
        d["latitude"] = to_json_safe(getattr(gps, "latitude", None))
        d["longitude"] = to_json_safe(getattr(gps, "longitude", None))
        d["altitude"] = to_json_safe(getattr(gps, "altitude", None))
    except Exception as e:
        d["error"] = "gps_read_error: " + str(e)
    return d

def collect_imu_data(imu):
    d = {}
    try:
        d["euler"] = to_json_safe(imu.euler())
        d["accel"] = to_json_safe(imu.accel())
        d["gyro"] = to_json_safe(imu.gyro())
        d["temperature"] = to_json_safe(imu.temperature())
    except Exception as e:
        d["error"] = "imu_read_error: " + str(e)
    return d

def append_record(rec):
    try:
        # Anhängen als einzelne JSON-Zeile (NDJSON)
        with open(FILENAME, "a") as f:
            f.write(json.dumps(rec) + "\n")
    except Exception as e:
        print("Write error:", e)

    # einfache Rotation: wenn Datei zu groß -> umbenennen
    try:
        st = os.stat(FILENAME)
        size = st[6]  # size index in stat tuple
        if size > MAX_FILE_BYTES:
            # falls schon eine alte Datei existiert, überschreiben
            try:
                os.remove(FILENAME + ".old")
            except Exception:
                pass
            try:
                os.rename(FILENAME, FILENAME + ".old")
            except Exception as e:
                print("Rotate error:", e)
    except Exception:
        pass

# --------------------------
# Logger-Variablen
# --------------------------
vx = 0.0
vx_buffer = []   # Liste von (timestamp_ms, vx)
vx_avg_3s = 0.0

last_sample = time.ticks_ms()
last_print = time.ticks_ms()

# --------------------------
# Hauptloop
# --------------------------
print("Starter: GPS + BNO055 Logger")
while True:
    now = time.ticks_ms()

    # --- GPS-Rohdaten lesen und Parser füttern ---
    data = uart.read()
    if data:
        # uart.read() liefert bytes; micropyGPS erwartet Zeichen
        for b in data:
            try:
                gps.update(chr(b))
            except Exception:
                # chr kann fehlschlagen, fallback
                try:
                    gps.update(str(b))
                except Exception:
                    pass

        # Wenn GPS gültig, update vx
        try:
            if gps.valid:
                # gps.speed ist meist ein tuple: (knots, m/s?, km/h?) - in ursprünglichem code wurde [2] genutzt
                sp = getattr(gps, "speed", None)
                if sp and isinstance(sp, (list, tuple)) and len(sp) > 2:
                    # sp[2] war km/h in originalcode -> in m/s umrechnen
                    try:
                        vx = float(sp[2]) / 3.6
                    except Exception:
                        vx = float(sp[2])
                else:
                    # fallback: falls speed direkt float/ int
                    try:
                        vx = float(sp)
                    except Exception:
                        pass
        except Exception:
            pass

    # --- vx-Puffer pflegen (für 3s-Mittel) ---
    vx_buffer.append((now, vx))
    # alte Werte entfernen
    while vx_buffer and time.ticks_diff(now, vx_buffer[0][0]) > WINDOW_MS:
        vx_buffer.pop(0)
    if vx_buffer:
        s = 0.0
        for _, v in vx_buffer:
            s += v
        vx_avg_3s = s / len(vx_buffer)
    else:
        vx_avg_3s = 0.0

    # --- Messung & Logging im Sample-Intervall ---
    if time.ticks_diff(now, last_sample) >= SAMPLE_INTERVAL_MS:
        last_sample = now

        rec = {
            "ts_ms": now,
            # falls RTC gesetzt ist, time.time() liefert epoch sekunden, sonst None
            "ts_unix": None,
            "gps": collect_gps_data(gps),
            "imu": collect_imu_data(imu),
            "vx_m_s": vx,
            "vx_avg_3s_m_s": vx_avg_3s
        }
        # versuche unix-time zu setzen (wenn RTC vorhanden)
        try:
            rec["ts_unix"] = time.time()
        except Exception:
            rec["ts_unix"] = None

        append_record(rec)

    # --- Debug-Ausgabe einmal pro Sekunde ---
    if time.ticks_diff(now, last_print) > 1000:
        last_print = now
        try:
            print("fix:", getattr(gps, "fix_stat", None),
                  "sats:", getattr(gps, "satellites_in_use", None),
                  "vx(m/s):", round(vx, 2),
                  "vx_avg_3s:", round(vx_avg_3s, 2),
                  "file:", FILENAME)
        except Exception:
            print("Status print error")

    # kleine Pause, damit REPL/USB nicht blockiert
    time.sleep_ms(10)
