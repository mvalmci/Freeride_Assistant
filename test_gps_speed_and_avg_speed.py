import time
from machine import UART
from micropyGPS import MicropyGPS

# --------------------------
# UART & GPS Setup
# --------------------------
# UART1, sichere Pins auf ESP32-E: TX=25, RX=26
uart = UART(1, baudrate=9600, tx=25, rx=26, timeout=10)

# GPS Parser, UTC+1 (Sommerzeit ggf. +2)
gps = MicropyGPS(local_offset=1)

# --------------------------
# Variablen
# --------------------------
vx = 0.0                  # aktuelle Geschwindigkeit [m/s]
vx_buffer = []            # Liste der letzten (timestamp_ms, vx)
vx_avg_3s = 0.0           # Mittelwert der letzten 3 Sekunden
WINDOW_MS = 3000          # Fensterdauer 3 Sekunden

absprung_trigger = False  # Platzhalter für spätere Absprungerkennung
vx_absprung = None        # gespeichert beim Trigger

last_print = time.ticks_ms()  # Timer für Ausgabe

# --------------------------
# Hauptloop
# --------------------------
while True:
    # --- GPS-Daten lesen ---
    data = uart.read()
    if data:
        for b in data:
            gps.update(chr(b))

        # Geschwindigkeit aktualisieren, wenn GPS-Fix vorhanden
        if gps.valid:
            vx = gps.speed[2]  # km/h von micropyGPS in m/s konvertieren optional: /3.6

    # --- vx-Puffer pflegen ---
    now = time.ticks_ms()
    vx_buffer.append((now, vx))

    # Alte Werte aus dem Puffer entfernen (>3 Sekunden)
    while vx_buffer and time.ticks_diff(now, vx_buffer[0][0]) > WINDOW_MS:
        vx_buffer.pop(0)

    # Mittelwert berechnen
    if vx_buffer:
        vx_avg_3s = sum(v for _, v in vx_buffer) / len(vx_buffer)
    else:
        vx_avg_3s = 0.0

    # --- Absprung-Trigger Platzhalter ---
    if absprung_trigger and vx_absprung is None:
        vx_absprung = vx_avg_3s
        print("Absprung gespeichert! vx_absprung =", vx_absprung)

    # --- Ausgabe (alle 1 Sekunde) ---
    if time.ticks_diff(now, last_print) > 1000:
        last_print = now
        print(
            "fix:", gps.fix_stat,
            "sats:", gps.satellites_in_use,
            "vx(m/s):", round(vx, 2),
            "vx_avg_3s:", round(vx_avg_3s, 2),
            "vx_absprung:", vx_absprung
        )

    # kleine Pause, um USB/REPL nicht zu blockieren
    time.sleep_ms(10)
