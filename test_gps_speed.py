import time
from machine import UART
from micropyGPS import MicropyGPS

# UART1 auf ESP32 ist ok; Pins 25/26 sind grundsätzlich unkritisch
uart = UART(1, baudrate=9600, tx=25, rx=26, timeout=10)

gps = MicropyGPS(local_offset=1)  # UTC+1 (Achtung: Sommerzeit = +2)

vx = 0.0
last_print = time.ticks_ms()

while True:
    data = uart.read()  # liest alles verfügbare (oder None)
    if data:
        for b in data:
            sentence_type = gps.update(chr(b))
            # optional: nur bei komplett geparstem Satz etwas tun

        # Geschwindigkeit aktualisieren, wenn Fix gültig
        if gps.valid:
            # gps.speed[2] in [m/s]
            vx = gps.speed[2] 

    # Ausgabe drosseln (sonst "busy" / Serial spam)
    if time.ticks_diff(time.ticks_ms(), last_print) > 1000:
        last_print = time.ticks_ms()
        print("fix:", gps.fix_stat,
              "sats:", gps.satellites_in_use,
              "vx(m/s):", vx,
              "lat:", gps.latitude,
              "lon:", gps.longitude)

    time.sleep_ms(10)  # wichtig, damit REPL/USB nicht verhungert
