import time
from machine import UART
from micropyGPS import MicropyGPS
import bluetooth

# --- GPS Setup ---
uart = UART(1, baudrate=9600, tx=25, rx=26, timeout=10)
gps = MicropyGPS(local_offset=1)

# --- BLE Setup ---
ble = bluetooth.BLE()
ble.active(True)

SERVICE_UUID = bluetooth.UUID("12345678-1234-5678-1234-56789abcdef0")
CHAR_SPEED_UUID = bluetooth.UUID("12345678-1234-5678-1234-56789abcdef1")
CHAR_GPS_UUID   = bluetooth.UUID("12345678-1234-5678-1234-56789abcdef2")

speed_char = (CHAR_SPEED_UUID, bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY)
gps_char   = (CHAR_GPS_UUID, bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY)

service = (SERVICE_UUID, (speed_char, gps_char))
services = (service,)

((handle_speed, handle_gps),) = ble.gatts_register_services(services)

# --- Werbung manuell erzeugen ---
def advertise(name="ESP32-GPS"):
    payload = bytearray()

    # Flags (0x01)
    payload += bytes([2, 0x01, 0x06])

    # Name (0x09)
    name_bytes = name.encode()
    payload += bytes([len(name_bytes) + 1, 0x09]) + name_bytes

    ble.gap_advertise(100_000, payload)

advertise()

print("BLE aktiv – bereit für Verbindung")

vx = 0.0
last_update = time.ticks_ms()

while True:
    data = uart.read()
    if data:
        for b in data:
            gps.update(chr(b))

        if gps.valid:
            vx = gps.speed[2] / 3.6

    if time.ticks_diff(time.ticks_ms(), last_update) > 1000:
        last_update = time.ticks_ms()

        lat = gps.latitude_string()
        lon = gps.longitude_string()
        gps_str = f"{lat},{lon}"

        ble.gatts_write(handle_speed, str(vx))
        ble.gatts_notify(0, handle_speed)

        ble.gatts_write(handle_gps, gps_str)
        ble.gatts_notify(0, handle_gps)

        print("BLE sendet:", vx, gps_str)

    time.sleep_ms(10)
