import time
import board
import busio
import adafruit_bno055
from machine import UART
from micropyGPS import MicropyGPS

# --- Setup IMU (BNO055) ---
i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno055.BNO055_I2C(i2c)

# --- Setup GPS (NEO-6M) ---
uart = UART(1, baudrate=9600, tx=17, rx=16)  # Pins anpassen!
gps = MicropyGPS(local_offset=1)  # UTC+1 für Mitteleuropa

# --- Variablen ---
in_air = False
takeoff_time = 0
landing_time = 0
vx = 0.0

def read_gps():
    global vx
    if uart.any():
        sentence = uart.read().decode('utf-8', errors='ignore')
        for char in sentence:
            gps.update(char)
        if gps.valid:
            # Geschwindigkeit in km/h → m/s
            vx = gps.speed[2] / 3.6
    return vx

def detect_flight():
    global in_air, takeoff_time, landing_time, vx

    lin_acc = bno.linear_acceleration
    gravity = bno.gravity

    if lin_acc is None or gravity is None:
        return None

    # --- Absprung ---
    if not in_air and lin_acc[2] > 2.0:  # Peak nach oben
        takeoff_time = time.ticks_ms()
        in_air = True
        vx = read_gps()
        print("Absprung erkannt, vx =", vx)

    # --- Landung ---
    if in_air and lin_acc[2] < -2.0 and abs(gravity[2] + 9.81) < 0.5:
        landing_time = time.ticks_ms()
        in_air = False
        t_flight = (landing_time - takeoff_time) / 1000.0
        distance = vx * t_flight
        print("Landung erkannt!")
        print("Flugzeit:", t_flight, "s")
        print("Sprungweite:", distance, "m")

# --- Hauptloop ---
while True:
    read_gps()
    detect_flight()
    time.sleep(0.05)  # 20 Hz Loop

