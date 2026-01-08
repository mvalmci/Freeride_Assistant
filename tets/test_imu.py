from machine import I2C, Pin
from bno055 import BNO055
import time

# I2C-Pins für ESP32 (Standard)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))

# Sensor initialisieren
sensor = BNO055(i2c)

print("BNO055 Test gestartet...")

while True:
    try:
        # Orientierung (Heading, Roll, Pitch)
        euler = sensor.euler()

        # Beschleunigung (m/s²)
        accel = sensor.accel()

        # Gyro (°/s)
        gyro = sensor.gyro()

        # Temperatur
        temp = sensor.temperature()

        print("Euler:", euler)
        print("Accel:", accel)
        print("Gyro :", gyro)
        print("Temp :", temp, "°C")
        print("-------------------------")

    except Exception as e:
        print("Fehler:", e)

    time.sleep(0.1)
