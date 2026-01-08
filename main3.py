import time
from machine import I2C, Pin, UART
from bno055 import BNO055
from micropyGPS import MicropyGPS

# =====================================================
# IMU – BNO055 (I2C)
# =====================================================
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
imu = BNO055(i2c)

# =====================================================
# GPS – UART1
# =====================================================
uart = UART(1, baudrate=9600, tx=25, rx=26, timeout=10)
gps = MicropyGPS(local_offset=1)

# =====================================================
# GPS Geschwindigkeit (vx)
# =====================================================
vx = 0.0
vx_buffer = []
vx_avg_3s = 0.0
WINDOW_MS = 3000

# =====================================================
# Freefall / Airtime Parameter
# =====================================================
FREEFALL_ACC_THRESHOLD = 1.5   # m/s²
FREEFALL_TIME_MS = 150         # ms
LANDING_ACC_THRESHOLD = 15.0   # m/s²

# =====================================================
# State Machine
# =====================================================
GROUND = 0
TAKEOFF = 1
AIR = 2
LANDING = 3

state = GROUND

# =====================================================
# Jump Daten
# =====================================================
freefall_start = None
air_start_time = None
airtime_ms = 0
vx_absprung = None
takeoff_angle = None

# =====================================================
# Timing
# =====================================================
last_print = time.ticks_ms()

print("System gestartet: GPS + IMU + Airtime")

# =====================================================
# Hauptloop
# =====================================================
while True:
    now = time.ticks_ms()

    # -------------------------------------------------
    # GPS lesen
    # -------------------------------------------------
    data = uart.read()
    if data:
        for b in data:
            gps.update(chr(b))
        if gps.valid:
            vx = gps.speed[2]  # m/s

    # -------------------------------------------------
    # vx-Puffer (3 Sekunden Mittelwert)
    # -------------------------------------------------
    vx_buffer.append((now, vx))

    while vx_buffer and time.ticks_diff(now, vx_buffer[0][0]) > WINDOW_MS:
        vx_buffer.pop(0)

    if vx_buffer:
        vx_avg_3s = sum(v for _, v in vx_buffer) / len(vx_buffer)
    else:
        vx_avg_3s = 0.0

    # -------------------------------------------------
    # IMU lesen
    # -------------------------------------------------
    try:
        accel = imu.accel()  # m/s² (linear)
        euler = imu.euler()  # Orientierung (Heading, Roll, Pitch)
    except Exception:
        accel = (0.0, 0.0, 0.0)
        euler = None

    ax, ay, az = accel
    acc_mag = (ax*ax + ay*ay + az*az) ** 0.5

    # -------------------------------------------------
    # State Machine + Airtime
    # -------------------------------------------------
    if state == GROUND:
        airtime_ms = 0
        vx_absprung = None
        if acc_mag < FREEFALL_ACC_THRESHOLD:
            freefall_start = now
            state = TAKEOFF

    elif state == TAKEOFF:
        if acc_mag < FREEFALL_ACC_THRESHOLD:
            if time.ticks_diff(now, freefall_start) > FREEFALL_TIME_MS:
                state = AIR
                air_start_time = now
                
                # Absprungsgeschwindigkeit
                vx_absprung = vx_avg_3s
                
                # Absprungwinkel (Pitch)
                if euler is not None:
                    takeoff_angle = euler[2] #Pitch (also Absprungwiunkel) in Grad
                else:
                    takeoff_angle = None
                    
        else:
            state = GROUND

    elif state == AIR:
        airtime_ms = time.ticks_diff(now, air_start_time)
        if acc_mag > LANDING_ACC_THRESHOLD:
            state = LANDING

    elif state == LANDING:
        if acc_mag > 9.0:
            state = GROUND

    # -------------------------------------------------
    # Ausgabe (1 Hz)
    # -------------------------------------------------
    if time.ticks_diff(now, last_print) > 1000:
        last_print = now
        print(
            "state:", state,
            "| fix:", gps.fix_stat,
            "| sats:", gps.satellites_in_use,
            "| vx:", round(vx, 2),
            "| vx_avg:", round(vx_avg_3s, 2),
            "| acc_mag:", round(acc_mag, 2),
            "| airtime(s):", round(airtime_ms / 1000, 2),
            "| vx_absprung:", vx_absprung,
            "| takeoff_angle:", takeoff_angle,
            "| pitch:", euler[2]
        )

    time.sleep_ms(10)
