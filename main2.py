# CircuitPython auf ESP32
import time
import board
import busio
from micropyGPS import MicropyGPS
import adafruit_bno055

# ---- IMU Setup (I2C) ----
i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno055.BNO055_I2C(i2c)
# Falls externer Quarz vorhanden:
# bno.use_external_crystal = True

# ---- GPS Setup (UART) ----
# Pins ggf. anpassen: TX=board.IO17, RX=board.IO16
uart = busio.UART(board.IO17, board.IO16, baudrate=9600, timeout=0.1)
gps = MicropyGPS(local_offset=1)  # CET/CEST

# ---- Zustandsvariablen ----
in_air = False
t_takeoff_ms = 0
t_landing_ms = 0

# Geschwindigkeitsfusion (horizontale Achse ~ Welt-X entlang Hang)
vx_est = 0.0           # m/s (geschätzt)
vx_gps = 0.0           # m/s (GPS)
vx_bias = 0.0          # Bias für IMU-Integration
alpha_cf = 0.1         # Complementary-Filter-Anteil GPS
dt_target = 0.01

# Buffer für die letzten 1 s vor Absprung (IMU-acc_x)
acc_x_buffer = []
time_buffer = []

# Hangneigung (Baseline) vor dem Run mitteln
baseline_pitch_deg = None
baseline_collect = True
baseline_pitch_samples = []

# Takeoff-Pitch-Erfassung: kurzes Fenster direkt nach Absprung
takeoff_pitch_deg = None
takeoff_pitch_samples = []
takeoff_window_ms = 200  # 150–250 ms ist gut
takeoff_window_open = False

def read_gps_update():
    global vx_gps
    data = uart.read()
    if data:
        try:
            chars = data.decode('utf-8', errors='ignore')
        except:
            chars = ''
        for ch in chars:
            gps.update(ch)
        # Ground speed (km/h) -> m/s
        if gps.speed:
            vx_gps = gps.speed[2] / 3.6

def world_acc_from_sensor():
    lin = bno.linear_acceleration
    quat = bno.quaternion
    if not lin or not quat:
        return None
    ax, ay, az = lin
    w, x, y, z = quat
    # Rotationsmatrix aus Quaternion
    r00 = 1 - 2*(y*y + z*z); r01 = 2*(x*y - w*z); r02 = 2*(x*z + w*y)
    r10 = 2*(x*y + w*z);     r11 = 1 - 2*(x*x + z*z); r12 = 2*(y*z - w*x)
    r20 = 2*(x*z - w*y);     r21 = 2*(y*z + w*x);     r22 = 1 - 2*(x*x + y*y)
    awx = r00*ax + r01*ay + r02*az
    awy = r10*ax + r11*ay + r12*az
    awz = r20*ax + r21*ay + r22*az
    return (awx, awy, awz)

def update_speed_fusion(dt):
    global vx_est, vx_bias
    a_world = world_acc_from_sensor()
    if not a_world:
        return
    awx, _, _ = a_world

    # Ringbuffer ~1 s
    acc_x_buffer.append(awx)
    time_buffer.append(time.monotonic())
    while time_buffer and (time.monotonic() - time_buffer[0]) > 1.0:
        acc_x_buffer.pop(0)
        time_buffer.pop(0)

    # Prediction (Bias-Korrektur im Stand optional setzen)
    vx_est = vx_est + (awx - vx_bias)*dt
    # Update mit GPS (Low-Freq)
    vx_est = (1 - alpha_cf)*vx_est + alpha_cf*vx_gps

def update_baseline_pitch():
    # Vor dem Run (ruhig stehen) sammeln und mitteln
    global baseline_pitch_deg, baseline_collect
    eul = bno.euler
    if not eul:
        return
    heading, roll, pitch = eul
    if baseline_collect:
        baseline_pitch_samples.append(pitch)
        # nach ~1.5 s mitteln
        if len(baseline_pitch_samples) >= 150:  # bei ~100 Hz Euler
            baseline_pitch_deg = sum(baseline_pitch_samples)/len(baseline_pitch_samples)
            baseline_collect = False
            print(f"Baseline-Hangneigung α = {baseline_pitch_deg:.2f}°")

def detect_takeoff_and_landing():
    global in_air, t_takeoff_ms, t_landing_ms
    global takeoff_pitch_deg, takeoff_pitch_samples, takeoff_window_open

    lin = bno.linear_acceleration
    grav = bno.gravity
    eul = bno.euler
    if not lin or not grav or not eul:
        return

    ax, ay, az = lin
    gz = grav[2] if grav else None
    heading, roll, pitch = eul

    # Schwellen kalibrieren!
    peak_up = 2.0
    peak_down = -2.0
    g_eps = 0.6

    # Absprung
    if not in_air and az > peak_up:
        t_takeoff_ms = time.ticks_ms()
        in_air = True
        takeoff_pitch_samples = []
        takeoff_window_open = True
        print(f"Absprung: raw Pitch={pitch:.2f}°, Roll={roll:.2f}°, Heading={heading:.1f}°")
        print(f"v_x (fusion) = {vx_est:.2f} m/s ; v_x (GPS) = {vx_gps:.2f} m/s")

    # Während des Takeoff-Fensters Pitch sammeln
    if in_air and takeoff_window_open:
        if time.ticks_diff(time.ticks_ms(), t_takeoff_ms) <= takeoff_window_ms:
            takeoff_pitch_samples.append(pitch)
        else:
            # Fenster schließen und Mittelwert bilden
            takeoff_window_open = False
            if takeoff_pitch_samples:
                takeoff_pitch_deg = sum(takeoff_pitch_samples)/len(takeoff_pitch_samples)
                if baseline_pitch_deg is not None:
                    absprungwinkel = takeoff_pitch_deg - baseline_pitch_deg
                    print(f"Takeoff-Pitch (gemittelt) = {takeoff_pitch_deg:.2f}° ; Absprungwinkel θ = {absprungwinkel:.2f}° (relativ zur Hangneigung)")
                else:
                    print(f"Takeoff-Pitch (gemittelt) = {takeoff_pitch_deg:.2f}° ; (Baseline α fehlt)")

    # Landung
    if in_air and az < peak_down and gz is not None and abs(gz + 9.81) < g_eps:
        t_landing_ms = time.ticks_ms()
        in_air = False
        t_flight = (t_landing_ms - t_takeoff_ms) / 1000.0
        distance = vx_est * t_flight
        print(f"Landung: Flugzeit={t_flight:.3f}s, Sprungweite≈{distance:.2f} m")

def main_loop():
    last = time.monotonic()
    print("System läuft: Absprungwinkel (Pitch), v_x Fusion, Flugzeit, Sprungweite")
    while True:
        read_gps_update()
        now = time.monotonic()
        dt = now - last
        last = now
        if dt <= 0 or dt > 0.05:
            dt = dt_target

        # Vor dem Run Baseline-Pitch erfassen (einmalig)
        update_baseline_pitch()

        update_speed_fusion(dt)
        detect_takeoff_and_landing()

        time.sleep(0.01)

# Start
main_loop()
