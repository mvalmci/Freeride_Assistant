import time
from machine import I2C, Pin, UART
from bno055 import BNO055
from micropyGPS import MicropyGPS
import math
import ujson


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
vx_avg_1s = 0.0
WINDOW_MS = 1000

# =====================================================
# Freefall / Airtime Parameter
# =====================================================
FREEFALL_ACC_THRESHOLD = 1.5   # m/s² - auf imu wirkt keine Erdbeschleunigung mehr
FREEFALL_TIME_MS = 150         # "Filter" bzw Puffer der kleine Stöße herausfiltert und erst eine Sprung als solchen erkennt wenn er über 0.15s dauert
LANDING_ACC_THRESHOLD = 15.0   # m/s² (Stoß)

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
takeoff_angle = None # Pitch des Euler Vektors

# =====================================================
# Timing
# =====================================================
last_print = time.ticks_ms()

print("System gestartet: GPS + IMU + Airtime")

#======================================================
# Abspeichern der Daten
#======================================================
def save_jump(data):
    try:
        # Datei öffnen oder erstellen
        with open("jumps.json", "a") as f:
            f.write(ujson.dumps(data) + "\n")
    except Exception as e:
        print("Fehler beim Speichern:", e)

    
# =====================================================
# Hauptloop
# =====================================================
while True:
    now = time.ticks_ms()

    # -------------------------------------------------
    # Testgeschwindigkeit (GPS simuliert)
    # -------------------------------------------------
    import random
    vx = 10.0 + random.uniform(-1, 1) # m/s

    # -------------------------------------------------
    # vx-Puffer (1 Sekunden Mittelwert)
    # -------------------------------------------------
    vx_buffer.append((now, vx))

    while vx_buffer and time.ticks_diff(now, vx_buffer[0][0]) > WINDOW_MS:
        vx_buffer.pop(0)

    if vx_buffer:
        vx_avg_1s = sum(v for _, v in vx_buffer) / len(vx_buffer)
    else:
        vx_avg_1s = 0.0

    # -------------------------------------------------
    # IMU lesen
    # -------------------------------------------------
    try:
        accel = imu.accel()  # m/s² (linear)
        euler = imu.euler()  # Orientierung (Heading, Roll, Pitch)
    except Exception:
        accel = (0.0, 0.0, 0.0)
        euler = None

    ax, ay, az = accel # Beschleunigungen in x, y, und z Richtung
    acc_mag = (ax*ax + ay*ay + az*az) ** 0.5 # Betrag aller drei Beschleunigungen, da z.B. Gravitation bei einer Drehung nicht konstant auf z-Achse wirkt sondern sich verteilt

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
            if time.ticks_diff(now, freefall_start) > FREEFALL_TIME_MS: # siehe oben... Puffer für kleine "hopser"
                state = AIR
                air_start_time = now
                
                # Absprungsgeschwindigkeit
                vx_absprung = vx_avg_1s # gemittelte Geschwindigkeit über 3s - wird in der Rechnung als v_0 (Anfangsgeschwindigkeit) verwendet
                
                # Absprungwinkel (Pitch)
                if euler is not None:
                    takeoff_angle = euler[2] #Pitch (also Absprungwiunkel) in Grad - wird in der Rechnung als Absprung(-wurf)winkel verwendet
                else:
                    takeoff_angle = None
                    
        else:
            state = GROUND

    elif state == AIR:
        airtime_ms = time.ticks_diff(now, air_start_time)
        if acc_mag > LANDING_ACC_THRESHOLD:
            state = LANDING
            
            #------------------------------------------
            # Sprunghöhenberechnung
            #------------------------------------------
            if vx_absprung is not None and takeoff_angle is not None:
                g = 9.81
                t_total = airtime_ms / 1000.0
                alpha_rad = takeoff_angle * 3.14159265 / 180.0
                
                # vertikale Anfangsgeschwindigkeit
                v0y = vx_absprung * math.sin(alpha_rad)
                
                # Aufstiegszeit
                t1 = v0y / g
                
                # Höhe bis Scheitelpunkt
                h1 = 0.5 * g * t1 * t1
                
                # Fallzeit
                t2 = t_total - t1
                if t2 < 0:
                    t2 = 0 # Sicherheitscheck
                
                # gesamte Fallhöhe
                hges = 0.5 * g * t2 * t2
                
                # tatsächliche Sprunghöhe
                h2 = hges - h1
                jump_height = h1 + h2
            
                #--------------------------------------
                # Sprungweitenberechnung
                #--------------------------------------
                #horizontale Werte
                x_ges = vx_absprung * math.cos(alpha_rad) * t_total
                
                # diagonale Weite mit Pythagoras
                c_ges = math.sqrt(x_ges * x_ges + jump_height * jump_height)
                
                jump_data = {
                    "airtime_s": round(t_total, 3),
                    "v0": round(vx_absprung, 2),
                    "angle_deg": round(takeoff_angle, 2),
                    "height_m": round(jump_height, 3),
                    "distance_x_m": round(x_ges, 3),
                    "distance_diag_m": round(c_ges, 3),
                    "timestamp_ms": now
                }

                save_jump(jump_data)

                
                print("===== SPRUNG ERKANNT =====")
                print("Airtime:", round(t_total, 3), "s")
                print("Absprunggeschwindigkeit:", round(vx_absprung, 2), "m/s")
                print("Absprungwinkel:", round(takeoff_angle, 2), "°")
                print("Sprunghöhe:", round(jump_height, 3), "m")
                print("Weite (horizontal):", round(x_ges, 3), "m")
                print("Weite (diagonal):", round(c_ges, 3), "m")
                print("==========================")

                
            else:
                jump_height = None
                x_ges = None
                c_ges = None

    # Zusätzlicher landing State um z.B Federn aus den Beinen und andere Störfaktoren herauszufiltern - damit bei einem "wippenden" aufkommen im Schnee nicht wild zwischen den States herumgesprungen wird
    elif state == LANDING:
        if acc_mag > 9.0:
            state = GROUND

    # -------------------------------------------------
    # Debug Ausgabe (1 Hz)
    # -------------------------------------------------
    if time.ticks_diff(now, last_print) > 1000:
        last_print = now
        print(
            "state:", state,
            "| fix:", gps.fix_stat,
            "| sats:", gps.satellites_in_use,
            "| vx:", round(vx, 2),
            "| vx_avg:", round(vx_avg_1s, 2),
            "| acc_mag:", round(acc_mag, 2),
            "| airtime(s):", round(airtime_ms / 1000, 2),
            "| vx_absprung:", vx_absprung,
            "| takeoff_angle:", takeoff_angle,
            "| pitch:", euler[2]
        )

    time.sleep_ms(10)

