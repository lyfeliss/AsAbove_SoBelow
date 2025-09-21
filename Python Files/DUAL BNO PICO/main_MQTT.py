# ---------------------------------------------------------------------------
# COMBINED BNO055 DATA LOGGER: USB‑Serial + MQTT  (+ optional UDP quaternion)
# ---------------------------------------------------------------------------
#  • Reads two BNO055s over I2C
#  • Prints JSON to USB serial
#  • Publishes same JSON to an MQTT topic (broker discovered via UDP)
#  • Optionally blasts raw quaternions to a PC over UDP
# ---------------------------------------------------------------------------

import time, json, machine, network, socket
from umqtt.simple import MQTTClient
import BNO055

# ================== CONFIGURATION ===========================================
DEFAULTS = {
    # Identity
    "pico_ID": "C",
    # Wi‑Fi
    "SSID": "YOUR_SSID",
    "PASSWORD": "YOUR_PASS",
    # MQTT
    "TOPIC": "imu/dataC",
    "UDP_DISC_PORT": 5005,           # port where broker responds to discovery
    # I²C
    "I2C_SDA": 14,
    "I2C_SCL": 15,
    "I2C_FREQ": 400000,              # Hz
    # Serial print rate
    "SERIAL_HZ": 100,                # 100 Hz → one JSON line every 0.01 s
    # UDP quaternion stream (set pc_ip to None to disable)
    "pc_ip": "192.168.0.100",
    "pc_port": 5000
}

# ---------- load/merge config.json if present ------------------------------
try:
    with open("config.json") as f:
        cfg = DEFAULTS.copy()
        cfg.update(json.load(f))
except (OSError, ValueError):
    cfg = DEFAULTS.copy()

def hz_to_dt(freq):
    return 0 if freq <= 0 else 1.0 / freq

serial_dt = hz_to_dt(cfg["SERIAL_HZ"])

# ================== NETWORK SET‑UP ==========================================
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("Connecting to Wi‑Fi '{}' ...".format(cfg["SSID"]), end="")
wlan.connect(cfg["SSID"], cfg["PASSWORD"])
while not wlan.isconnected():
    print(".", end="")
    time.sleep(1)
print("\nWi‑Fi up.  IP:", wlan.ifconfig()[0])

# ---------- discover MQTT broker by UDP broadcast --------------------------
UDP_BCAST_IP  = "255.255.255.255"
UDP_DISC_PORT = cfg["UDP_DISC_PORT"]
broker_ip     = None

disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
disc_sock.settimeout(5)

print("Waiting for MQTT broker broadcast ...")
while True:
    try:
        disc_sock.sendto(b"DISCOVER_MQTT_BROKER", (UDP_BCAST_IP, UDP_DISC_PORT))
        data, _ = disc_sock.recvfrom(64)
        broker_ip = data.decode().strip()
        print("Broker found @", broker_ip)
        break
    except OSError:
        print("  ...retrying")
disc_sock.close()

# ================== I²C & BNO055s ===========================================
i2c = machine.I2C(1,
    sda=machine.Pin(cfg["I2C_SDA"]),
    scl=machine.Pin(cfg["I2C_SCL"]),
    freq=cfg["I2C_FREQ"]
)

bno1 = BNO055.BNO055(i2c, address=0x28)   # ADR pin LOW
bno2 = BNO055.BNO055(i2c, address=0x29)   # ADR pin HIGH

# ================== MQTT CLIENT ============================================
client = MQTTClient(cfg["pico_ID"], broker_ip)

def mqtt_connect():
    try:
        client.connect()
        print("MQTT connected.")
        return True
    except OSError as e:
        print("MQTT connect failed:", e)
        return False

mqtt_ok = mqtt_connect()

# ================== OPTIONAL UDP SOCKET ====================================
udp_sock = None
if cfg["pc_ip"]:
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ================== MAIN LOOP ==============================================
last_serial_ms = 0
while True:
    # ---- read both sensors -------------------------------------------------
    e1, g1, a1, q1 = bno1.read_euler(), bno1.read_gyro(), bno1.read_accel(), bno1.read_quat()
    e2, g2, a2, q2 = bno2.read_euler(), bno2.read_gyro(), bno2.read_accel(), bno2.read_quat()

    payload = {
        "Sensor": cfg["pico_ID"],
        "sensor_1": {
            "euler": {"heading": e1[0], "roll": e1[1], "pitch": e1[2]},
            "gyro" : {"x": g1[0], "y": g1[1], "z": g1[2]},
            "accel": {"x": a1[0], "y": a1[1], "z": a1[2]},
            "quat" : {"w": q1[0], "x": q1[1], "y": q1[2], "z": q1[3]}
        },
        "sensor_2": {
            "euler": {"heading": e2[0], "roll": e2[1], "pitch": e2[2]},
            "gyro" : {"x": g2[0], "y": g2[1], "z": g2[2]},
            "accel": {"x": a2[0], "y": a2[1], "z": a2[2]},
            "quat" : {"w": q2[0], "x": q2[1], "y": q2[2], "z": q2[3]}
}}