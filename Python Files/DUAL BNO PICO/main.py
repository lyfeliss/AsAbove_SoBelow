# ---------------------------------------------------------------------------
# COMBINED BNO055 DATA LOGGER v2.2-robust  (Pico W)
# ---------------------------------------------------------------------------
import time, ujson as json, machine, network, socket, struct, gc
from umqtt.simple import MQTTClient
import BNO055

# ================== USER CONFIGURATION =====================================
DEFAULTS = {
    "pico_ID": "C",
    # Wi-Fi
    "SSID":    "YOUR_SSID",
    "PASSWORD":"YOUR_PASS",
    # MQTT
    "TOPIC":   "imu/dataC",
    "UDP_DISC_PORT": 5005,
    # I²C
    "I2C_SDA": 14, "I2C_SCL": 15, "I2C_FREQ": 400_000,
    # USB-serial print rate (Hz). 0 = disable
    "SERIAL_HZ": 100,
    # Raw-quaternion UDP stream (None → disable)
    "pc_ip":   "192.168.0.100",
    "pc_port": 5000
}
try:
    with open("config.json") as f:
        cfg = DEFAULTS.copy()
        cfg.update(json.load(f))
except (OSError, ValueError):
    cfg = DEFAULTS.copy()

SERIAL_PERIOD = None if cfg["SERIAL_HZ"] <= 0 else 1.0 / cfg["SERIAL_HZ"]

# ================== Wi-Fi ===================================================
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print(f"Connecting to Wi-Fi '{cfg['SSID']}' …", end="")
wlan.connect(cfg["SSID"], cfg["PASSWORD"])
while not wlan.isconnected():
    print(".", end="")
    time.sleep(1)
print("\nWi-Fi up.  IP:", wlan.ifconfig()[0])



SERIAL_DT = None if cfg["SERIAL_HZ"] <= 0 else 1 / cfg["SERIAL_HZ"]
last_serial = time.ticks_ms()

# ================== Wi-Fi (original dot-loop) ===============================
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print(f"Connecting to Wi-Fi '{cfg['SSID']}' …", end="")
wlan.connect(cfg["SSID"], cfg["PASSWORD"])
while not wlan.isconnected():
    print(".", end="")
    time.sleep(1)
print("\nWi-Fi up.  IP:", wlan.ifconfig()[0])

# ================== Discover MQTT broker (UDP) =============================
DISC_PORT = cfg["UDP_DISC_PORT"]

def discover_broker():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(3)
    while True:
        try:
            sock.sendto(b"DISCOVER_MQTT_BROKER",
                        ("255.255.255.255", DISC_PORT))
            data, _ = sock.recvfrom(64)
            sock.close()
            return data.decode().strip()
        except OSError:
            print("  …retrying")

print("Looking for broker …")
BROKER_IP = discover_broker()
print("Broker found @", BROKER_IP)

# ================== I²C & sensors ==========================================
i2c = machine.I2C(1,
    sda=machine.Pin(cfg["I2C_SDA"]),
    scl=machine.Pin(cfg["I2C_SCL"]),
    freq=cfg["I2C_FREQ"])
bno1 = BNO055.BNO055(i2c, 0x28)
bno2 = BNO055.BNO055(i2c, 0x29)

# ================== MQTT helpers ===========================================
def new_client():
    c = MQTTClient(cfg["pico_ID"], BROKER_IP, keepalive=30)
    c.connect()
    print("MQTT connected.")
    return c

client = new_client()

# ================== Optional UDP socket ====================================
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if cfg["pc_ip"] else None
gc_counter = 0

# ================== MAIN LOOP ==============================================
while True:
    try:
        # ---- read both sensors -------------------------------------------
        e1,g1,a1,q1 = bno1.read_euler(), bno1.read_gyro(), \
                      bno1.read_accel(), bno1.read_quat()
        e2,g2,a2,q2 = bno2.read_euler(), bno2.read_gyro(), \
                      bno2.read_accel(), bno2.read_quat()

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
            }
        }

        # ---- publish and service MQTT ------------------------------------
        client.publish(cfg["TOPIC"], json.dumps(payload))
        client.check_msg()            # handles keep-alive & incoming packets

    except (OSError, ValueError, MemoryError) as err:
        print("Warn:", err, "→ reconnecting …")
        try:
            client.disconnect()
        except Exception:
            pass
        time.sleep(1)
        BROKER_IP = discover_broker()
        client   = new_client()
        continue                      # restart main loop immediately

    # ---- optional USB-serial print ---------------------------------------
    now = time.ticks_ms()
    if SERIAL_DT and time.ticks_diff(now, last_serial) >= SERIAL_DT*1000:
        try:
            print(json.dumps(payload))
        except Exception:
            pass
        last_serial = now

    # ---- optional raw quaternion UDP blast -------------------------------
    if udp_sock:
        try:
            udp_sock.sendto(struct.pack("<4f", *q1),
                            (cfg["pc_ip"], cfg["pc_port"]))
        except OSError:
            pass                       # ignore Wi-Fi TX hiccup

    # ---- heap housekeeping ----------------------------------------------
    gc_counter += 1
    if gc_counter >= 500:              # ~5 s at 100 Hz
        gc.collect(); gc_counter = 0
