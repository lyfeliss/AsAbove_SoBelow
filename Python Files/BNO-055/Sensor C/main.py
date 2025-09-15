import network
import time
import json
import os
from umqtt.simple import MQTTClient
import machine
import BNO055
import socket

# Delay between main loop iterations
delay = 0.1

def load_config():
    try:
        with open("config.json", "r") as f:
            return json.load(f)
    except (OSError, ValueError) as e:
        print("Error loading config.json:", e)
        return None

config = load_config()
if not config:
    print("Failed to load config. Using defaults.")
    config = {
        "Type": "BNO",
        "pico_ID": "C",
        "SSID": "YOUR_SSID",
        "PASSWORD": "YOUR_PASS",
        "MQTT_BROKER": "foundationpi",
        "TOPIC": "imu/dataC",
        "I2C_SDA": 14,
        "I2C_SCL": 15,
        "I2C_FREQ": 400000
    }

pico_ID     = config["pico_ID"]
SSID        = config["SSID"]
PASSWORD    = config["PASSWORD"]
# We'll ignore the broker in the config, since we want indefinite discovery:
TOPIC       = config["TOPIC"]
I2C_SDA     = config["I2C_SDA"]
I2C_SCL     = config["I2C_SCL"]
I2C_FREQ    = config["I2C_FREQ"]

# 1) Connect to Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)
print("Connecting to Wi-Fi...", end="")
while not wlan.isconnected():
    print(".", end="")
    time.sleep(1)
print("\nConnected to Wi-Fi! IP:", wlan.ifconfig()[0])

# 2) Indefinitely discover MQTT broker IP over UDP
discovered_broker_ip = None
UDP_BROADCAST_IP = "255.255.255.255"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.settimeout(5)  # We'll catch timeouts & retry

print("Waiting indefinitely for MQTT broker discovery response...")
while True:
    try:
        discovery_message = "DISCOVER_MQTT_BROKER"
        sock.sendto(discovery_message.encode(), (UDP_BROADCAST_IP, UDP_PORT))
        data, addr = sock.recvfrom(1024)
        discovered_broker_ip = data.decode()
        print("Received MQTT broker IP:", discovered_broker_ip)
        break
    except OSError:
        print("No response, retrying broadcast...")
        time.sleep(3)

sock.close()

# 3) Set up I2C & BNO055
i2c = machine.I2C(1, sda=machine.Pin(I2C_SDA), scl=machine.Pin(I2C_SCL), freq=I2C_FREQ)
bno1 = BNO055.BNO055(i2c, address=0x28)  # ADR pin LOW
bno2 = BNO055.BNO055(i2c, address=0x29)  # ADR pin HIGH

# 4) Prepare MQTT client
MQTT_BROKER = discovered_broker_ip
client = MQTTClient(pico_ID, MQTT_BROKER)
mqtt_connected = False

# NOTE:
# If you were using paho-mqtt (e.g., on PC or Raspberry Pi), you would instantiate the client like this:
#   client = mqtt.Client(client_id="csv_subscriber", callback_api_version=1)
# However, since this code uses MicroPython’s umqtt.simple, the above line is correct.

def mqtt_connect():
    global mqtt_connected
    try:
        print("Attempting MQTT connect to:", MQTT_BROKER)
        client.connect()
        mqtt_connected = True
        print("MQTT connected!")
    except OSError as e:
        print("MQTT connection failed:", e)
        mqtt_connected = False

mqtt_connect()

# 5) Optionally: Prepare UDP for sending quaternions to PC
pc_ip = "192.168.0.100"  # <--- change to PC/host IP
pc_port = 5000
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    # Reconnect MQTT if needed
    if not mqtt_connected:
        mqtt_connect()

    # --- Read data from BNO1 ---
    euler1 = bno1.read_euler()
    gyro1  = bno1.read_gyro()
    accel1 = bno1.read_accel()
    quat1  = bno1.read_quat()  # (w, x, y, z)

    # --- Read data from BNO2 ---
    euler2 = bno2.read_euler()
    gyro2  = bno2.read_gyro()
    accel2 = bno2.read_accel()
    quat2  = bno2.read_quat()

    # Package as JSON
    imu_data = {
        "Sensor": pico_ID,
        "sensor_1": {
            "euler": {"heading": euler1[0], "roll": euler1[1], "pitch": euler1[2]},
            "gyro":  {"x": gyro1[0],  "y": gyro1[1],  "z": gyro1[2]},
            "accel": {"x": accel1[0], "y": accel1[1], "z": accel1[2]},
            "quat":  {"w": quat1[0], "x": quat1[1], "y": quat1[2], "z": quat1[3]}
        },
        "sensor_2": {
            "euler": {"heading": euler2[0], "roll": euler2[1], "pitch": euler2[2]},
            "gyro":  {"x": gyro2[0],  "y": gyro2[1],  "z": gyro2[2]},
            "accel": {"x": accel2[0], "y": accel2[1], "z": accel2[2]},
            "quat":  {"w": quat2[0], "x": quat2[1], "y": quat2[2], "z": quat2[3]}
        }
    }

    # Publish over MQTT if connected
    if mqtt_connected:
        try:
            client.publish(TOPIC, json.dumps(imu_data))
        except OSError as e:
            print("Lost MQTT connection:", e)
            mqtt_connected = False

    # Optionally send quaternions over UDP for PC visualization
    w1, x1, y1, z1 = quat1
    w2, x2, y2, z2 = quat2
    udp_msg = f"{pico_ID},{w1},{x1},{y1},{z1},{w2},{x2},{y2},{z2}"
    udp_socket.sendto(udp_msg.encode(), (pc_ip, pc_port))

    print("Sent =>", udp_msg)
    time.sleep(delay)
