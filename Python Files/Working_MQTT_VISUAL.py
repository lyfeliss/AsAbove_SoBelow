# -*- coding: utf-8 -*-
"""
Created on Sun Apr 27 14:14:05 2025

@author: otorr
"""

"""
Live MQTT display for BNO055 quaternions coming from the Pico script.
-- Converts the original serial-based viewer to MQTT.
"""

import json, socket, time
from vpython import box, vector, rate, scene
import paho.mqtt.client as mqtt

# ───────────── USER SETTINGS ──────────────────────────────
BROKER_IP  = None        # None ⇒ discover by UDP broadcast, otherwise put "192.168.x.x"
DISC_PORT  = 5005        # must match Pico's UDP_DISC_PORT
MQTT_PORT  = 1883
TOPIC      = "imu/dataC" # must match Pico's cfg["TOPIC"]
# scaling & box geometry (same as before)
scale = 50
sensor_length, sensor_height, sensor_width = 0.025, 0.020, 0.003
# ──────────────────────────────────────────────────────────

# 1.  Find the broker if we don't already know it -----------------------------
def discover_broker(timeout_s=5) -> str | None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(1.0)
    end = time.time() + timeout_s
    while time.time() < end:
        sock.sendto(b"DISCOVER_MQTT_BROKER", ("255.255.255.255", DISC_PORT))
        try:
            data, _ = sock.recvfrom(64)
            return data.decode().strip()
        except socket.timeout:
            pass
    return None

if BROKER_IP is None:
    print("Searching for MQTT broker …")
    BROKER_IP = discover_broker()
    if not BROKER_IP:
        raise RuntimeError("Broker not found.  Start one and/or ensure it replies to discovery.")

print(f"Using broker @ {BROKER_IP}")

# 2.  VPython setup -----------------------------------------------------------
my_box = box(length=sensor_length*scale,
             height=sensor_height*scale,
             width=sensor_width*scale,
             color=vector(1,0,0))

def quaternion_to_axis_up(q):
    """Convert [w,x,y,z] into VPython (axis, up) vectors, flipping Z to match frames."""
    w,x,y,z = q
    r00 = 1 - 2*(y*y + z*z)
    r10 = 2*(x*y + z*w)
    r20 = 2*(x*z - y*w)
    r01 = 2*(x*y - z*w)
    r11 = 1 - 2*(x*x + z*z)
    r21 = 2*(y*z + x*w)
    return vector(r00, r10, -r20), vector(r01, r11, -r21)


############################################
# Camera (view) control via keyboard events
############################################
def reset_view():
    """Reset the camera view to a default orientation and center."""
    scene.forward = vector(-1, -1, -1).norm()  # Set a default forward direction.
    scene.center = vector(0, 0, 0)               # Center the scene at the origin.
    print("View reset.")

def keydown(evt):
    """Rotate the camera based on arrow keys or reset view on 'r'."""
    key = evt.key
    # Rotate left/right about the vertical (y) axis
    if key == "left":
        scene.forward = scene.forward.rotate(angle=0.1, axis=vector(0, 1, 0))
    elif key == "right":
        scene.forward = scene.forward.rotate(angle=-0.1, axis=vector(0, 1, 0))
    # Rotate up/down about the horizontal (x) axis
    elif key == "up":
        scene.forward = scene.forward.rotate(angle=0.1, axis=vector(1, 0, 0))
    elif key == "down":
        scene.forward = scene.forward.rotate(angle=-0.1, axis=vector(1, 0, 0))
    elif key.lower() == "r":
        reset_view()
    print("Key pressed:", key)

# Bind the keydown event to our keydown function.
scene.bind("keydown", keydown)

# 3.  MQTT callbacks ----------------------------------------------------------
latest_quat = None

def on_message(client, userdata, msg):
    global latest_quat
    try:
        payload = json.loads(msg.payload.decode())
        q = payload["sensor_1"]["quat"]
        # reorder to [w,x,y,z] list for convenience
        latest_quat = [q["w"], q["x"], q["y"], q["z"]]
    except (ValueError, KeyError):
        print("Bad payload:", msg.payload[:80])

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER_IP, MQTT_PORT, keepalive=0)
client.subscribe(TOPIC)
client.loop_start()           # run network loop in background

# 4.  Render loop -------------------------------------------------------------
while True:
    rate(20)                   # 50 FPS cap
    if latest_quat:
        axis, up = quaternion_to_axis_up(latest_quat)
        my_box.axis = axis
        my_box.up   = up
