#!/usr/bin/env python3
# viewer.py – resilient PC viewer for Pico-W BNO055 stream
# ---------------------------------------------------------------------------
import socket, json, time, sys
from vpython import box, vector, rate, scene
import paho.mqtt.client as mqtt

# ───────────── USER SETTINGS ────────────────────────────────────────────────
BROKER_IP  = None          # "192.168.0.123" to skip discovery
DISC_PORT  = 5005
MQTT_PORT  = 1883
TOPIC      = "imu/#"
KEEPALIVE  = 0
scale      = 50
sensor_dims = (0.025, 0.020, 0.003)    # L, H, W (m)
# ────────────────────────────────────────────────────────────────────────────

# ═══════════ 1. Broker discovery (optional) ═════════════════════════════════
def discover_broker(timeout=5):
    net = socket.gethostbyname(socket.gethostname()).rsplit('.', 1)[0]
    bcast_ip = f"{net}.255"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(1.0)
    end = time.time() + timeout
    while time.time() < end:
        sock.sendto(b"DISCOVER_MQTT_BROKER", (bcast_ip, DISC_PORT))
        try:
            data, _ = sock.recvfrom(64)
            return data.decode().strip()
        except socket.timeout:
            pass
    return None

if BROKER_IP is None:
    print("Searching for broker …")
    BROKER_IP = discover_broker()
    if not BROKER_IP:
        sys.exit("‼  Broker not found – start the responder or hard-code BROKER_IP.")
print("Using broker @", BROKER_IP)

# ═══════════ 2. VPython scene ═══════════════════════════════════════════════
scene.background = vector(0.6, 0.8, 1.0)            # ← sky-blue backdrop

my_box = box(length=sensor_dims[0]*scale,
             height=sensor_dims[1]*scale,
             width=sensor_dims[2]*scale,
             color=vector(0.25, 0.25, 0.25))        # ← dark-grey box

def quat_to_axis_up(q):
    w,x,y,z = q
    r00 = 1 - 2*(y*y + z*z)
    r10 = 2*(x*y + z*w)
    r20 = 2*(x*z - y*w)
    r01 = 2*(x*y - z*w)
    r11 = 1 - 2*(x*x + z*z)
    r21 = 2*(y*z + x*w)
    return vector(r00, r10, r20), vector(r01, r11, r21)

latest_quat, last_print = None, 0

############################################
# Camera (view) control via keyboard events
############################################
def reset_view():
    scene.forward = vector(-1, -1, -1).norm()
    scene.center  = vector(0, 0, 0)
    print("View reset.")

def keydown(evt):
    key = evt.key
    if   key == "left":  scene.forward = scene.forward.rotate( 0.1, vector(0,1,0))
    elif key == "right": scene.forward = scene.forward.rotate(-0.1, vector(0,1,0))
    elif key == "up":    scene.forward = scene.forward.rotate( 0.1, vector(1,0,0))
    elif key == "down":  scene.forward = scene.forward.rotate(-0.1, vector(1,0,0))
    elif key.lower() == "r": reset_view()

scene.bind("keydown", keydown)

# ═══════════ 3. MQTT client (robust) ════════════════════════════════════════
def on_connect(cli, _ud, _flags, rc, _=None):
    print("[MQTT] connected (rc =", rc, ")")
    cli.subscribe((TOPIC, 0))

def on_disconnect(cli, _ud, rc):
    print("[MQTT] disconnected (rc =", rc, ") – reconnecting …")

def on_message(cli, _ud, msg):
    global latest_quat, last_print
    try:
        data = json.loads(msg.payload)
        q = data["sensor_1"]["quat"]
        latest_quat = [q["w"], q["x"], q["y"], q["z"]]
        now = time.time()
        if now - last_print >= 1:         # console print throttled to 1 Hz
            print(f"[{msg.topic}] heading {data['sensor_1']['euler'][0]:.1f}°")
            last_print = now
    except (ValueError, KeyError):
        pass

def make_client():
    cli = mqtt.Client(client_id="pc_viewer_live", protocol=mqtt.MQTTv311)
    cli.on_connect    = on_connect
    cli.on_disconnect = on_disconnect
    cli.on_message    = on_message
    cli.max_inflight_messages_set(20)
    cli.max_queued_messages_set(0)
    return cli

client = make_client()
client.connect(BROKER_IP, MQTT_PORT, keepalive=KEEPALIVE)
client.loop_start()                     # background network thread

# ═══════════ 4. Render loop ════════════════════════════════════════════════
try:
    while True:
        rate(50)
        if latest_quat:
            axis, up = quat_to_axis_up(latest_quat)
            my_box.axis = axis
            my_box.up   = up
except KeyboardInterrupt:
    print("\nExiting viewer …")
finally:
    client.loop_stop()
    client.disconnect()
