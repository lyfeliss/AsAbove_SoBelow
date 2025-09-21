import time
import json
import machine
import BNO055

# Delay between sensor readings (in seconds)
delay = 0.1

def load_config():
    try:
        with open("config.json", "r") as f:
            return json.load(f)
    except Exception as e:
        print("Error loading config.json:", e)
        return None

config = load_config()
if not config:
    # Default configuration if config.json is missing/invalid
    config = {
        "pico_ID": "C",
        "I2C_SDA": 14,
        "I2C_SCL": 15,
        "I2C_FREQ": 400000
    }

pico_ID  = config["pico_ID"]
I2C_SDA  = config["I2C_SDA"]
I2C_SCL  = config["I2C_SCL"]
I2C_FREQ = config["I2C_FREQ"]

# Set up I2C and initialize both BNO055 sensors
i2c = machine.I2C(1, sda=machine.Pin(I2C_SDA), scl=machine.Pin(I2C_SCL), freq=I2C_FREQ)
bno1 = BNO055.BNO055(i2c, address=0x28)  # Sensor 1 (ADR pin LOW)
bno2 = BNO055.BNO055(i2c, address=0x29)  # Sensor 2 (ADR pin HIGH)

while True:
    # Read quaternion data from both sensors (format: [w, x, y, z])
    quat1 = bno1.read_quat()
    quat2 = bno2.read_quat()

    # Package data as JSON; you can choose to send one or both sensors’ data.
    data = {
        "pico_ID": pico_ID,
        "sensor_1": {"quat": quat1},
        "sensor_2": {"quat": quat2}
    }
    print(json.dumps(data))
    time.sleep(delay)
