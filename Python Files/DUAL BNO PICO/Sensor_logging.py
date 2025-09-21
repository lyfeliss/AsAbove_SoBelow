import time
import json
import os
import machine
import BNO055

# Delay between sensor readings (in seconds)
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
        "I2C_SDA": 14,
        "I2C_SCL": 15,
        "I2C_FREQ": 400000
    }

pico_ID  = config["pico_ID"]
I2C_SDA  = config["I2C_SDA"]
I2C_SCL  = config["I2C_SCL"]
I2C_FREQ = config["I2C_FREQ"]

# Set up I2C & BNO055 sensors
i2c = machine.I2C(1, sda=machine.Pin(I2C_SDA), scl=machine.Pin(I2C_SCL), freq=I2C_FREQ)
bno1 = BNO055.BNO055(i2c, address=0x28)  # ADR pin LOW
bno2 = BNO055.BNO055(i2c, address=0x29)  # ADR pin HIGH

while True:
    # Read data from sensor 1 (BNO1)
    euler1 = bno1.read_euler()
    gyro1  = bno1.read_gyro()
    accel1 = bno1.read_accel()
    quat1  = bno1.read_quat()  # (w, x, y, z)

    # Read data from sensor 2 (BNO2)
    euler2 = bno2.read_euler()
    gyro2  = bno2.read_gyro()
    accel2 = bno2.read_accel()
    quat2  = bno2.read_quat()

    # Package the sensor data as JSON
    imu_data = {
        "Sensor": pico_ID,
        "sensor_1": {
            "euler": {"heading": euler1[0], "roll": euler1[1], "pitch": euler1[2]},
            "gyro":  {"x": gyro1[0], "y": gyro1[1], "z": gyro1[2]},
            "accel": {"x": accel1[0], "y": accel1[1], "z": accel1[2]},
            "quat":  {"w": quat1[0], "x": quat1[1], "y": quat1[2], "z": quat1[3]}
        },
        "sensor_2": {
            "euler": {"heading": euler2[0], "roll": euler2[1], "pitch": euler2[2]},
            "gyro":  {"x": gyro2[0], "y": gyro2[1], "z": gyro2[2]},
            "accel": {"x": accel2[0], "y": accel2[1], "z": accel2[2]},
            "quat":  {"w": quat2[0], "x": quat2[1], "y": quat2[2], "z": quat2[3]}
        }
    }

    # Log the sensor data (prints the JSON string)
    print("Logged Data:", json.dumps(imu_data))
    time.sleep(delay)
