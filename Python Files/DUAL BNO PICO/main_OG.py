import time
import json
import machine
import BNO055

# Adjust I2C pins & frequency as needed for your hardware setup
I2C_SDA  = 14
I2C_SCL  = 15
I2C_FREQ = 400000

# Initialize I2C
i2c = machine.I2C(1, sda=machine.Pin(I2C_SDA), scl=machine.Pin(I2C_SCL), freq=I2C_FREQ)

# Initialize two BNO055 IMUs 
#   address=0x28 -> ADR pin LOW, 
#   address=0x29 -> ADR pin HIGH
bno1 = BNO055.BNO055(i2c, address=0x28)
bno2 = BNO055.BNO055(i2c, address=0x29)

# Continuously output sensor data over serial in JSON format.
# The PC side decides how often to 'consume' or record a line of this data.
while True:
    # --- Read data from BNO1 ---
    euler1 = bno1.read_euler()  # (heading, roll, pitch)
    gyro1  = bno1.read_gyro()   # (x, y, z)
    accel1 = bno1.read_accel()  # (x, y, z)
    quat1  = bno1.read_quat()   # (w, x, y, z)
    
    # --- Read data from BNO2 ---
    euler2 = bno2.read_euler()
    gyro2  = bno2.read_gyro()
    accel2 = bno2.read_accel()
    quat2  = bno2.read_quat()
    
    # Package data as JSON 
    # (You can rename or add fields as needed.)
    imu_data = {
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
    
    # Print JSON line over serial
    # The PC script can read it via pyserial
    print(json.dumps(imu_data))
    
    # Tiny sleep just to avoid flooding the serial port too fast. 
    # PC will also do its own timing.
    time.sleep(0.01)
