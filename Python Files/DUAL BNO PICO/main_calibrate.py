import time
import json
import machine
import BNO055
import sys

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

def wait_for_start_command():
    """
    Blocks until it reads a line from the PC that matches 'START'.
    This ensures the Pico won't proceed until the PC explicitly 
    tells it to start (i.e., the PC is connected & ready).
    """
    print("Pico is ready. Send 'START' from PC to begin calibration checks.")
    while True:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            if line.upper() == "START":
                print("Received START command. Proceeding with calibration loop...")
                break
        time.sleep(0.1)

def check_calibrations():
    """
    Prints calibration status every second until 
    *both* BNO055 sensors are fully calibrated.
    """
    print("Move each sensor in all orientations until fully calibrated.")
    while True:
        cal_s1 = bno1.get_calibration_status()  # (sys, gyro, accel, mag)
        cal_s2 = bno2.get_calibration_status()
        
        sys1, gyr1, acc1, mag1 = cal_s1
        sys2, gyr2, acc2, mag2 = cal_s2

        # Print status
        print(f"S1 => SYS:{sys1} GYR:{gyr1} ACC:{acc1} MAG:{mag1} | "
              f"S2 => SYS:{sys2} GYR:{gyr2} ACC:{acc2} MAG:{mag2}")

        # If both fully calibrated, break
        if bno1.is_fully_calibrated() and bno2.is_fully_calibrated():
            print("Both sensors fully calibrated!")
            break
        
        time.sleep(1.0)

def main_loop():
    """
    Continuously outputs sensor data (including magnetometer) in JSON format.
    """
    print("Starting to output sensor data...")
    while True:
        # --- Read data from BNO1 ---
        euler1 = bno1.read_euler()  # (heading, roll, pitch)
        gyro1  = bno1.read_gyro()   # (x, y, z)
        accel1 = bno1.read_accel()  # (x, y, z)
        quat1  = bno1.read_quat()   # (w, x, y, z)
        mag1   = bno1.read_mag()    # (x, y, z) in µT (assuming read_mag() is defined)

        # --- Read data from BNO2 ---
        euler2 = bno2.read_euler()
        gyro2  = bno2.read_gyro()
        accel2 = bno2.read_accel()
        quat2  = bno2.read_quat()
        mag2   = bno2.read_mag()

        # Package data as JSON 
        imu_data = {
            "sensor_1": {
                "euler": {"heading": euler1[0], "roll": euler1[1], "pitch": euler1[2]},
                "gyro":  {"x": gyro1[0],  "y": gyro1[1],  "z": gyro1[2]},
                "accel": {"x": accel1[0], "y": accel1[1], "z": accel1[2]},
                "quat":  {"w": quat1[0], "x": quat1[1], "y": quat1[2], "z": quat1[3]},
                "mag":   {"x": mag1[0],  "y": mag1[1],   "z": mag1[2]}
            },
            "sensor_2": {
                "euler": {"heading": euler2[0], "roll": euler2[1], "pitch": euler2[2]},
                "gyro":  {"x": gyro2[0],  "y": gyro2[1],  "z": gyro2[2]},
                "accel": {"x": accel2[0], "y": accel2[1], "z": accel2[2]},
                "quat":  {"w": quat2[0], "x": quat2[1], "y": quat2[2], "z": quat2[3]},
                "mag":   {"x": mag2[0],  "y": mag2[1],   "z": mag2[2]}
            }
        }

        # Print JSON line over serial
        print(json.dumps(imu_data))
        
        # Adjust as needed to control your data rate
        time.sleep(0.05)

# ---------------------------------------------------
# MAIN ENTRY POINT
# ---------------------------------------------------
if __name__ == "__main__":
    import select

    # Wait for PC to send 'START' 
    wait_for_start_command()

    # Then do calibration checks
    check_calibrations()

    # Then proceed to main data output loop
    main_loop()
