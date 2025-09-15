import serial
import time
import json
import pandas as pd

#######################################################
#                CONFIG PARAMETERS
#######################################################
COM_PORT              = "COM9"  # e.g. "COM3" on Windows, "/dev/ttyUSB0" on Linux
BAUD_RATE             = 115200
TIME_BETWEEN_CAPTURES = 1.0     # seconds between each data capture
TOTAL_CAPTURE_TIME    = 10.0    # total duration (seconds) to record data
FILE_LOCATION         = r"C:\Users\otorr\Documents"  # where to save Excel

#######################################################
#          OPEN SERIAL & SEND "START"
#######################################################
print("Opening serial port...")
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)

# Give it a moment to establish
time.sleep(1.0)

# Send "START" command so the Pico begins calibration sequence
ser.write(b"START\n")
print("Sent 'START' to Pico. Waiting for calibration to complete...")

#######################################################
#            WAIT FOR FULL CALIBRATION
#######################################################
has_calibrated = False
while not has_calibrated:
    line = ser.readline().decode().strip()
    if not line:
        continue
    # Print everything we get, so you see the calibration progress
    print(line)
    
    # Once we see "Both sensors fully calibrated!", we proceed
    if "Both sensors fully calibrated!" in line:
        has_calibrated = True

#######################################################
#   WAIT FOR USER INPUT BEFORE STARTING TO LOG DATA
#######################################################
print("Calibration complete!")
input("Press Enter to begin logging data...")  # <-- Waits for user keypress
print("Now collecting sensor data...")

#######################################################
#          PREPARE TO COLLECT DATA
#######################################################
# Data columns (including magnetometer)
sensor1_cols = [
    "timestamp",
    "heading", "roll", "pitch",
    "gyro_x", "gyro_y", "gyro_z",
    "accel_x", "accel_y", "accel_z",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "mag_x", "mag_y", "mag_z"
]
sensor2_cols = sensor1_cols[:]  # same structure for sensor_2

sensor1_data = []
sensor2_data = []

start_time = time.time()

#######################################################
#          MAIN DATA CAPTURE LOOP
#######################################################
while (time.time() - start_time) < TOTAL_CAPTURE_TIME:
    line = ser.readline().decode().strip()
    if not line:
        # If we got nothing, just wait the capture interval
        time.sleep(TIME_BETWEEN_CAPTURES)
        continue
    
    try:
        # Attempt to parse JSON
        data = json.loads(line)
        
        # Extract sensor_1
        e1 = data["sensor_1"]["euler"]
        g1 = data["sensor_1"]["gyro"]
        a1 = data["sensor_1"]["accel"]
        q1 = data["sensor_1"]["quat"]
        m1 = data["sensor_1"]["mag"]

        # Extract sensor_2
        e2 = data["sensor_2"]["euler"]
        g2 = data["sensor_2"]["gyro"]
        a2 = data["sensor_2"]["accel"]
        q2 = data["sensor_2"]["quat"]
        m2 = data["sensor_2"]["mag"]

        now = time.time()

        # Build rows for each sensor
        sensor1_row = [
            now,
            e1["heading"], e1["roll"], e1["pitch"],
            g1["x"], g1["y"], g1["z"],
            a1["x"], a1["y"], a1["z"],
            q1["w"], q1["x"], q1["y"], q1["z"],
            m1["x"], m1["y"], m1["z"]
        ]
        sensor2_row = [
            now,
            e2["heading"], e2["roll"], e2["pitch"],
            g2["x"], g2["y"], g2["z"],
            a2["x"], a2["y"], a2["z"],
            q2["w"], q2["x"], q2["y"], q2["z"],
            m2["x"], m2["y"], m2["z"]
        ]

        sensor1_data.append(sensor1_row)
        sensor2_data.append(sensor2_row)

    except json.JSONDecodeError:
        # Not JSON -> likely a status message
        print("[Non-JSON line] ", line)

    # Wait the user-defined interval
    time.sleep(TIME_BETWEEN_CAPTURES)

#######################################################
#          SAVE TO EXCEL
#######################################################
print("Data capture complete. Saving to Excel...")

df_sensor1 = pd.DataFrame(sensor1_data, columns=sensor1_cols)
df_sensor2 = pd.DataFrame(sensor2_data, columns=sensor2_cols)

with pd.ExcelWriter(FILE_LOCATION, engine='openpyxl') as writer:
    df_sensor1.to_excel(writer, sheet_name='Sensor_1', index=False)
    df_sensor2.to_excel(writer, sheet_name='Sensor_2', index=False)

print(f"Data saved to {FILE_LOCATION}")
ser.close()
