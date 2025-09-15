import serial
import json
from vpython import box, vector, rate, scene

# Adjust the serial port and baud rate as needed.
SERIAL_PORT = "COM8"  # e.g., "COM3" on Windows
BAUD_RATE = 115200

# Scaling factor to amplify the sensor dimensions for visualization.
scale = 50  # Adjust as needed

# Actual sensor dimensions (in meters)
sensor_length = 0.025  # 25 mm along x-axis
sensor_height = 0.020  # 20 mm along y-axis
sensor_width  = 0.003  # 3 mm along z-axis

# Create a 3D box representing the sensor.
my_box = box(length=sensor_length * scale, 
             height=sensor_height * scale, 
             width= sensor_width  * scale, 
             color=vector(1, 0, 0))

# Initialize serial connection to the Pico.
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def quaternion_to_axis_up(q):
    """
    Convert a quaternion q = [w, x, y, z] into two VPython vectors:
    - axis: The rotated x-axis (from the first column of the rotation matrix)
    - up:   The rotated y-axis (from the second column of the rotation matrix)
    
    We invert the x and y components to correct for the mounting orientation,
    so that a positive tilt in the sensor produces the expected rotation.
    """
    w, x, y, z = q
    
    # Rotated x-axis: first column of the rotation matrix
    r00 = 1 - 2*(y*y + z*z)
    r10 = 2*(x*y + z*w)
    r20 = 2*(x*z - y*w)
    # Invert x and y for VPython
    axis = vector(-r00, -r10, r20)
    
    # Rotated y-axis: second column of the rotation matrix
    r01 = 2*(x*y - z*w)
    r11 = 1 - 2*(x*x + z*z)
    r21 = 2*(y*z + x*w)
    # Invert x and y for VPython
    up = vector(-r01, -r11, r21)
    
    return axis, up

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

############################################
# Main loop: update the sensor visualization.
############################################
while True:
    rate(50)  # Limit update rate to about 50 frames per second.
    line = ser.readline().decode().strip()
    if line:
        try:
            data = json.loads(line)
            # Use sensor_1's quaternion for the visualization.
            q = data["sensor_1"]["quat"]
            axis, up = quaternion_to_axis_up(q)
            my_box.axis = axis
            my_box.up = up
        except Exception as e:
            print("Error parsing line:", line, e)
