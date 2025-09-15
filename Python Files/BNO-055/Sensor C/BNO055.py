"""
BNO055 MicroPython Driver
Author: Example
Description:
  A user-friendly BNO055 driver for MicroPython on a Raspberry Pi Pico.
  Leverages the official datasheet:
    https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
"""

import time
import machine
import math

class BNO055:
    # Common I2C Addresses
    ADDRESS_A = 0x28  # ADR pin low
    ADDRESS_B = 0x29  # ADR pin high

    # Register map (see datasheet, section 3.5)
    REG_CHIP_ID        = 0x00
    REG_PAGE_ID        = 0x07
    REG_ACCEL_DATA     = 0x08  # 6 bytes: X LSB/MSB, Y, Z
    REG_GYRO_DATA      = 0x14  # 6 bytes: X LSB/MSB, Y, Z
    REG_EULER_H_LSB    = 0x1A  # 6 bytes: heading, roll, pitch, each 2 bytes
    REG_QUATERNION     = 0x20  # 8 bytes: w, x, y, z
    REG_UNIT_SEL       = 0x3B
    REG_OPER_MODE      = 0x3D
    REG_PWR_MODE       = 0x3E
    REG_SYS_TRIGGER    = 0x3F
    REG_TEMP_SOURCE    = 0x40
    REG_CALIB_STAT     = 0x35  # Each nibble for mag, accel, gyro, sys
    REG_ST_RESULT      = 0x36
    REG_SYS_STAT       = 0x39
    REG_SYS_ERR        = 0x3A

    # Operation Modes
    CONFIGMODE = 0x00
    ACCONLY    = 0x01
    MAGONLY    = 0x02
    GYRONLY    = 0x03
    ACCMAG     = 0x04
    ACCGYRO    = 0x05
    MAGGYRO    = 0x06
    AMG        = 0x07
    # Fusion modes:
    IMU        = 0x08
    COMPASS    = 0x09
    M4G        = 0x0A
    NDOF_FMC_OFF = 0x0B
    NDOF       = 0x0C

    # Chip ID for BNO055
    CHIP_ID = 0xA0

    def __init__(self, i2c, address=ADDRESS_A, mode=NDOF):
        """
        Initialize the BNO055 with given I2C bus and address.
        The default mode is NDOF (sensor fusion).
        """
        self.i2c = i2c
        self.address = address
        self._mode = mode

        # Verify Chip ID
        chip_id = self._read_u8(self.REG_CHIP_ID)
        if chip_id != self.CHIP_ID:
            raise OSError("BNO055 not found. Expected 0xA0, got 0x{:02X}".format(chip_id))

        # Put device into CONFIGMODE before changing settings
        self._set_mode(self.CONFIGMODE)
        time.sleep_ms(20)

        # Select Page 0
        self._write_u8(self.REG_PAGE_ID, 0x00)

        # Optional: Reset the system if you want a clean start
        # self._write_u8(self.REG_SYS_TRIGGER, 0x20) # SYS_RST
        # time.sleep_ms(650)  # Wait for reset

        # Set default units to:
        #   Accelerometer: m/s^2
        #   Gyro: degrees/sec
        #   Euler: degrees
        #   Temperature: Celsius
        # UNIT_SEL bits = 0x00 for the above config
        self._write_u8(self.REG_UNIT_SEL, 0x00)

        # Switch to normal power mode
        self._write_u8(self.REG_PWR_MODE, 0x00)
        time.sleep_ms(10)

        # Clear SYS_TRIGGER
        self._write_u8(self.REG_SYS_TRIGGER, 0x00)
        time.sleep_ms(10)

        # Finally, set operation mode to the chosen fusion mode
        self._set_mode(self._mode)
        time.sleep_ms(20)

    def _set_mode(self, mode):
        """Set operation mode register (REG_OPER_MODE)."""
        self._write_u8(self.REG_OPER_MODE, mode)
        time.sleep_ms(20)

    def get_mode(self):
        """Return the current operation mode."""
        return self._read_u8(self.REG_OPER_MODE)

    def read_euler(self):
        """
        Reads Euler angles (heading, roll, pitch) in degrees.
        Each axis is 16 bits, with 1 LSB = 1/16 deg if in deg mode.
        Data in registers 0x1A..0x1F.
        """
        data = self.i2c.readfrom_mem(self.address, self.REG_EULER_H_LSB, 6)
        heading = self._to_signed_16(data[0] | (data[1] << 8)) / 16.0
        roll    = self._to_signed_16(data[2] | (data[3] << 8)) / 16.0
        pitch   = self._to_signed_16(data[4] | (data[5] << 8)) / 16.0
        return (heading, roll, pitch)

    def read_gyro(self):
        """
        Reads raw gyro data in deg/sec (assuming default config).
        GYR_DATA_X_LSB = 0x14 -> 6 bytes total.
        1 LSB = 1/16 dps.
        """
        data = self.i2c.readfrom_mem(self.address, self.REG_GYRO_DATA, 6)
        gx = self._to_signed_16(data[0] | (data[1] << 8)) / 16.0
        gy = self._to_signed_16(data[2] | (data[3] << 8)) / 16.0
        gz = self._to_signed_16(data[4] | (data[5] << 8)) / 16.0
        return (gx, gy, gz)

    def read_accel(self):
        """
        Reads raw accelerometer in m/s^2 (by default).
        ACC_DATA_X_LSB = 0x08 -> 6 bytes total.
        Typically 1 LSB = 1 mg = 0.01 m/s^2 if in m/s^2 mode; 
        but check the datasheet and UNIT_SEL config for exact scaling.
        """
        data = self.i2c.readfrom_mem(self.address, self.REG_ACCEL_DATA, 6)
        ax = self._to_signed_16(data[0] | (data[1] << 8)) / 100.0
        ay = self._to_signed_16(data[2] | (data[3] << 8)) / 100.0
        az = self._to_signed_16(data[4] | (data[5] << 8)) / 100.0
        return (ax, ay, az)

    def read_temp(self):
        """
        Reads the temperature in Celsius (if UNIT_SEL = 0x00).
        By default, register 0x34 is the temperature. 
        The datasheet states 1 LSB = 1°C if UNIT_SEL is 0 for Celsius.
        """
        val = self._to_signed_8(self._read_u8(0x34))
        return val

    def read_quat(self):
        """
        Reads the quaternion (w, x, y, z) as floats.
        Each component is a 16-bit signed value (2 bytes),
        stored in registers 0x20..0x27.
        1 LSB = 1 / 2^14.
        """
        data = self.i2c.readfrom_mem(self.address, self.REG_QUATERNION, 8)
        # data = [wLSB, wMSB, xLSB, xMSB, yLSB, yMSB, zLSB, zMSB]
        w = self._to_signed_16(data[0] | (data[1] << 8))
        x = self._to_signed_16(data[2] | (data[3] << 8))
        y = self._to_signed_16(data[4] | (data[5] << 8))
        z = self._to_signed_16(data[6] | (data[7] << 8))

        # Convert to float by dividing by 2^14
        scale = 1.0 / (1 << 14)
        w *= scale
        x *= scale
        y *= scale
        z *= scale
        return (w, x, y, z)

    def get_calibration_status(self):
        """
        Returns a tuple (sys, gyro, accel, mag) each 0..3 
        indicating calibration progress. 3 = fully calibrated.
        Register 0x35:
         bits [7:6]: mag cal,
         bits [5:4]: accel cal,
         bits [3:2]: gyro cal,
         bits [1:0]: sys cal
        """
        cal = self._read_u8(self.REG_CALIB_STAT)
        mag   = (cal >> 6) & 0x03
        accel = (cal >> 4) & 0x03
        gyro  = (cal >> 2) & 0x03
        sys   = (cal >> 0) & 0x03
        return (sys, gyro, accel, mag)

    def is_fully_calibrated(self):
        """
        Returns True if sys, gyro, accel, and mag are all 3.
        """
        sys, gyro, accel, mag = self.get_calibration_status()
        return (sys == 3 and gyro == 3 and accel == 3 and mag == 3)

    ### Helper methods
    def _read_u8(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def _write_u8(self, reg, val):
        self.i2c.writeto_mem(self.address, reg, bytes([val]))

    def _to_signed_16(self, val):
        """Helper: Convert 16-bit unsigned to signed."""
        if val > 32767:
            val -= 65536
        return val

    def _to_signed_8(self, val):
        """Helper: Convert 8-bit unsigned to signed."""
        if val > 127:
            val -= 256
        return val
