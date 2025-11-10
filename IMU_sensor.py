# IMU_sensor.py
# ==============================================================================
# IMU driver interface for a BNO055 IMU using I2C communication
#
#
#
# ==============================================================================

from pyb import delay
from struct import calcsize, unpack_from

class IMU:
    '''A IMU driver interface that works with an IMU using a SCL, SDA, RST inputs such as the BNO055'''

    DEV_ADDR = 0x28  # I2C address for the IMU

    class reg:
        # Data registers and struct format strings
        EULER_DATA_ALL = (0x1A, "<hhh")     # heading, roll, pitch (6 bytes)
        GYRO_DATA_ALL  = (0x14, "<hhh")     # x, y, z (6 bytes)
        CALIB_STAT     = (0x35, "<B")       # single byte
        CALIB_PROFILE  = (0x55, "<hhhhhhhhhhh")  # 11 x 16-bit values (22 bytes)
        OPR_MODE       = (0x3D, "<B")
        SYS_TRIGGER    = (0x3F, "<B")

    def __init__(self, i2c):
        '''Initialize an IMU object'''
        self._i2c = i2c
        self._buf = bytearray((0 for n in range(22))) # buffer for reading data
        self._current_mode = "config"  # Start in config mode

        # Mode dictionary
        self.mode_dict = {
            "config":       {"code": 0x00, "name": "Configuration mode"},
            "acconly":      {"code": 0x01, "name": "Accelerometer only"},
            "magonly":      {"code": 0x02, "name": "Magnetometer only"},
            "gyronly":      {"code": 0x03, "name": "Gyroscope only"},
            "accmag":       {"code": 0x04, "name": "Accelerometer + Magnetometer"},
            "accgyro":      {"code": 0x05, "name": "Accelerometer + Gyroscope"},
            "maggyro":      {"code": 0x06, "name": "Magnetometer + Gyroscope"},
            "any_motion":   {"code": 0x07, "name": "Accelerometer + Magnetometer + Gyroscope"},
            "imuplus":      {"code": 0x08, "name": "IMU Plus (Accel + Gyro + Fusion)"},
            "compass":      {"code": 0x09, "name": "Compass (Accel + Mag + Fusion)"},
            "m4g":          {"code": 0x0A, "name": "M4G (Accel + Mag + Fusion)"},
            "ndof_fmc_off": {"code": 0x0B, "name": "NDOF (Fast Mag Calibration Off)"},
            "ndof":         {"code": 0x0C, "name": "NDOF (Full 9-DOF Fusion)"}
        }
        
        delay(700)  # Delay for IMU to start up

    # --------------------------------------------------------------------------
    def _read_reg(self, reg):
        '''Generic register read method using calcsize() and unpack_from()'''
        addr, fmt = reg
        # Determine number of bytes to read
        length = calcsize(fmt)
        # Create a memoryview object of the right size
        buf = memoryview(self._buf)[:length]
        # Read from the I2C bus into the memoryview
        self._i2c.mem_read(buf, IMU.DEV_ADDR, addr)
        # Unpack the bytes into a tuple
        return unpack_from(fmt, buf)

    # --------------------------------------------------------------------------
    def set_operation_mode(self, mode_name):
        '''Set the operation mode of the IMU.'''
        # check for valid mode name
        try:
            mode_name = str(mode_name).lower()
            if mode_name not in self.mode_dict:
                raise ValueError(f"Invalid mode name: {mode_name}")
        except Exception as e:
            print(f"Error in set_operation_mode: {e}")
            return
        
        code = self.mode_dict[mode_name]["code"]
        self._i2c.mem_write(bytes([code]), IMU.DEV_ADDR, self.reg.OPR_MODE[0], timeout=100)
        delay(30)  # Small delay for mode switch
        self._current_mode = mode_name
        print(f"IMU operation mode set to {mode_name}")

    # --------------------------------------------------------------------------
    def read_calibration_status(self):
        '''Read the calibration status of the IMU and return (sys, gyr, acc, mag) each from 0-3.'''
        print("Reading calibration status")
        # The calibration status is 1 byte with 2 bits each for sys, gyr, acc, mag
        cal_start = self._read_reg(self.reg.CALIB_STAT)
        # Shift bits and mask to isolate each 2-bit field
        mag_stat = cal_start[0] & 0b11
        acc_stat = (cal_start[0] >> 2) & 0b11
        gyr_stat = (cal_start[0] >> 4) & 0b11
        sys_stat = (cal_start[0] >> 6) & 0b11
        return (sys_stat, gyr_stat, acc_stat, mag_stat)
    
    # --------------------------------------------------------------------------
    def read_calibration_coeffs(self, save_to_file=True, filename='imu_cal.txt'):
        '''Read the 22 bytes of calibration coefficients (offsets + radii) from the IMU'''

        # MUST be in config mode to read calibration coefficients
        prev_mode = self._current_mode
        if prev_mode != "config":
            print("Switching to CONFIG mode to read calibration coefficients")
            self.set_operation_mode("config")

        print("Reading calibration coefficients")
        coeffs = self._read_reg(self.reg.CALIB_PROFILE)
        # decode each signed 16-bit little-endian value
        accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, \
        gyro_x, gyro_y, gyro_z, accel_r, mag_r = coeffs

        if save_to_file:
            # Save the raw 22 bytes to a file
            length = calcsize(self.reg.CALIB_PROFILE[1])
            buf = memoryview(self._buf)[:length]
            with open(filename, 'wb') as f: # 'write binary' mode
                f.write(buf)

        data = {
            "accel_offset": (accel_x, accel_y, accel_z),
            "mag_offset":   (mag_x, mag_y, mag_z),
            "gyro_offset":  (gyro_x, gyro_y, gyro_z),
            "accel_radius": accel_r,
            "mag_radius":   mag_r
        }

        if prev_mode != "config":
            print(f"Restoring previous mode: {prev_mode}")
            self.set_operation_mode(prev_mode)

        return data

    # --------------------------------------------------------------------------
    def write_calibration_coeffs(self, filename='imu_cal.txt'):

        # MUST be in config mode to write calibration coefficients
        prev_mode = self._current_mode
        if prev_mode != "config":
            print("Switching to CONFIG mode to write calibration coefficients")
            self.set_operation_mode("config")

        with open(filename, 'rb') as f: # 'read binary' mode
            coeffs = f.read()
        if len(coeffs) != calcsize(self.reg.CALIB_PROFILE[1]):
            raise ValueError("Calibration file must contain exactly 22 bytes.")
        # Write the coefficients to the IMU
        print("Writing calibration coefficients to IMU")
        self._i2c.mem_write(coeffs, IMU.DEV_ADDR, self.reg.CALIB_PROFILE[0], timeout=100)
        delay(20) # Small delay for write to complete
        print("Calibration coefficients written to IMU")

        if prev_mode != "config":
            print(f"Restoring previous mode: {prev_mode}")
            self.set_operation_mode(prev_mode)
        

    # --------------------------------------------------------------------------
    def read_euler_angles(self):
        '''Return the Euler angles (heading, roll, pitch) in degrees.'''
        # The 6 bytes are: H_LSB, H_MSB, R_LSB, R_MSB, P_LSB, P_MSB
        print("Reading Euler angles")
        heading, roll, pitch = self._read_reg(self.reg.EULER_DATA_ALL)
        # Convert to degrees (1 degree = 16 LSB) and return the values
        return (heading / 16.0, roll / 16.0, pitch / 16.0)

    # --------------------------------------------------------------------------
    def read_angular_velocity(self):
        '''Return the angular velocity (x, y, z) in degrees/s.'''
        # The 6 bytes are: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
        print("Reading angular velocity")
        x, y, z = self._read_reg(self.reg.GYRO_DATA_ALL)
        # Convert to degrees/s (1 degree/s = 16 LSB/s)
        return (x / 16.0, y / 16.0, z / 16.0)
    
    # --------------------------------------------------------------------------
    def reset(self):
        '''Reset the IMU'''
        # The IMU reset is performed by setting bit 5 of the SYS_TRIGGER register, which we access via the reset_addr
        self._i2c.mem_write(bytes([0x20]), IMU.DEV_ADDR, self.reg.SYS_TRIGGER[0], timeout=100)
        delay(700)  # Delay for reset to complete
        self._current_mode = "config"
        print("IMU reset complete; set to CONFIG mode")