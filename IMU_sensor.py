
class IMU:
    '''A IMU driver interface that works with an IMU using a SCL, SDA, RST inputs such as the BNO055'''

    def __init__(self, i2c):
        '''Initialize an IMU object'''
        self.i2c = i2c
        self.dev_addr = 0x28            # IMU device address
        self.opr_mode_addr = 0x3D       # Operation mode address
        self.ndof = 0x0C                # NDOF mode code
        self.calib_stat_addr = 0x35     # Calibration status address
        self.offset_addr = 0x55         # Calibration offset address
        self.euler_addr = 0x1A          # Euler angles address
        self.gyro_data_addr = 0x14        # Angular velocity address
        self.accel_data_addr = 0x08      # Acceleration data address

        # self.calib_buf = bytearray((0 for _ in range(8)))


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

    def set_operation_mode(self, mode_name):
        '''Set the operation mode of the IMU'''
        # check for valid mode name
        try:
            mode_name = str(mode_name).lower()
            if mode_name not in self.mode_dict:
                raise ValueError(f"Invalid mode name: {mode_name}")
        except Exception as e:
            print(f"Error in set_operation_mode: {e}")
            return
                
        self.i2c.mem_write(bytes[self.mode_dict[mode_name]["code"]], self.dev_addr, self.opr_mode_addr, timeout=100)
        print(f"Setting operation mode to {mode_name}")

    def read_calibration_status(self):
        '''Read the calibration status of the IMU'''
        self.cal_stat = self.i2c.mem_read(8, self.dev_addr, self.calib_stat_addr)
        print("Reading calibration status")
        self.mag_stat = self.cal_stat[0] & 0b11
        self.acc_stat = (self.cal_stat[0] >> 2) & 0b11
        self.gyr_stat = (self.cal_stat[0] >> 2) & 0b11
        self.sys_stat = (self.cal_stat[0] >> 2) & 0b11
        return self.cal_stat
    
    def read_calibration_coeffs(self):
        '''Read the calibration coefficients (offsets) from the IMU'''
        self.cal_coeffs = self.i2c.mem_read(22, self.dev_addr, self.offset_addr)
        print("Reading calibration coefficients")
        with open('imu_cal.dat', 'wb') as f:
            f.write(self.cal_coeffs)
        return self.cal_coeffs

    def write_calibration_coeffs(self):
        with open('imu_cal.dat', 'rb') as f:
            coeffs = f.read()
        '''Write the calibration coefficients back to the IMU'''
        self.i2c.mem_write(coeffs, self.dev_addr, self.offset_addr)
        print("Writing calibration coefficients")

    def read_euler_angles(self):
        '''Read the Euler angles from the IMU'''
        print("Reading Euler angles")
        return (0.0, 0.0, 0.0)  # Example Euler angles
    
    def read_angular_velocity(self):
        '''Read the angular velocity from the IMU'''
        print("Reading angular velocity")
        return (0.0, 0.0, 0.0)  # Example angular velocity
    
    # def reset(self):
    #     '''Reset the IMU'''
    #     print("Resetting IMU")
    #     pass

    