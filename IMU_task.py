# IMU_task.py
#
# ==============================================================================
# IMUTask
# ------------------------------------------------------------------------------
# This task periodically reads data from the IMU sensor and writes it to shares
# that can be accessed by other tasks. This allows state_estimation_task to run 
# faster.
# ==============================================================================

import math

class IMUTask:
    '''A task that reads IMU data and writes it to shares'''

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT = 1
    S2_READING = 2

    ### Initialize the attributes
    # --------------------------------------------------------------------------
    def __init__(self, imu, psi_share, psi_dot_share, first_psi_flag,
                 read_IMU_flag):
        '''Initialize the IMU task'''

        # Hardware
        self.imu = imu

        # Shares
        self.psi_share = psi_share
        self.psi_dot_share = psi_dot_share

        # Flags
        self.first_psi_flag = first_psi_flag
        self.read_IMU_flag = read_IMU_flag

        self.state = self.S0_INIT # esnure the FSM starts in the state S0_INIT

    def run(self):
        while True:
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                # Get the initial measured yaw angle reading from IMU
                psi_meas_init = self.imu.read_euler_angles()[0] * (math.pi / 180.0)
                # Write initial measured yaw angle to share
                self.psi_share.put(psi_meas_init)
                # tell other tasks that the initial angle has been stored
                self.first_psi_flag.put(1)
                self.state = self.S1_WAIT  # Transition to wait state

            ### 1: WAIT STATE --------------------------------------------------
            elif (self.state == self.S1_WAIT):
                # Wait for the read_IMU_flag to be set
                if self.read_IMU_flag.get():
                    self.state = self.S2_READING  # Transition to read state
                # if read_IMU_flag is not set, fall through to the yield

            ### 2: READ IMU STATE ----------------------------------------------
            elif (self.state == self.S2_READING):
                # Read yaw angle and yaw rate from IMU
                psi = self.imu.read_euler_angles()[0] * (math.pi / 180.0)  # Yaw angle in radians
                psi_dot = self.imu.read_angular_velocity()[2] * (math.pi / 180.0)  # Yaw rate in radians/s

                # Write to shares
                self.psi_share.put(psi)
                self.psi_dot_share.put(psi_dot)
            
            yield self.state
