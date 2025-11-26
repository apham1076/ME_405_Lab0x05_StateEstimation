# IMU_task.py
#
# ==============================================================================
# IMUTask
# ------------------------------------------------------------------------------
# This task periodically reads data from the IMU sensor and writes it to shares
# that can be accessed by other tasks.
# ==============================================================================

import math

class IMUTask:
    '''A task that reads IMU data and writes to shares'''

    # The states of the FSM
    S0_INIT = 0
    S1_READ_IMU = 1

    ### Initialize the attributes
    # --------------------------------------------------------------------------
    def __init__(self, imu, yaw_share, yaw_rate_share):
        '''Initialize the IMU task'''

        # Hardware
        self.imu = imu

        # Shares
        self.yaw_share = yaw_share
        self.yaw_rate_share = yaw_rate_share

        self.state = self.S0_INIT # esnure the FSM starts in the state S0_INIT

    def run(self):
        while True:
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                psi_init = self.imu.read_euler_angles()[0] * (math.pi / 180.0)  # Initial yaw angle in radians
                self.state = self.S1_READ_IMU  # Transition to read state

            ### 1: READ IMU STATE ----------------------------------------------
            elif (self.state == self.S1_READ_IMU):
                # Read yaw angle and yaw rate from IMU
                psi = self.imu.read_euler_angles()[0] * (math.pi / 180.0)  # Yaw angle in radians
                psi_dot = self.imu.read_angular_velocity()[2] * (math.pi / 180.0)  # Yaw rate in radians/s

                # Write to shares
                self.yaw_share.put(psi)
                self.yaw_rate_share.put(psi_dot)
            
            yield self.state
