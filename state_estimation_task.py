# state_estimation_task.py
#
# ==============================================================================
# StateEstimationTask
# ------------------------------------------------------------------------------
# This task predicts the future state of the system using a system model and 
# sensor inputs
# ==============================================================================

from pyb import millis
from ulab import numpy as np
import math

class StateEstimationTask:
    """Estimates the state of the robot using sensor inputs and a system model."""

    # --------------------------------------------------------------------------
    # CONSTANTS
    GEAR_RATIO = 3952/33  # Gear ratio of motor to wheel (~120)
    CPR_MOTOR = 12 # Counts per rev of the motor shaft (before gearbox)
    CPR_WHEEL = GEAR_RATIO*CPR_MOTOR  # Counts per rev of the wheel (~1440)
    RAD_PER_COUNT = 2 * (math.pi) / CPR_WHEEL  # Radians per count
    WHEEL_RADIUS_MM = 35  # Wheel radius in mm
    # --------------------------------------------------------------------------

    # The states of the FSM
    S0_INIT = 0
    S1_ESTIMATING = 1

    # --------------------------------------------------------------------------
    ### Initialize attributes
    # --------------------------------------------------------------------------
    def __init__(self, start_time, obsv_time_sh, left_pos_sh, right_pos_sh, 
                 left_vel_sh, right_vel_sh,
                 psi_sh, psi_dot_sh, first_psi_flag, read_IMU_flag,
                 left_eff_sh, right_eff_sh,
                 battery,
                 obsv_sL_sh, obsv_sR_sh, obsv_psi_sh, obsv_psi_dot_sh,
                 obsv_left_vel_sh, obsv_right_vel_sh, obsv_s_sh, obsv_yaw_sh):

        # Shares (inputs from other tasks)
        self.start_time = start_time
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh
        self.psi_sh = psi_sh  # Yaw angle from IMU_task (rad)
        self.psi_dot_sh = psi_dot_sh  # Yaw rate from IMU_task (rad/s)
        self.left_eff_sh = left_eff_sh
        self.right_eff_sh = right_eff_sh

        # Shares (outputs to other tasks)
        self.obsv_time_sh = obsv_time_sh
        self.obsv_sL_sh = obsv_sL_sh
        self.obsv_sR_sh = obsv_sR_sh
        self.obsv_psi_sh = obsv_psi_sh
        self.obsv_psi_dot_sh = obsv_psi_dot_sh
        self.obsv_left_vel_sh = obsv_left_vel_sh
        self.obsv_right_vel_sh = obsv_right_vel_sh
        self.obsv_s_sh = obsv_s_sh
        self.obsv_yaw_sh = obsv_yaw_sh

        # Flags
        self.first_psi_flag = first_psi_flag
        self.read_IMU_flag = read_IMU_flag

        # Hardware
        self.battery = battery
        # Parameters
        self.r = 0.035 # wheel radius (m)
        self.w = 0.141  # wheelbase (distance between wheels) (m)

        self.V_meas = self.battery.read_voltage()  # initial battery voltage

        # Arrays for discrete-time state-space model        
        self.A_D = np.array([[0, 0, 0.1331, 0],
                             [0, 0, 0.1331, 0],
                             [0, 0, 0, 0],
                             [0, 0, 0, 0]])

        self.B_D = np.array([[0.0406, 0.0373, -0.0666, -0.0666, 0, -2.0123],
                             [0.0373, 0.0406, -0.0666, -0.0666, 0, 2.0123],
                             [0, 0, 0.5, 0.5, 0, 0],
                             [0, 0, -0.0698, 0.0698, 0.9902, 0.0001]])
        
        self.C = np.array([[0, 0, 1, -self.w/2],
                           [0, 0, 1, self.w/2],
                           [0, 0, 0, 1],
                           [-self.r/self.w, self.r/self.w, 0, 0]])
        
        self.x_k = np.array([[0],
                             [0], 
                             [0],
                             [0]])
        
        self.x_kplus1 = np.array([[0],
                                 [0],
                                 [0],
                                 [0]])
        
        self.y_k = np.array([[0],
                           [0],
                           [0],
                           [0]])


        self.state = self.S0_INIT # ensure FSM starts in state S0_INIT

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                # Set initial yaw angle
                s_L = self.left_pos_sh.get() * self.RAD_PER_COUNT * self.WHEEL_RADIUS_MM / 1000.0 # initial left wheel displacement (m)
                s_R = self.right_pos_sh.get() * self.RAD_PER_COUNT * self.WHEEL_RADIUS_MM / 1000.0  # initial right wheel displacement (m)
                psi = (s_R - s_L) / self.w  # initial yaw angle from wheel odometry (rad)
                if first_psi_flag:
                    psi_meas = self.psi_sh.get()  # initial yaw angle from IMU measurement (rad)
                    self.psi_offset = psi_meas - psi # offset between IMU yaw and odometry yaw

                    # Get initial time
                    self.t0 = self.start_time.get()

                    self.state = self.S1_ESTIMATING # set next state
                yield self.state
            
            ### 1: ESTIMATING STATE --------------------------------------------
            elif (self.state == self.S1_ESTIMATING):

                # Determine input vector, u
                V_L = self.left_eff_sh.get()
                V_R = self.right_eff_sh.get()
                V_L = V_L * self.V_meas / 100.0
                V_R = V_R * self.V_meas / 100.0

                # Determine values for output state vector, y
                s_L = self.left_pos_sh.get() * self.RAD_PER_COUNT * self.WHEEL_RADIUS_MM / 1000.0 # Left wheel displacement (m)
                s_R = self.right_pos_sh.get() * self.RAD_PER_COUNT * self.WHEEL_RADIUS_MM / 1000.0 # Right wheel displacement (m)
                psi = self.psi_sh.get()  # Yaw angle from IMU (rad)
                psi -= self.psi_offset  # Subtract offset
                psi_dot = self.psi_dot_sh.get()  # Yaw rate (rad/s)
                
                # Form u_star = [ u, y ] (column vector of inputs and outputs)
                u_star = np.array([ [V_L],      # Left voltage (V)
                                    [V_R],      # Right voltage (V)
                                    [s_L],      # Left position (m)
                                    [s_R],      # Right position (m)
                                    [psi],      # Yaw angle (rad)
                                    [psi_dot]]) # Yaw rate (rad/s)
                
                # Predict next state
                self.x_kplus1 = np.dot(self.A_D, self.x_k) + np.dot(self.B_D, u_star)
                # Determine present output
                self.y_k = np.dot(self.C, self.x_k)

                # Put observer values in shares
                self.obsv_sL_sh.put(float(self.y_k[0,0]))
                self.obsv_sR_sh.put(float(self.y_k[1,0]))

                self.obsv_left_vel_sh.put(float(self.x_kplus1[0,0]))
                self.obsv_right_vel_sh.put(float(self.x_kplus1[1,0]))
                self.obsv_s_sh.put(float(self.x_kplus1[2,0]))
                self.obsv_yaw_sh.put(float(self.x_kplus1[3,0]))
                
                # Put time log for the observer data in a share
                t = millis() - self.t0
                self.obsv_time_sh.put(int(t))

                # Update state for next iteration
                self.x_k = self.x_kplus1
                
            yield self.state