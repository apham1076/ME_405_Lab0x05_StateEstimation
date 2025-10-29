# motor_task.py
#
# ==============================================================================
# MotorControlTask
# ------------------------------------------------------------------------------
# This task periodically reads both encoders, computes position and velocity,
# updates time/pos/vel shares, and applies the latest motor efforts.
# ==============================================================================

from pyb import millis

class MotorControlTask:
    """Handles reading from encoders and actuating motors, updates encoder position and velocity"""

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_EFF = 1
    S2_RUN = 2

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 left_motor, right_motor,
                 left_encoder, right_encoder,
                 eff, mtr_enable, abort, mode,
                 time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh):

        # Hardware
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder

        # Shares
        self.eff = eff
        self.mode = mode
        
        # Queues
        self.time_sh = time_sh
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh

        # Flags
        self.mtr_enable = mtr_enable

        self.abort = abort
        
        self.t0 = 0 # zero the start time offset
        self.state = self.S0_INIT # ensure FSM starts in state S0_INIT

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True: # run infinite iterations of the FSM
            ### INIT STATE -----------------------------------------------------
            if (self.state == self.S0_INIT):
                # Zero encoders, disable motors, and initialize variables
                self.left_encoder.zero()
                self.right_encoder.zero()
                self.left_motor.disable()
                self.right_motor.disable()

                # Clear command and shares

                self.eff.put(0)
                self.mtr_enable.put(0)

                self.state = self.S1_WAIT_FOR_EFF # set next state

            ### WAITING STATE --------------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_EFF):
                if self.mtr_enable.get():
                    self.left_encoder.zero()
                    self.right_encoder.zero()
                    self.t0 = millis() # a timestamp to zero the time right when the motors are enabled
                    self.left_motor.enable()
                    self.right_motor.enable()
                    
                    self.state = self.S2_RUN # set next state
            
            ### RUN STATE ------------------------------------------------------
            elif (self.state == self.S2_RUN):
                # Update encoders
                self.left_encoder.update()
                self.right_encoder.update()

                # Read commanded effort and apply
                self.left_motor.set_effort(self.eff.get())
                self.right_motor.set_effort(self.eff.get())

                # Calculate the exact timestamp of the measurement
                t = millis() - self.t0

                # Get data samples
                pL = self.left_encoder.get_position()
                pR = self.right_encoder.get_position()
                vL = self.left_encoder.get_velocity()
                vR = self.right_encoder.get_velocity()

                # Write data samples to shares (for other tasks using them)
                # Ensure values being put are integers
                self.time_sh.put(t)
                self.left_pos_sh.put(int(pL))
                self.right_pos_sh.put(int(pR))
                self.left_vel_sh.put(int(vL))
                self.right_vel_sh.put(int(vR))

                # Disable if mtr_enable flag is cleared and return to WAIT state
                if not self.mtr_enable.get() or self.abort.get():
                    self.left_motor.disable()
                    self.right_motor.disable()
                    self.state = self.S1_WAIT_FOR_EFF
            
            yield self.state