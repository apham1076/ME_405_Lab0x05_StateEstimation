# motor_task.py
#
# ==============================================================================
# MotorControlTask
# ------------------------------------------------------------------------------
# This task periodically reads both encoders, computes position and velocity,
# updates time/pos/vel shares, and applies the latest motor efforts.
# ==============================================================================

from pyb import millis
from closed_loop import ClosedLoop

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
                 battery,
                 eff, mtr_enable, abort, mode, setpoint, kp, ki, control_mode,
                 time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                 lf_enable=None, left_eff_sh=None, right_eff_sh=None):

        # Hardware
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.battery = battery

        # Shares
        self.eff = eff
        self.mode = mode
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.control_mode = control_mode
        self.lf_enable = lf_enable
        self.left_eff_sh = left_eff_sh
        self.right_eff_sh = right_eff_sh

        # Queues
        self.time_sh = time_sh
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh

        # Flags
        self.mtr_enable = mtr_enable
        self.abort = abort

        # Controllers
        self.left_controller = ClosedLoop(effort_limits=(-100, 100))
        self.right_controller = ClosedLoop(effort_limits=(-100, 100))
        # Attach battery objects to controllers for voltage droop
        self.left_controller.attach_battery(self.battery)
        self.right_controller.attach_battery(self.battery)
        
        self.t0 = 0 # zero the start time offset
        self.state = self.S0_INIT # ensure FSM starts in state S0_INIT

    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    ### Split setpoints for left and right motors based on driving mode
    def _split_setpoints(self, mode_val, setpoint):
        """
        mode 1: straight  -> (spL, spR) = (sp, sp)
        mode 2: pivot     -> (spL, spR) = ( sp, -sp)
        mode 3: arc       -> (spL, spR) = ( sp, sp*RATIO )
        """
        if mode_val == 1:            # straight
            return setpoint, setpoint
        elif mode_val == 2:          # pivot in place
            return setpoint, -setpoint
        else:                        # arc (simple fixed ratio; refine later if desired)
            RATIO = 0.6
            return setpoint, int(setpoint * RATIO)

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

                # Reset controllers
                self.left_controller.reset()
                self.right_controller.reset()

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

                # Get current velocities and positions
                pL = self.left_encoder.get_position()
                pR = self.right_encoder.get_position()
                vL = self.left_encoder.get_velocity()
                vR = self.right_encoder.get_velocity()
                # print(f"Encoder Positions - Left: {pL}, Right: {pR}")
                # print(f"Encoder Velocities - Left: {vL}, Right: {vR}")

                # --------------------------------------------------------------
                # Determine which control mode to use: Line Follow, Velocity, or Effort
                # --------------------------------------------------------------
                if (self.lf_enable is not None) and self.lf_enable.get():
                    # ----------------------------------------------------------
                    # LINE FOLLOW MODE
                    # ----------------------------------------------------------
                    # Retrieve left/right efforts published by the line follow task
                    left_effort  = float(self.left_eff_sh.get()  if self.left_eff_sh  else 0)
                    right_effort = float(self.right_eff_sh.get() if self.right_eff_sh else 0)

                    # Apply battery droop compensation
                    if self.battery is not None:
                        try:
                            gain = self.battery.droop_gain()
                            left_effort  *= gain
                            right_effort *= gain
                        except Exception:
                            pass  # fall back to raw efforts if ADC/battery read fails

                    # Clamp and apply
                    left_effort  = max(-100, min(100, left_effort))
                    right_effort = max(-100, min(100, right_effort))
                    self.left_motor.set_effort(left_effort)
                    self.right_motor.set_effort(right_effort)

                    # Reset PI controllers since we're not using them in this mode
                    self.left_controller.reset()
                    self.right_controller.reset()

                elif self.control_mode.get():  # Velocity mode
                    # ----------------------------------------------------------
                    # VELOCITY (CLOSED LOOP) MODE
                    # ----------------------------------------------------------
                    # Update controller parameters
                    kp = self.kp.get()  # Convert from fixed-point
                    ki = self.ki.get()
                    setpoint = self.setpoint.get()
                    
                    # Update controller gains if needed
                    self.left_controller.set_gains(kp, ki)
                    self.right_controller.set_gains(kp, ki)
                    
                    # Update setpoints (per driving mode)
                    mode_val = self.mode.get() or 1  # Default to straight if mode not set
                    sp = self.setpoint.get()
                    spL, spR = self._split_setpoints(mode_val, sp)
                    self.left_controller.set_setpoint(spL)
                    self.right_controller.set_setpoint(spR)

                    # Calculate control efforts
                    left_effort = self.left_controller.run(vL)
                    right_effort = self.right_controller.run(vR)

                    # Apply efforts
                    self.left_motor.set_effort(left_effort)
                    self.right_motor.set_effort(right_effort)

                else:
                    # ----------------------------------------------------------
                    # EFFORT (OPEN LOOP) MODE
                    # ----------------------------------------------------------
                    # print("In effort mode")
                    effort = float(self.eff.get())

                    # Apply battery droop compensation in open-loop (effort) mode
                    if self.battery is not None:
                        try:
                            effort *= self.battery.droop_gain()
                        except Exception:
                            pass  # fall back to raw effort on any ADC/battery reading error
                    
                    # Clamp to safe limits and apply effort
                    left_effort  = max(-100, min(100, effort))
                    right_effort = max(-100, min(100, effort))
                    self.left_motor.set_effort(left_effort)
                    self.right_motor.set_effort(right_effort)

                    # Reset controllers when not in use
                    self.left_controller.reset()
                    self.right_controller.reset()


                # Calculate the exact timestamp of the measurement
                t = millis() - self.t0

                # Write data samples to shares (for other tasks using them)
                # Ensure values being put are integers
                self.time_sh.put(int(t))
                self.left_pos_sh.put(float(pL))
                self.right_pos_sh.put(float(pR))
                self.left_vel_sh.put(float(vL))
                self.right_vel_sh.put(float(vR))
                
                # print("Motor Task Shares:")
                # print(self.time_sh.get())
                # print(self.left_pos_sh.get())
                # print(self.right_pos_sh.get())
                # print(self.left_vel_sh.get())
                # print(self.right_vel_sh.get())

                yield self.state

                # Disable if mtr_enable flag is cleared or abort is triggered
                if not self.mtr_enable.get() or self.abort.get():
                    self.left_motor.disable()
                    self.right_motor.disable()
                    self.abort.put(0)  # Reset abort flag after handling it
                    # Reset controllers
                    self.left_controller.reset()
                    self.right_controller.reset()
                    self.state = self.S1_WAIT_FOR_EFF
            
            yield self.state