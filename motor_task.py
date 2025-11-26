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
    S1_WAIT_FOR_ENABLE = 1
    S2_RUN = 2

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 left_motor, right_motor,
                 left_encoder, right_encoder,
                 battery,
                 eff, mtr_enable, abort, mode, setpoint, kp, ki, control_mode, start_time,
                 time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                 left_sp_sh, right_sp_sh, left_eff_sh, right_eff_sh):

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
        self.left_sp_sh = left_sp_sh
        self.right_sp_sh = right_sp_sh
        self.left_eff_sh = left_eff_sh
        self.right_eff_sh = right_eff_sh
        self.start_time = start_time

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
            return float(setpoint), float(setpoint)
        elif mode_val == 2:          # pivot in place
            return float(setpoint), -float(setpoint)
        else:                        # arc (simple fixed ratio; refine later if desired)
            RATIO = 0.6
            return float(setpoint), float(setpoint) * RATIO

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
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

                self.state = self.S1_WAIT_FOR_ENABLE # set next state

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_ENABLE):
                if self.mtr_enable.get():

                    if self.battery is not None:
                        try:
                            gain = self.battery.refresh()
                            print(f"Measured battery: {self.battery._cached_voltage:.2f} V, applying droop gain: {gain:.3f}")
                        except Exception as e:
                            print(f"Battery refresh failed: {e}")

                    self.left_encoder.zero()
                    self.right_encoder.zero()
                    self.t0 = millis() # a timestamp to zero the time right when the motors are enabled

                    self.start_time.put(self.t0)
                    self.left_motor.enable()
                    self.right_motor.enable()
                    
                    self.state = self.S2_RUN # set next state
            
            ### 2: RUN STATE ---------------------------------------------------
            elif (self.state == self.S2_RUN):
                # Check for abort signal
                if self.abort.get(): # (abort is a share)
                    self.left_motor.disable()
                    self.right_motor.disable()
                    self.left_controller.reset()
                    self.right_controller.reset()
                    self.abort.put(0)  # Reset abort flag after handling it
                    self.mtr_enable.put(0)  # Clear enable flag
                    self.state = self.S1_WAIT_FOR_ENABLE
                    yield self.state
                    continue

                # Update encoders
                self.left_encoder.update()
                self.right_encoder.update()

                # Calculate the exact timestamp of the measurements
                t = millis() - self.t0

                # Get current positions and velocities (in raw units, counts and counts/s, for data streaming)
                pL_counts = self.left_encoder.get_position_counts()
                pR_counts = self.right_encoder.get_position_counts()
                vL_counts_s = self.left_encoder.get_velocity_counts_s()
                vR_counts_s = self.right_encoder.get_velocity_counts_s()

                ### Determine which control mode to use:
                # 0 = Effort (open loop)
                # 1 = Velocity (closed loop)
                # 2 = Line Follow (outer + inner loop)
                mode_val = int(self.control_mode.get())

                if mode_val != 0: # if not in open-loop (effort) mode
                    # Get current velocities IN RAD/S for the closed loop controllers
                    vL_rad_s = self.left_encoder.get_velocity_rad_s() # RAD/S for controller
                    vR_rad_s = self.right_encoder.get_velocity_rad_s() # RAD/S for controller
                else:
                    pass  # no need for RAD/S velocities in open-loop mode

                # ----------------------------------------------------------
                # MODE 0: EFFORT (open loop control)
                # ----------------------------------------------------------
                if mode_val == 0:
                    effort = float(self.eff.get())

                    # # Battery droop compensation even in open-loop (effort) mode
                    # if self.battery is not None:
                    #     try:
                    #         effort *= self.battery.droop_gain()
                    #     except Exception:
                    #         pass  # fall back to raw effort on any ADC/battery reading error
                    
                    # Clamp to safe limits and apply effort
                    left_effort  = max(-100, min(100, effort))
                    right_effort = max(-100, min(100, effort))
                    self.left_motor.set_effort(left_effort)
                    self.right_motor.set_effort(right_effort)

                    # Reset controllers when in open-loop mode
                    self.left_controller.reset()
                    self.right_controller.reset()

                # ----------------------------------------------------------
                # MODE 1: VELOCITY (closed-loop velocity control)
                # ----------------------------------------------------------
                elif mode_val == 1:
                    # Update controller parameters
                    kp = self.kp.get()
                    ki = self.ki.get()
                    
                    # Update controller gains if needed
                    self.left_controller.set_gains(kp, ki)
                    self.right_controller.set_gains(kp, ki)
                    
                    # Update setpoints (per driving mode)
                    drive_mode = self.mode.get() or 1  # Default to straight if mode not set
                    setpoint = self.setpoint.get()
                    spL, spR = self._split_setpoints(drive_mode, setpoint)
                    self.left_controller.set_setpoint(spL)
                    self.right_controller.set_setpoint(spR)

                    # Run controllers with feedback velocities (in RAD/S !!!)
                    left_effort = self.left_controller.run(vL_rad_s)
                    right_effort = self.right_controller.run(vR_rad_s)

                    # Apply efforts
                    self.left_motor.set_effort(left_effort)
                    self.right_motor.set_effort(right_effort)

                # ----------------------------------------------------------
                # MODE 2: LINE FOLLOW (outer + inner loop)
                # ----------------------------------------------------------
                else: # mode_val == 2
                    # Update controller parameters
                    kp = self.kp.get()
                    ki = self.ki.get()

                    # Update controller gains if needed
                    self.left_controller.set_gains(kp, ki)
                    self.right_controller.set_gains(kp, ki)

                    # Generate velocity setpoints with SteeringTask
                    spL = float(self.left_sp_sh.get() if self.left_sp_sh else 0)
                    spR = float(self.right_sp_sh.get() if self.right_sp_sh else 0)
                    self.left_controller.set_setpoint(spL)
                    self.right_controller.set_setpoint(spR)

                    # Run controllers with feedback velocities (in RAD/S !!!)
                    left_effort = self.left_controller.run(vL_rad_s)
                    right_effort = self.right_controller.run(vR_rad_s)

                    # Apply efforts
                    self.left_motor.set_effort(left_effort)
                    self.right_motor.set_effort(right_effort)
                # ----------------------------------------------------------

                # Write data samples (raw units of counts and counts/s) to shares (for data streaming and other tasks using them)
                self.time_sh.put(int(t))
                self.left_pos_sh.put(int(pL_counts))
                self.right_pos_sh.put(int(pR_counts))
                self.left_vel_sh.put(int(vL_counts_s))
                self.right_vel_sh.put(int(vR_counts_s))

                # Store efforts to shares for monitoring

            yield self.state