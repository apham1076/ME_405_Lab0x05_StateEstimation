# steering_task.py
# ==============================================================================
# SteeringTask (outer loop for closed-loop line following)
# ------------------------------------------------------------------------------
# Uses the IR sensor array to compute a steering correction and publish left
# right motor VELOCITY setpoints (to maintain the inner closed-loop control)
# for the MotorControlTask.
# ==============================================================================

from pyb import millis

class SteeringTask:
    """Outer-loop controller for closed-loop line following."""

    # FSM States
    S0_INIT = 0
    S1_WAIT_ENABLE = 1
    S2_FOLLOW = 2
    S3_LOST = 3

    def __init__(self, ir_array, battery,
                 lf_enable, ir_cmd,
                 left_sp_sh, right_sp_sh):
        # Hardware
        self.ir = ir_array
        self.battery = battery

        # Shares
        self.lf_enable = lf_enable
        self.ir_cmd = ir_cmd
        self.left_sp_sh = left_sp_sh # share for left motor velocity setpoint
        self.right_sp_sh = right_sp_sh # share for right motor velocity setpoint

        # Tuning parameters
        self.Kp_line = 2.0 # proportional steering gain (increase if sluggish, reduce if hunting)
        self.v_target = 2.0 # forward target [rad/s] (bump up after stable)

        # Derived clamp: v_target + Kp_line * half of index span
        idx_min = min(self.ir.sensor_index)
        idx_max = max(self.ir.sensor_index)
        half_span = 0.5 * (idx_max - idx_min)    # e.g., (11-1)/2 = 5.0
        self._max_sp = self.v_target + self.Kp_line * half_span

        # Lost-line behavior
        self.search_speed = 0.5 # fraction of v_target to creep forward while searching
        self.reacquire_thresh = 0.05  # minimum sum(norm) threshold to consider the line as "seen"

        # FSM state initialization
        self.state = self.S0_INIT

        # Timing
        self.last_time = millis()

    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    def _clamp(self, x, lo, hi):
        return lo if x < lo else (hi if x > hi else x)

    # --------------------------------------------------------------------------
    def _publish(self, v_left, v_right):
        v_left = self._clamp(v_left, -self._max_sp, self._max_sp)
        v_right = self._clamp(v_right, -self._max_sp, self._max_sp)
        self.left_sp_sh.put(v_left)
        self.right_sp_sh.put(v_right)

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True:
            # Handle calibration commands regardless of FSM state
            cmd = self.ir_cmd.get()
            if cmd == 1:   # white calibration
                print("Calibrating white background...")
                self.ir.calibrate('w')
                self.ir_cmd.put(0)
            elif cmd == 2: # black calibration
                print("Calibrating black line...")
                self.ir.calibrate('b')
                self.ir_cmd.put(0)

            # FSM actually starts here
            # S0: INIT ---------------------------------------------------------
            if self.state == self.S0_INIT:
                # Nothing special to init beyond being explicit
                self._publish(0.0, 0.0) # ensure motors are stopped
                self.state = self.S1_WAIT_ENABLE

            # S1: WAIT FOR ENABLE ----------------------------------------------
            elif self.state == self.S1_WAIT_ENABLE:
                self._publish(0.0, 0.0) # ensure motors are stopped
                if self.lf_enable.get(): # if line following enabled
                    self.state = self.S2_FOLLOW # go to FOLLOW state

            # S2: FOLLOW LINE -------------------------------------------------
            elif self.state == self.S2_FOLLOW:
                if not self.lf_enable.get(): # if line following disabled
                    self._publish(0.0, 0.0) # ensure motors are stopped
                    self.state = self.S1_WAIT_ENABLE # go to WAIT ENABLE state
                else:
                    centroid, seen = self.ir.get_centroid() # get line centroid
                    if not seen:
                        # No line detected -> go to LOST
                        self.state = self.S3_LOST
                    else:
                        center = self.ir.center_index() # ideal centroid index
                        error = centroid - center # positive if line is to the right
                        correction = self.Kp_line * error # steering correction

                        v_left = self.v_target + correction # correct steering
                        v_right = self.v_target - correction # correct steering
                        self._publish(v_left, v_right)

            # S3: LOST LINE --------------------------------------------------
            elif self.state == self.S3_LOST:
                if not self.lf_enable.get(): # if line following disabled
                    self._publish(0.0, 0.0) # ensure motors are stopped
                    self.state = self.S1_WAIT_ENABLE # go to WAIT ENABLE state
                else:
                    # Gentle creep to reacquire (or set both to 0.0 if you prefer a stop)
                    v = self.search_speed * self.v_target
                    self._publish(v, v)

                    # Quick check for line reacquisition
                    # (use read() to avoid computing centroid every tick)
                    norm = self.ir.read()
                    if sum(norm) > self.reacquire_thresh:
                        self.state = self.S2_FOLLOW

            yield self.state