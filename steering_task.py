# steering_task.py
# ==============================================================================
# SteeringTask
# ------------------------------------------------------------------------------
# Computes line-following steering efforts based on IR sensor centroid.
# ==============================================================================

from pyb import millis

class SteeringTask:
    """Uses IR sensor readings to compute differential efforts for line following."""

    def __init__(self, ir_array, battery,
                 lf_enable, ir_cmd,
                 left_eff_sh, right_eff_sh):
        # Hardware
        self.ir = ir_array
        self.battery = battery

        # Shares
        self.lf_enable = lf_enable
        self.ir_cmd = ir_cmd
        self.left_eff_sh = left_eff_sh
        self.right_eff_sh = right_eff_sh

        # Parameters
        self.Kp_line = 35.0    # proportional steering gain
        self.base_effort = 40  # base forward effort
        self.last_time = millis()

        # FSM state
        self.state = 0

    def run(self):
        """Non-blocking cooperative task to compute left/right efforts."""
        while True:
            # --- Handle calibration commands ---
            if self.ir_cmd.get() == 1:   # white calibration
                print("Calibrating white background...")
                self.ir.calibrate('w')
                self.ir_cmd.put(0)
            elif self.ir_cmd.get() == 2: # black calibration
                print("Calibrating black line...")
                self.ir.calibrate('b')
                self.ir_cmd.put(0)

            # --- Only compute when line-follow mode enabled ---
            if self.lf_enable.get():
                centroid = self.ir.get_centroid()  # 1–N range
                error = centroid - (len(self.ir.sensor_index) + 1) / 2  # center offset

                # proportional steering correction
                correction = self.Kp_line * error

                left_effort = self.base_effort + correction
                right_effort = self.base_effort - correction

                # Clamp and store to shares
                left_effort = max(min(left_effort, 100), -100)
                right_effort = max(min(right_effort, 100), -100)

                self.left_eff_sh.put(left_effort)
                self.right_eff_sh.put(right_effort)

            yield self.state
