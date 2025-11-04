# closed_loop.py
# ==============================================================================
# PI Velocity Controller for Romi motors (in rad/s)
# ==============================================================================

from time import ticks_diff, ticks_ms

class ClosedLoop:
    """Proportional-Integral (PI) controller for velocity control in rad/s."""

    def __init__(self,
                 Kp=0.0, Ki=0.0,
                 setpoint=0.0,
                 effort_limits=(-100, 100),
                 battery=None):
        self.Kp = Kp
        self.Ki = Ki
        self.setpoint = setpoint
        self.effort_min, self.effort_max = effort_limits
        self.integrator = 0.0
        self.last_time = ticks_ms()
        self.output = 0.0

        self.battery = battery  # battery object for droop compensation

    def reset(self):
        """Reset controller integrator and state."""
        self.integrator = 0.0
        self.output = 0.0
        self.last_time = ticks_ms()

    def set_gains(self, Kp, Ki):
        """Set controller gains with validation."""
        if Kp < 0 or Ki < 0:
            raise ValueError("Controller gains must be non-negative")
        self.Kp = Kp
        self.Ki = Ki

    def set_setpoint(self, sp):
        self.setpoint = sp

    def attach_battery(self, battery_obj):
        """Attach a Battery object after initialization."""
        self.battery = battery_obj

    def run(self, feedback_rad_per_s):
        """Compute control output (effort %) from velocity feedback and measured battery voltage."""
        now = ticks_ms()
        dt_ms = ticks_diff(now, self.last_time)
        
        # Ignore first run or very long delays
        if dt_ms > 1000 or dt_ms <= 0:  # > 1 second or negative
            self.last_time = now
            return 0.0
        
        dt = dt_ms / 1000.0  # Convert to seconds
        self.last_time = now

        # Sanity check on feedback
        if abs(feedback_rad_per_s) > 100:  # Reasonable max rad/s for our motors
            print(f"Warning: Unusually high velocity: {feedback_rad_per_s} rad/s")
        
        # --- Core PI control ---
        error = self.setpoint - feedback_rad_per_s
        self.integrator += error * dt
        
        # Clamp integrator to reasonable bounds based on output limits (prevent windup)
        max_integral = (self.effort_max - self.Kp * error) / (self.Ki + 1e-6)
        min_integral = (self.effort_min - self.Kp * error) / (self.Ki + 1e-6)
        self.integrator = max(min(self.integrator, max_integral), min_integral)
        
        # Base PI output (without droop compensation)
        u = self.Kp * error + self.Ki * self.integrator

        # --- Battery droop compensation ---
        if self.battery is not None:
            gain = self.battery.droop_gain()  # Calls the battery method to compute a gain for droop compensation
        else:
            gain = 1.0  # No compensation
        
        # Compute final output with droop compensation
        u *= gain # apply droop gain block to controller output
        
        # Final clamp on output (clamp effort to safe limits)
        u = max(min(u, self.effort_max), self.effort_min)
        
        self.output = u
        return u