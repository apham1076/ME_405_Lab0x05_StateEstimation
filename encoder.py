from pyb import Pin, Timer
from time import ticks_us, ticks_diff   # Use to get dt value in update()

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class. Provides position in [rad] and velocity in [rad/s].'''

    # --------------------------------------------------------------------------
    # CONSTANTS
    CPR = 1440            # Counts (pulses) per revolution of the encoder
    RAD_PER_COUNT = 2*3.14159265 / CPR  # Radians per count
    # --------------------------------------------------------------------------

    def __init__(self,
                 tim_num: int,
                 chA_pin: Pin,
                 chB_pin: Pin):
        '''Initializes an Encoder object'''
        self.AR = 65535         # Auto-reload value for 16-bit timer
        self.tim = Timer(tim_num, prescaler=0, period=self.AR) # freq is ignored
        self.chA_pin = Pin(chA_pin)
        self.chB_pin = Pin(chB_pin)
        self.tim.channel(1, mode=Timer.ENC_AB, pin=self.chA_pin)
        self.tim.channel(2, mode=Timer.ENC_AB, pin=self.chB_pin)

        # Internal states
        self.position_counts   = 0     # Total accumulated position counts
        self.prev_count = self.tim.counter() # initialize from current counter
        self.delta = 0      # Change in count between last two updates
        self.prev_time = ticks_us()    # Time from most recent update
        self.dt         = 0     # Amount of time between last two updates
        self.velocity_counts_per_s = 0  # Velocity in counts per second
        
    # --------------------------------------------------------------------------
    def update(self):
        '''Update encoder count and compute velocity.'''
        curr_count = self.tim.counter()
        delta = curr_count - self.prev_count

        # Check if delta is out of range (handle 16-bit over/underflow)
        if delta < -(self.AR + 1)/2:
            delta += self.AR + 1
        elif delta > (self.AR + 1)/2:
            delta -= self.AR + 1
        
        # Update count-based position
        self.position_counts += delta
        self.delta = delta

        # Compute time step and velocity
        curr_time = ticks_us()
        self.dt = ticks_diff(curr_time, self.prev_time)
        if self.dt != 0:
            self.velocity_counts_per_s = self.delta*1e6 / self.dt
        else:
            self.velocity_counts_per_s = 0

        # Save for next update
        self.prev_count = curr_count
        self.prev_time = curr_time

    # --------------------------------------------------------------------------
    def get_position(self):
        '''Return position in radians.'''
        return self.position_counts * self.RAD_PER_COUNT  # position in [rad]

    def get_velocity(self):
        '''Return velocity in radians per second.'''
        return self.velocity_counts_per_s * self.RAD_PER_COUNT  # velocity in [rad/s]

    def zero(self):
        '''Zero the encoder position.'''
        self.position_counts = 0
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()