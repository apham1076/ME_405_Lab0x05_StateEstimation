from pyb import Pin, Timer
from time import ticks_us, ticks_diff   # Use to get dt value in update()

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

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

        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = self.tim.counter() # initialize from current counter
        self.delta      = 0     # Change in count between last two updates
        self.prev_time  = ticks_us()    # Time from most recent update
        self.dt         = 0     # Amount of time between last two updates
        
    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        curr_count = self.tim.counter()
        delta = curr_count - self.prev_count

        # Check if delta is out of range
        if delta < -(self.AR + 1)/2:
            delta += self.AR + 1
        elif delta > (self.AR + 1)/2:
            delta -= self.AR + 1
        
        # Determine dt
        self.position += delta
        self.delta = delta
        curr_time = ticks_us()
        self.dt = ticks_diff(curr_time, self.prev_time)
        if self.dt != 0:
            self.velocity = self.delta*1e6 / self.dt
        else:
            self.velocity = 0
        self.prev_time = curr_time
        self.prev_count = curr_count
            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position        # position in encoder counts
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        return self.velocity
        # return self.delta*1e6 // self.dt if self.dt !=0 else 0
               # velocity in [encoder counts/second]
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position = 0
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()