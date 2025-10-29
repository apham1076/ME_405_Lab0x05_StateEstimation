from pyb import Pin, Timer

class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''
    
    def __init__(self,
                 PWM_pin: Pin,
                 DIR_pin: Pin,
                 nSLP_pin: Pin,
                 tim: int,
                 chan: int):
        '''Initializes a Motor object'''
        self.PWM_pin = Pin(PWM_pin, mode=Pin.OUT_PP, value=0)
        self.DIR_pin = Pin(DIR_pin, mode=Pin.OUT_PP, value=0)
        self.nSLP_pin = Pin(nSLP_pin, mode=Pin.OUT_PP, value=0)
        self.tim_obj = Timer(tim, freq=20000)
        self.chan_obj = self.tim_obj.channel(chan, pin=self.PWM_pin, mode=Timer.PWM, pulse_width_percent = 0)
        self.effort = 0 # match effort to initialized PWM percent (0)
        self.enabled = False # initialize the "enabled" flag as LOW
    
    def set_effort(self, effort: float):
        '''Sets the present effort requested from the motor based on an input value between -100 and 100'''
        
        ### if motor not enabled, don't change the pulse_width_percent yet
        if not self.enabled: # effort not set/stored while motor disabled
            return
        
        ### motor must already be enabled, so it's okay to set effort
        self.effort = max(min(effort,100),-100) # clamp effort b/w -100 and 100

        if self.effort >= 0: # effort is >=0, and already clamped to max 100
            self.DIR_pin.low() # set motor to forward direction
            self.chan_obj.pulse_width_percent(self.effort) # set PWM
        else: # effort is <0, and already clamped to min -100
            self.DIR_pin.high() # set motor to reverse direction
            self.chan_obj.pulse_width_percent(-self.effort) # set PWM

    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''

        if self.enabled: # if motor already enabled
            return       # don't change anything
        
        ### otherwise, motor was disabled
        self.chan_obj.pulse_width_percent(0) # ensure it doesn't start moving
        self.effort = 0                      # change effort to match PWM %
        self.nSLP_pin.high()                 # enable motor
        self.enabled = True                  # update status to enabled
            
    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
 
        if not self.enabled: # if motor already disabled
            return           # don't change anything
        
        ### otherwise, motor was enabled
        self.chan_obj.pulse_width_percent(0) # stop moving motor
        self.effort = 0                      # change effort to match PWM %
        self.nSLP_pin.low()                  # disable motor
        self.enabled = False                 # update status to disabled

    def get_state(self):
        '''Report values for motor state'''
       
        if self.DIR_pin.value() == 0:
            print("Motor is set in forward direction.")
        else:
            print("Motor is set in reverse direction.")
        if self.nSLP_pin.value() == 0:
            print("Motor is disabled.")
        else:
            print("Motor is enabled.")
        print(f"Motor effort is {self.effort}")

### Commands for controlling the motors from the REPL ###
# motor_left = Motor(Pin.cpu.B1, Pin.cpu.B5, Pin.cpu.B3, 3, 4)
# motor_left.enable()
# motor_left.disable()
# motor_left.set_effort(50)
# motor_left.get_state()
# motor_right = Motor(Pin.cpu.B0, Pin.cpu.C0, Pin.cpu.C1, 3, 3)
# motor_right.enable()
# motor_right.disable()
# motor_right.set_effort(50)
# motor_right.get_state()