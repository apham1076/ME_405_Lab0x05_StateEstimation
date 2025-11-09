# Script to test IMU module functionality

from pyb import Pin, I2C
# from IMU_sensor import IMU

# Initialize and configure pins
Left_nSLP = Pin('PB3', mode=Pin.OUT_PP)
Right_nSLP = Pin('PC9', mode=Pin.OUT_PP)
# sda = Pin('PB14', mode=Pin.ALT, alt=4)
# scl = Pin('PB13', mode=Pin.ALT, alt=4)
# reset = Pin('PB15', mode=Pin.OUT_PP)

# Initialize I2C interface
# i2c = I2C(2, mode=I2C.CONTROLLER, baudrate=400000)

# Ensure motors are disabled
def disable_motors():
    Left_nSLP.low()
    Right_nSLP.low()
    print("Motors disabled.")

disable_motors()

