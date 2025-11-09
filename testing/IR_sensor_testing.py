# IR Sensor Testing Script

import array
from pyb import Pin, ADC, Timer

tim = Timer(6, freq=10)

# Initialize the IR sensor pin (assuming it's connected to pin X1)
ir_pin1 = Pin(Pin.cpu.PC2, mode=Pin.IN)
ir_pin2 = Pin(Pin.cpu.PC3, mode=Pin.IN)
ir_pin3 = Pin(Pin.cpu.PC4, mode=Pin.IN)
ir_pin4 = Pin(Pin.cpu.PC5, mode=Pin.IN)

# Initialize ADC for analog IR sensor (assuming it's connected to pin X2)
adc1 = ADC(ir_pin1)
adc2 = ADC(ir_pin2)
adc3 = ADC(ir_pin3)
adc4 = ADC(ir_pin4)


# buf = bytearray(200)
buf = array.array('H', [0] * 100)
adc1.read_timed(buf, tim)

