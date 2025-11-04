# battery_droop.py
# ==============================================================================
# Measures the battery voltage and compensates for droop by correcting the
# nominal effort based on the measured battery voltage.
# ==============================================================================

from pyb import Pin, ADC

class Battery:
    """Reads raw battery voltage through a voltage divider and computes a
       gain (based on the measured voltage) to be used in the closed loop control to compensate the effort for battery droop."""
    
    #---------------------------------------------------------------------------
    # Constants for voltage divider
    R_1_OHM = 9.98e3   # 10k ohm (measured 9.98k ohm)
    R_2_OHM = 4.712e3  # 4.7k ohm (measured 4.712k ohm)
    V_REF = 3.3        # ADC reference voltage
    V_NOM_SINGLE = 1.5         # Nominal battery voltage for a single cell
    V_NOM_TOTAL = 6*V_NOM_SINGLE  # Total nominal battery voltage
    LOW_BATT_THRESHOLD = 6.0  # Voltage threshold for "replace batteries"
    #---------------------------------------------------------------------------
    
    def __init__(self, adc_pin):
        """Initializes the Battery object with ADC pin and voltage divider values."""
        self.adc = ADC(Pin(adc_pin))
        self.scale = (self.R_1_OHM + self.R_2_OHM) / self.R_2_OHM
        self.warned = False  # flag so we only warn "low battery" once

    #---------------------------------------------------------------------------
    def read_voltage(self):
        """Reads the raw ADC value and computes the actual battery voltage."""
        raw_adc = self.adc.read()  # Read raw ADC value (0-4095)
        V_adc = (raw_adc / 4095.0) * self.V_REF  # Measured voltage at ADC pin
        V_batt = V_adc * self.scale  # Compute actual battery voltage
        if V_batt < self.LOW_BATT_THRESHOLD and not self.warned:
            print("Warning: Battery voltage low ({:.2f} V). Consider replacing batteries.".format(V_batt))
            self.warned = True  # Set flag to avoid repeated warnings
        return V_batt
    
    #---------------------------------------------------------------------------
    def droop_gain(self):
        """Computes the droop compensation gain (V_NOM / V_batt)."""
        V_batt = self.read_voltage()
        # Avoid division by zero or very low voltage (ADC may be disconnected)
        if V_batt <= 0.5:
            print("Warning: Measured battery voltage VERY low (<0.5). Check connections, or invalid.")
            return 1.0  # Avoid division by zero, no compensation
        
        # Actually compute the droop gain
        droop_gain = self.V_NOM_TOTAL / V_batt
        return droop_gain