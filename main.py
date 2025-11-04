# main.py

# ==============================================================================
# ME 405 Lab 0x02 - Cooperative Multitasking Framework.
# ------------------------------------------------------------------------------
# Each task runs cooperatively under Dr. JR Ridgley's cotask scheduler.
# Communication between tasks happens exclusively via task_share objects
# (Shares and Queues), never through global variables.
# ------------------------------------------------------------------------------
# Tasks:
#   UITask: handles Bluetooth serial communication from PC; sets flags and motor
#           effort commands.
#
#   MotorControlTask: reads encoder, updates motor outputs, writes latest
#                     samples (time/pos/vel) to shares.
#
#   DataCollectionTask: moves data from shares into queues to be used by StreamTask
#
#   StreamTask: buffers data stored in queues over bluetooth serial connection back to PC
# ==============================================================================

### TO-DO:
# Check period and priorty
# Use Bluetooth module to send commands from Putty and automate data collection and plotting in Python
# For plotting, save data for plots, and overlay them, check units
#

'''
Bluetooth related changes:
Commands will now be sent from keyboard through the bluetooth serial port to the bluetooth module, consider a separate task for bluetooth buffering data?

Stream task will need to be substantially changed now that all Romi-PC communication will happen via bluetooth. Fixed messages are stored in this task, but all commands (go, stop, effort) will need to be sent over bluetooth, data will be streamed across bluetooth as wells.

Data task will remain unchanged

Motor task will remain unchanged
'''

import gc
import cotask
import task_share
from pyb import Pin, UART
from motor import Motor
from encoder import Encoder
from battery_droop import Battery
from IR_sensor import IRArray
from motor_task import MotorControlTask
from data_task import DataCollectionTask
from ui_task import UITask
from stream_task import StreamTask
from steering_task import SteeringTask


def main():
    print("\r\n=== ME405 Lab 0x03 Scheduler Start ===")
    MAX_SAMPLES = 250
    
    # Hardware Setup:
    # Create motor and encoder objects
    left_motor = Motor(Pin.cpu.B1, Pin.cpu.B5, Pin.cpu.B3, 3, 4)
    right_motor = Motor(Pin.cpu.B0, Pin.cpu.C0, Pin.cpu.C1, 3, 3)
    left_encoder = Encoder(1, Pin.cpu.A8, Pin.cpu.A9)
    right_encoder = Encoder(2, Pin.cpu.A0, Pin.cpu.A1)
    battery = Battery(Pin.cpu.A4) # battery voltage measurement object
    tim6 = Timer(6, freq=1000) # timer for ADC sampling (can be reused by the IR array)
    ir_pins = [Pin.cpu.xx, Pin.cpu.xy, Pin.cpu.xz, Pin.cpu.xw, Pin.cpu.xv]  # replace xx, xy, xz, xw, xv with actual pin names
    ir_array = IRArray(ir_pins, tim6, samples=100)  # IR sensor array object
	
    # -----------------------------------------------------------------------
    ### Shared Variables: create shares and queues (inter-task communication)

    # Data Shares...
    time_sh = task_share.Share('H', name='Time share')
    left_pos_sh = task_share.Share('f', name= 'Left motor position share')
    right_pos_sh = task_share.Share('f', name= 'Right motor position share')
    left_vel_sh = task_share.Share('f', name= 'Left motor velocity share')
    right_vel_sh = task_share.Share('f', name= 'Right motor velocity share')
    
    # Motor control shares...
    eff = task_share.Share('f', name='Requested Effort')  # float effort percent
    setpoint = task_share.Share('h', name='Velocity Setpoint')  # 'h' for signed 16-bit to handle larger velocity values
    kp = task_share.Share('f', name='Proportional Gain')  # 'f' for float to store Kp
    ki = task_share.Share('f', name='Integral Gain')  # 'f' for float to store Ki
    # Initialize motor control shares
    eff.put(0)  # Start with zero effort
    setpoint.put(0)  # Start with zero setpoint
    kp.put(0)  # Start with zero gains
    ki.put(0)

    # Driving mode and control mode shares...
    driving_mode = task_share.Share('B', name='Driving Mode')  # straight line, pivot, or arc
    control_mode = task_share.Share('B', name='Control Mode')  # 0: effort mode, 1: velocity mode
    # Initialize driving and control mode shares
    driving_mode.put(1)  # Start in straight line mode
    control_mode.put(0)  # Start in effort (open-loop) mode

    # line following shares...
    lf_enable   = task_share.Share('B', name='LineFollow Enable'); lf_enable.put(0)
    left_eff_sh = task_share.Share('f', name='LF Left Effort');    left_eff_sh.put(0.0)
    right_eff_sh= task_share.Share('f', name='LF Right Effort');   right_eff_sh.put(0.0)
    ir_cmd      = task_share.Share('B', name='IR Calibrate Cmd');  ir_cmd.put(0)


    # Boolean flags
    col_start = task_share.Share('B', name='Start Collection Flag')
    col_done = task_share.Share('B', name='Collection Done Flag')
    mtr_enable = task_share.Share('B', name='Motor Enable Flag')
    stream_data = task_share.Share('B', name='Stream Data Flag')
    abort = task_share.Share('B', name='Abort Flag')

    # Data Queues...
    time_q = task_share.Queue('H', size=MAX_SAMPLES, name='Time share')
    left_pos_q = task_share.Queue('f', size=MAX_SAMPLES, name= 'Left motor position share')
    right_pos_q = task_share.Queue('f', size=MAX_SAMPLES, name= 'Right motor position share')
    left_vel_q = task_share.Queue('f', size=MAX_SAMPLES, name= 'Left motor velocity share')
    right_vel_q = task_share.Queue('f', size=MAX_SAMPLES, name= 'Right motor velocity share')
    # -----------------------------------------------------------------------

    # Create UART object
    uart5 = UART(5, 115200)

    # Create Task Objects (since tasks are classes)    
    ui_task_obj = UITask(col_start, col_done, mtr_enable, stream_data, abort,
                         eff, driving_mode, setpoint, kp, ki, control_mode,
                         uart5, battery,
                         time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q)

    motor_task_obj = MotorControlTask(left_motor, right_motor,
                                      left_encoder, right_encoder,
                                      battery,
                                      eff, mtr_enable, abort, driving_mode, setpoint, kp, ki, control_mode,
                                      time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh)
        
    data_task_obj = DataCollectionTask(col_start, col_done,
                                       mtr_enable, abort,
                                       time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q,
                                       time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh)
    
    stream_task_obj = StreamTask(eff, col_done, stream_data, uart5,
                                 time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q,
                                 control_mode, setpoint, kp, ki)
    
    steering_task_obj = SteeringTask(ir_array, battery,
                                 lf_enable, ir_cmd,
                                 left_eff_sh, right_eff_sh)


	# Create costask.Task WRAPPERS. (If trace is enabled for any task, memory will be allocated for state transition tracing, and the application will run out of memory after a while and quit. Therefore, use tracing only for debugging and set trace to False when it's not needed)
    _motor_task = cotask.Task(motor_task_obj.run, name='Motor Control Task', priority=3, period=4, profile=True, trace=False)
    
    _data_collection_task = cotask.Task(data_task_obj.run, name='Data Collection Task', priority=2, period=6, profile=True, trace=False)

    _ui_task = cotask.Task(ui_task_obj.run, name='User Interface Task', priority=0, period=40, profile=True, trace=False)

    _stream_task = cotask.Task(stream_task_obj.run, name='Stream Task', priority=1, period=20, profile=True, trace=False)

    _steering_task = cotask.Task(steering_task_obj.run,
                             name='Steering Task',
                             priority=2, period=10,
                             profile=True, trace=False)


	# Now add (append) the tasks to the scheduler list
    cotask.task_list.append(_motor_task)
    cotask.task_list.append(_data_collection_task)
    cotask.task_list.append(_ui_task)
    cotask.task_list.append(_stream_task)
    cotask.task_list.append(_steering_task)

    ### The scheduler is ready to start ###

	# Run the memory garbage collector to ensure memory is as defragmented as possible before the real-time scheduler is started
    gc.collect()
    
	# Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    # print("Scheduler running... Press Ctrl-C to halt.\r\n")
    while True: # run inifinite iterations of the scheduler
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break

    # Diagnostics on exit: print a table of task data and a table of shared information data
    print("\n=== Scheduler Halted ===")
    print('\n' + str(cotask.task_list))
    # print(task_share.show_all())
    # print(_ui_task.get_trace())
    # print(_motor_task.get_trace())
    # print(_data_collection_task.get_trace())
    # print(_stream_task.get_trace())

    
if __name__ == "__main__":
	main()