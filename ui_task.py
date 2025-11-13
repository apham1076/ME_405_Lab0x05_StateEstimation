# ui_task.py
#
# ==============================================================================
# UITask
# ------------------------------------------------------------------------------
# Reads user input from Bluetooth UART5 (PC test.py), interprets commands, and
# sets control flags and motor effort shares accordingly.
# Allows the user to:
#   - set motor effort
#   - start a new step-response test (GO)
#   - stop a test mid-run (STOP)
#   - stream data (SEND)
# Users can adjust the effort between runs.
# ==============================================================================

# from pyb import USB_VCP
# from pyb import UART

class UITask:
    """Reads user input from Bluetooth UART (UART5), interprets commands, and sets. Reads user input from the Bluetooth UART (UART5). Commands are sent from PC via test.py (VS Code terminal)."""

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_COMMAND = 1
    S2_PROCESS_COMMAND = 2
    S3_MONITOR_TEST = 3

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 col_start, col_done, mtr_enable, stream_data, abort,
                 eff, driving_mode, setpoint, kp, ki, control_mode,
                 uart5, battery,
                 time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q,
                 ir_cmd,
                 k_line, lf_target):
        
        # Flags
        self.col_start = col_start
        self.col_done = col_done
        self.mtr_enable = mtr_enable
        self.stream_data = stream_data
        self.abort = abort
        
        # Shares
        self.eff = eff
        self.driving_mode = driving_mode
        self.setpoint = setpoint  # Share for velocity setpoint
        self.kp = kp  # Share for proportional gain
        self.ki = ki  # Share for integral gain
        self.k_line = k_line  # Share for line following K_line gain
        self.lf_target = lf_target  # Share for line following target velocity
        self.control_mode = control_mode  # Share for control mode (effort/velocity/line-follow)
        self.ir_cmd = ir_cmd  # Share for IR command

        # Serial interface (USB virtual COM port)
        # self.ser = USB_VCP()
        # Serial interface (Bluetooth port)
        self.ser = uart5
        # Battery object
        self.battery = battery

        # Queues
        self.time_q = time_q
        self.left_pos_q = left_pos_q
        self.right_pos_q = right_pos_q
        self.left_vel_q = left_vel_q
        self.right_vel_q = right_vel_q

        # ensure FSM starts in state S0_INIT
        self.state = self.S0_INIT


        # self.char_dict = {"0": 0,
        #                 "1": 10,
        #                 "2": 20,
        #                 "3": 30,
        #                 "4": 40,
        #                 "5": 50,
        #                 "6": 60,
        #                 "7": 70,
        #                 "8": 80,
        #                 "9": 90,
        #                 "a": 100}


    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        """Generator that checks PuTTY for commands and updates flags"""
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                self.col_start.put(0)
                self.col_done.put(0)
                self.last_eff = 0
                self.abort.put(0)
                self.driving_mode.put(1) # default to straight line mode
                
                self.state = self.S1_WAIT_FOR_COMMAND

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_COMMAND):
                
                # First check if the last test finished
                # if self.col_done.get():
                #     self.ser.write(b'q')
                #     self.col_done.put(0)

                # Wait for user input
                if self.ser.any():
                    ch = self.ser.read(1).decode().lower() # read 1 char AAT
                    self.cmd_buf = ch
                    # Romi will send, PC's turn to receive
                    # self.ser.write(b's')

                    self.state = self.S2_PROCESS_COMMAND # set next state
            
            ### 2: PROCESS COMMAND STATE ---------------------------------------
            elif self.state == self.S2_PROCESS_COMMAND:
                cmd = self.cmd_buf
                # Velocity setpoint message: 'y' + 4 digits for >= 0, or 'z' + 4 digits for < 0
                if cmd in ['y', 'z'] and self.ser.any() >= 4:
                    # Read the 4 digits for setpoint value
                    value_str = self.ser.read(4).decode()
                    try:
                        value = int(value_str)
                        if cmd == 'z':
                            value = -value
                        if self.mtr_enable.get():
                            pass  # Cannot change setpoint mid-test
                        else:
                            self.setpoint.put(value)
                            print("Setpoint value set to:", value)
                    except ValueError:
                        pass  # Invalid setpoint format
                
                # Digits or 'a' set the effort (open-loop control)
                elif cmd.isdigit() or cmd == 'a':
                    val = 10 * (int(cmd) if cmd.isdigit() else 10)  # 0-9 → 0–90%, a→100%
                    if self.mtr_enable.get():
                        pass
                        # Cannot change effort mid-test
                    else:
                        self.last_eff = val
                        self.eff.put(val)
                        # Ready to go

                # 'g' → GO
                elif cmd == 'g':
                    if not self.mtr_enable.get():
                        print("Starting run")
                        self.abort.put(0)
                        self.mtr_enable.put(1)

                        if self.control_mode.get() in (0, 1):
                            self.col_start.put(1)  # start data collection only for open-loop or velocity control
                            # Clear queues
                            self.time_q.clear()
                            self.left_pos_q.clear()
                            self.right_pos_q.clear()
                            self.left_vel_q.clear()
                            self.right_vel_q.clear()
                            # Change state to monitor test
                            self.state = self.S3_MONITOR_TEST
                        else:
                            # Line-following mode does not collect data (runs indefinitely until stopped)
                            self.col_start.put(0)
                            # remain in WAIT state, don't auto-stop
                            print("Line-follow mode active; running indefinitely.")

                # 'p' → Set Kp (when followed by 4 digits)
                elif cmd == 'p' and self.ser.any() >= 4:
                    # Read the 4 digits for Kp value
                    value_str = self.ser.read(4).decode()
                    try:
                        kp_int = int(value_str)
                        kp = kp_int / 100.0  # Scale back down from integer
                        if not self.mtr_enable.get():
                            self.kp.put(kp)
                        print("Kp received:", kp)
                    except ValueError:
                        pass  # Invalid Kp format

                # 't' → Set target/setpoint (when followed by 4 digits)
                elif cmd == 't' and self.ser.any() >= 4:
                    # Read the 4 digits for setpoint value
                    value_str = self.ser.read(4).decode()
                    try:
                        setpoint = int(value_str)
                        print("Setpoint received:")
                        print(setpoint)
                        if not self.mtr_enable.get():
                            self.setpoint.put(setpoint)
                    except ValueError:
                        pass  # Invalid setpoint format

                # 'k' → KILL (STOP)
                elif cmd == 'k':
                    print("Stopping test")
                    self.abort.put(1)  # Set abort flag first
                    self.mtr_enable.put(0)  # Then disable motors
                    self.col_start.put(0)  # Stop data collection
                    self.ser.write(b'q')  # Tell PC test is done

                # 'i' → Set Ki (when followed by 4 digits)
                elif cmd == 'i' and self.ser.any() >= 4:
                    # Read the 4 digits for Ki value
                    value_str = self.ser.read(4).decode()
                    try:
                        ki_int = int(value_str)
                        ki = ki_int / 100.0  # Scale back down from integer
                        if not self.mtr_enable.get():
                            self.ki.put(ki)
                        print("Ki received:", ki)
                    except ValueError:
                        pass  # Invalid Ki format

                # 'e' → Toggle control mode
                elif cmd == 'e':
                    if not self.mtr_enable.get():  # Only allow mode change when motors are off
                        current_mode = self.control_mode.get()
                        new_mode = (current_mode + 1) % 3  # Cycle through 0, 1, 2
                        self.control_mode.put(new_mode)
                        print("Switching control mode to:", "Velocity" if new_mode == 1 else "Effort" if new_mode == 0 else "Line Following")

                
                # 's' → STREAM
                elif cmd == 's':
                    self.stream_data.put(1)

                    # if self.mtr_enable.get() or self.col_start.get():
                    #     pass
                    #     # Cannot send data while test is running
                    # else:
                    #     self.stream_data.put(1)

                # 'm' → TOGGLE MODE
                elif cmd == 'm':
                    # Toggle driving mode (1 = straight, 2 = pivot, 3 = arc)
                    if not self.mtr_enable.get():            # only change when stopped
                        current_mode = self.driving_mode.get() or 1

                        if current_mode == 1:
                            new_mode = 2
                            print("Driving mode set to: Pivot")
                        elif current_mode == 2:
                            new_mode = 3
                            print("Driving mode set to: Arc")
                        else:
                            new_mode = 1
                            print("Driving mode set to: Straight")

                        self.driving_mode.put(new_mode)
                
                # 'v' → print current battery voltage
                elif cmd == 'v':
                    v_batt = self.battery.read_voltage()
                    voltage_msg = f"{v_batt:.2f}\n"
                    self.ser.write(voltage_msg.encode()) # send over bluetooth UART
                
                elif cmd == 'w':   # Calibrate on WHITE background
                    if self.ir_cmd: self.ir_cmd.put(1)

                elif cmd == 'b':   # Calibrate on BLACK line
                    if self.ir_cmd: self.ir_cmd.put(2)

                elif cmd == 'l' and self.ser.any() >= 16:
                    # Set gains for line-following
                    try:
                        # Read 4 digits for Kp and 4 digits for Ki and 4 digits for setpoint
                        kp_str = self.ser.read(4).decode()
                        ki_str = self.ser.read(4).decode()
                        kline_str = self.ser.read(4).decode()
                        target_str = self.ser.read(4).decode()
                        kp = int(kp_str) / 100.0
                        ki = int(ki_str) / 100.0
                        k_line = int(kline_str) / 100.0
                        v_target = int(target_str) / 100.0

                        self.kp.put(kp)
                        self.ki.put(ki)
                        self.k_line.put(k_line)
                        self.lf_target.put(v_target)
                        self.control_mode.put(2)  # switch to line-following mode
                        print(f"Line-following params set: Kp={kp}, Ki={ki}, K_line={k_line}, Target={v_target} (Mode=Line-Follow)")
                    except ValueError:
                        print("Invalid line-follow parameter format")

                # Anything else → ignore, Shouldn't need to worry about other commmands handled by PC
                else:
                    pass

                if self.state == self.S2_PROCESS_COMMAND:
                    self.state = self.S1_WAIT_FOR_COMMAND

            ### 3: MONITOR TEST STATE ------------------------------------------
            elif self.state == self.S3_MONITOR_TEST:
                # Check for abort signal first
                if self.ser.any():
                    ch = self.ser.read(1).decode().lower() # read 1 char AAT
                    if ch == 'k':
                        print("Stopping test")
                        self.abort.put(1)  # Set abort flag first
                        self.mtr_enable.put(0)  # Then disable motors
                        self.col_start.put(0)  # Stop data collection
                        self.ser.write(b'q')  # Tell PC test is done
                        self.state = self.S1_WAIT_FOR_COMMAND
                        yield self.state

                        # Clear any other junk that might be in the buffer
                        while self.ser.any():
                            self.ser.read(1)

                # Then, check for normal completion if no kill command
                elif not self.mtr_enable.get() or not self.col_start.get() or self.col_done.get():
                    # Tell PC test is testing is done
                    self.ser.write(b'q')
                    # print("Test completed")
                    self.abort.put(1)
                    self.mtr_enable.put(0)
                    self.col_start.put(0)
                    self.col_done.put(0)
                    self.state = self.S1_WAIT_FOR_COMMAND

            yield self.state