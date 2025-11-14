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
                 k_line, lf_target,
                 ack_end=None):
        
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

        # Optional share for ACK_END from PC (stream end acknowledgement)
        self.ack_end = ack_end


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
                self.ack_end.put(0)
                
                self.state = self.S1_WAIT_FOR_COMMAND

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_COMMAND):

                # Wait for user input (read available bytes non-blocking)
                if self.ser.any():
                    ch = self.ser.read(1).decode()
                    self.cmd_buf = ch
                    # try:
                    #     raw = self.ser.read(self.ser.any())
                    #     text = raw.decode()
                    #     print("Received text:", text)
                    # except Exception:
                    #     # Fallback to single-char read
                    #     try:
                    #         text = self.ser.read(1).decode()
                    #     except Exception:
                    #         text = ''

                    # if not text:
                    #     continue

                    # # Normalize and handle multi-byte messages like ACK_END
                    # lower = text.lower()
                    # if 'ack_end' in lower and self.ack_end:
                    #     try:
                    #         self.ack_end.put(1)
                    #     except Exception:
                    #         pass
                    #     # remove ack_end token from text
                    #     print("ACK_END received")
                    #     lower = lower.replace('ack_end', '')
                    #     text = ''

                    # # Find first non-whitespace character to use as single-char command
                    # first_char = None
                    # for c in lower:
                    #     if not c.isspace():
                    #         first_char = c
                    #         break
                    # if first_char:
                    #     self.cmd_buf = first_char
                    #     print("first_char:", first_char)
                    #     # Romi will send, PC's turn to receive
                    #     # self.ser.write(b's')
                    #     self.state = self.S2_PROCESS_COMMAND # set next state
                    # else:
                    #     # No command found; remain waiting
                    #     pass
                    
                    self.state = self.S2_PROCESS_COMMAND # set next state
            
            ### 2: PROCESS COMMAND STATE ---------------------------------------
            elif self.state == self.S2_PROCESS_COMMAND:
                cmd = self.cmd_buf

                if cmd == 'e' and self.ser.any() >= 1:
                    # Effort command: 'e' + 2 digits (00 to 99) or 'e' + 'a' for 100%
                    value_str = self.ser.read(1).decode()

                    if value_str.isdigit() or value_str == 'a':
                        val = 10 * (int(value_str) if value_str.isdigit() else 10)  # '00'-'99' → 0–99%, 'a'→100%
                        self.last_eff = val
                        self.eff.put(val)
                        self.control_mode.put(0)  # switch to effort control mode
                        print("Effort value set to:", val)
                    else:
                        print("Invalid effort command format")

                if cmd == 'v' and self.ser.any() >= 12:
                    # Velocity command: 'v' + 4 digits for setpoint + 4 digits for Kp + 4 digits for Ki
                    value_str = self.ser.read(12).decode()
                    try:
                        sp_str = value_str[0:4]
                        kp_str = value_str[4:8]
                        ki_str = value_str[8:12]
                        setpoint = int(sp_str)
                        kp_int = int(kp_str)
                        ki_int = int(ki_str)
                        kp = kp_int / 100.0
                        ki = ki_int / 100.0
                        self.setpoint.put(setpoint)
                        self.kp.put(kp)
                        self.ki.put(ki)
                        self.control_mode.put(1)  # switch to velocity control mode
                        print("Velocity control params set: Setpoint =", setpoint, "Kp =", kp, "Ki =", ki)
                    except ValueError:
                        print("Invalid velocity command format")

                if cmd == 'l' and self.ser.any() >= 16:
                    # Line-following command: 'l' + 4 digits for Kp + 4 digits for Ki + 4 digits for K_line + 4 digits for target velocity
                    value_str = self.ser.read(16).decode()
                    try:
                        kp_str = value_str[0:4]
                        ki_str = value_str[4:8]
                        kline_str = value_str[8:12]
                        target_str = value_str[12:16]
                        kp_int = int(kp_str)
                        ki_int = int(ki_str)
                        k_line_int = int(kline_str)
                        v_target_int = int(target_str)
                        kp = kp_int / 100.0
                        ki = ki_int / 100.0
                        k_line = k_line_int / 100.0
                        v_target = v_target_int / 100.0
                        self.kp.put(kp)
                        self.ki.put(ki)
                        self.k_line.put(k_line)
                        self.lf_target.put(v_target)
                        self.control_mode.put(2)  # switch to line-following control mode
                        print("Line-following params set: Kp =", kp, "Ki =", ki, "K_line =", k_line, "Target Velocity =", v_target)
                    except ValueError:
                        print("Invalid line-following command format")
                    

                # 'r' → Run test
                if cmd == 'r':
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

                # 'k' → KILL (STOP)
                elif cmd == 'k':
                    print("Stopping test")
                    self.abort.put(1)  # Set abort flag first
                    self.mtr_enable.put(0)  # Then disable motors
                    self.col_start.put(0)  # Stop data collection
                    self.ser.write(b'q')  # Tell PC test is done

                # 's' → STREAM
                elif cmd == 's':
                    self.stream_data.put(1)

                # 'n' → TOGGLE MODE
                elif cmd == 'n':
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
                elif cmd == 'V':
                    v_batt = self.battery.read_voltage()
                    voltage_msg = f"{v_batt:.2f}\n"
                    self.ser.write(voltage_msg.encode()) # send over bluetooth UART
                
                elif cmd == 'w':   # Calibrate on WHITE background
                    if self.ir_cmd: self.ir_cmd.put(1)

                elif cmd == 'b':   # Calibrate on BLACK line
                    if self.ir_cmd: self.ir_cmd.put(2)

                else:
                    pass

                if self.state == self.S2_PROCESS_COMMAND:
                    self.state = self.S1_WAIT_FOR_COMMAND

            ### 3: MONITOR TEST STATE ------------------------------------------
            elif self.state == self.S3_MONITOR_TEST:
                # Check for abort signal first
                if self.ser.any():
                    try:
                        raw = self.ser.read(self.ser.any())
                        text = raw.decode()
                    except Exception:
                        try:
                            text = self.ser.read(1).decode()
                        except Exception:
                            text = ''

                    if not text:
                        pass
                    else:
                        lower = text.lower()
                        # Handle ACK_END from PC (stream end acknowledgement)
                        if 'ack_end' in lower and self.ack_end:
                            try:
                                self.ack_end.put(1)
                            except Exception:
                                pass
                            # remove ack token for further processing
                            lower = lower.replace('ack_end', '')

                        # If a 'k' kill command is present, honor it
                        if 'k' in lower:
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