# ui_task.py
#
# ==============================================================================
# UITask
# ------------------------------------------------------------------------------
# Reads user input from USB serial (PuTTY), interprets commands, and sets
# control flags and motor effort shares accordingly.
# Allows the user to:
#   - set motor effort
#   - start a new step-response test (GO)
#   - stop a test mid-run (STOP)
#   - stream data (SEND)
# Users can adjust the effort between runs.
# ==============================================================================

# from pyb import USB_VCP
# from pyb import UART

# Q: Will not worry about sending commands mid-test since already checking for that using send/receive flags?

class UITask:
    """Receive user command from PC and sets flags to run and stop motors"""

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_COMMAND = 1
    S2_PROCESS_COMMAND = 2
    S3_MONITOR_TEST = 3

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 col_start, col_done, mtr_enable, stream_data,
                 uart5, abort, eff, mode,
                 time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q):
        
        # Flags
        self.col_start = col_start
        self.col_done = col_done
        self.mtr_enable = mtr_enable
        self.stream_data = stream_data
        # Abort falg
        self.abort = abort
        
        # Shares
        self.eff = eff
        self.mode = mode

        # Queues
        self.time_q = time_q
        self.left_pos_q = left_pos_q
        self.right_pos_q = right_pos_q
        self.left_vel_q = left_vel_q
        self.right_vel_q = right_vel_q

        # Serial interface (USB virtual COM port)
        # self.ser = USB_VCP()
        # Serial interface (Bluetooth port)
        self.ser = uart5

        # ensure FSM starts in state S0_INIT
        self.state = self.S0_INIT

        # # List of acceptable characters
        # self.char_list = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a"]
        # # Input to PWM value dictionary mappings
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
                self.mode.put(1)
                
                self.state = self.S1_WAIT_FOR_COMMAND

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_COMMAND):
                
                # First check if the last test finished
                if self.col_done.get():
                    self.ser.write(b'q')
                    self.col_done.put(0)

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
                # Digits or 'a' set the effort
                if cmd.isdigit() or cmd == 'a':
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
                    if self.mtr_enable.get():
                        pass
                        # Test already running
                    else:
                        self.abort.put(0)
                        self.mtr_enable.put(1)
                        self.col_start.put(1)
                        # Clear queues
                        self.time_q.clear()
                        self.left_pos_q.clear()
                        self.right_pos_q.clear()
                        self.left_vel_q.clear()
                        self.right_vel_q.clear()

                        self.state = self.S3_MONITOR_TEST

                # 'k' → KILL (STOP)
                elif cmd == 'k':
                    self.abort.put(1)
                    self.mtr_enable.put(0)
                    self.col_start.put(0)

                    # Tell PC test is testing is done
                    self.ser.write(b'q')
                    # Motors disabled and test stopped

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
                    if not self.mtr_enable.get():
                        if self.mode == 1:
                            self.mode = 2
                            continue
                        elif self.mode == 2:
                            self.mode = 3
                            continue
                        else:
                            self.mode = 1

                # Anything else → ignore, Shouldn't need to worry about other commmands handled by PC
                else:
                    pass

                self.state = self.S1_WAIT_FOR_COMMAND
                    

            ### 3: MONITOR TEST STATE ------------------------------------------
            elif self.state == self.S3_MONITOR_TEST:
                # Wait for collection or enable flag to clear
                if not self.mtr_enable.get() or not self.col_start.get():
                    # Tell PC test is testing is done
                    self.ser.write(b'q')
                    self.state = self.S1_WAIT_FOR_COMMAND

                elif self.col_done.get():
                    # Tell PC test is testing is done
                    self.ser.write(b'q')
                    self.mtr_enable.put(0)
                    self.col_start.put(0)
                    self.state = self.S1_WAIT_FOR_COMMAND
            
            yield self.state