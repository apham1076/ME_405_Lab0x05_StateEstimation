# stream_task.py
#
# ==============================================================================
# StreamTask (BLUETOOTH version)
# ------------------------------------------------------------------------------
# Waits for a stream command (stream_data flag) and the data collection
# completion (col_done flag). Then, buffer data in CSV format over Bluetooth one
# line at a time.
# ==============================================================================


import gc # garbage collector
import time

class StreamTask:
    """Streams data in CSV format over Bluetooth serial port to PC."""

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_TRIGGER = 1
    S2_STREAM_DATA = 2
    S3_DONE = 3

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 eff, col_done, stream_data, uart,
                 time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q,
                 control_mode, setpoint, kp, ki, k_line, lf_target):

         # Serial interface (Bluetooth UART)
        self.ser = uart

        # Flags
        self.col_done = col_done
        self.stream_data = stream_data
        
        # Control parameters
        self.eff = eff
        self.control_mode = control_mode
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.k_line = k_line
        self.lf_target = lf_target

        # Queues
        self.time_q = time_q
        self.left_pos_q = left_pos_q
        self.right_pos_q = right_pos_q
        self.left_vel_q = left_vel_q
        self.right_vel_q = right_vel_q
        
       
        # ensure FSM starts in state S0_INIT
        self.state = self.S0_INIT

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        """Finite state machine for data streaming."""
        while True: # run infinite iterations of the state machine
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                # Clear stream flags
                self.stream_data.put(0)
                self.state = self.S1_WAIT_FOR_TRIGGER # set next state

            ### 1: WAIT FOR TRIGGER STATE --------------------------------------
            elif (self.state == self.S1_WAIT_FOR_TRIGGER):
                if self.stream_data.get():
                    self.state = self.S2_STREAM_DATA # set next state
                
            ### 2: STREAM DATA STATE -------------------------------------------
            elif (self.state == self.S2_STREAM_DATA):

                # # Get control mode info to send to PC
                # mode_val = int(self.control_mode.get())
                # if mode_val == 0:
                #     mode = "E"  # effort mode
                #     control_val = self.eff.get()
                # elif mode_val == 1:
                #     mode = "V"  # velocity mode
                #     control_val = self.setpoint.get()
                # else:
                #     mode = "L"  # line follow mode
                #     control_val = self.lf_target.get()

                # # Determine the number of samples actually in the queues
                # _size = self.time_q.num_in()

                # # Send a line with the control mode and parameters
                # if mode == "E":
                #     line = f"{mode},{control_val},{_size}\r\n"
                # elif mode == "V":
                #     line = f"{mode},{control_val},{self.kp.get():.2f},{self.ki.get():.2f},{_size}\r\n"
                # else:
                #     line = f"{mode},{control_val},{self.kp.get():.2f},{self.ki.get():.2f},{self.k_line.get():.2f},{_size}\r\n"

                # self.ser.write(line.encode()) # write the line over Bluetooth
                # yield self.state # write only one line then yield

                # while we still have items in queue
                index = 0 # sample index number (sent to PC for alignment)
                while self.time_q.any():
                    # Get items from the queues
                    t = self.time_q.get()
                    pL = self.left_pos_q.get()
                    pR = self.right_pos_q.get()
                    vL = self.left_vel_q.get()
                    vR = self.right_vel_q.get()
                    # Put it all into a CSV-style line stamped with the index
                    data = f"{index},{t},{pL},{pR},{vL},{vR}\r\n"
                    # Send the line over Bluetooth
                    self.ser.write(data.encode())
                    # Increment the sample index
                    index += 1
                    # Only write one line then yield
                    yield self.state

                # Once all samples have been sent, mark the end of the stream
                self.ser.write(b"#END\r\n") # explicit end marker for PC

                # Do NOT block waiting for ACK here; ack handling will be done in UI task
                # Reset flags
                self.stream_data.put(0)
                self.col_done.put(0) # reset done flag for next test

                gc.collect() # run garbage collector
                self.state = self.S1_WAIT_FOR_TRIGGER # go back to wait state

            yield self.state