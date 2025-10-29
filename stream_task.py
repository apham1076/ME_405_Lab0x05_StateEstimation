# stream_task.py
#
# ==============================================================================
# SteamTask (BLUETOOTH version)
# ------------------------------------------------------------------------------
# Waits for a stream command (stream_data flag) and the data collection
# completion (col_done flag). Then, buffer data in CSV format over Bluetooth one
# line at a time.
# ==============================================================================


import gc # garbage collector

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
                 eff, col_done, stream_data, uart5,
                 time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q):

         # Serial interface (Bluetooth UART)
        self.ser = uart5

        # Flags
        self.col_done = col_done
        self.stream_data = stream_data
        self.eff = eff
        
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
                # # Print header
                # self.ser.write("time_ms,pos_L,pos_R,vel_L,vel_R\r\n".encode())
                # yield self.state 
                
                # Get number of items in queue to determine size
                _size = self.time_q.num_in()
                _eff = self.eff.get()
                line = f"{_eff},{_size}\r\n"
                self.ser.write(line.encode())

                yield self.state

                # Wait for READY from PC


                # while we still have items in queue
                while self.time_q.any():
                    # Get items from the queue
                    t = self.time_q.get()
                    pL = self.left_pos_q.get()
                    pR = self.right_pos_q.get()
                    vL = self.left_vel_q.get()
                    vR = self.right_vel_q.get()
                    line = f"{t},{pL},{pR},{vL},{vR}\r\n"
                    self.ser.write(line.encode())
                    yield self.state

                else:
                    # Tell PC streaming is done
                    # self.ser.write(b'r')
                    self.stream_data.put(0)
                    self.col_done.put(0) # reset done flag for next test

                    gc.collect() # run garbage collector
                    self.state = self.S1_WAIT_FOR_TRIGGER # set next state

            yield self.state