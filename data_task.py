# data_task.py
#
# ==============================================================================
# DataCollectionTask
# ------------------------------------------------------------------------------
# This task logs samples (time, pos, and vel) from shares into arrays for CSV
# export. Since we are creating an object (an instance of the class) of this
# class, the object lives in memory as a single instance. We can then pass the
# object to something else, like another task (class object). This allows the
# other task (class object) to access the same actual instance (object) of this
# class. Thus, we can referrence the same arrays created in this task as long
# as we pass the class object to any other tasks where we want to reference the
# arrays.
#
# Alternatively, data can be stored into queues for continuous streaming of
# data. Since we only need to send the data once (after the test is done),
# using queues to transfer the data isn't necessary. None of the tasks for this
# assignment need every sample in real time; we are mostly interested in a
# final log of the results after recording.
# ==============================================================================

### TO-DO: In state 2, consider setting col_done flag when buffers are full

from array import array

class DataCollectionTask:
    """Records motor data, moves values from shares and into queues."""

    ### The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_START_COLLECTING = 1
    S2_COLLECTING_DATA = 2
    S3_COLLECTION_DONE = 3

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 col_start, col_done,
                 mtr_enable, abort,
                 time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q,
                 time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh):

        # Flags
        self.col_start = col_start
        self.col_done = col_done
        self.mtr_enable = mtr_enable
        self.abort = abort

        # Queues
        self.time_q = time_q
        self.left_pos_q = left_pos_q
        self.right_pos_q = right_pos_q
        self.left_vel_q = left_vel_q
        self.right_vel_q = right_vel_q

        # Shares
        self.time_sh = time_sh
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh

        # ensure FSM starts in state S0_INIT
        self.state = self.S0_INIT

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):

                # Clear queues
                self.time_q.clear()
                self.left_pos_q.clear()
                self.right_pos_q.clear()
                self.left_vel_q.clear()
                self.right_vel_q.clear()

                self.state = self.S1_WAIT_FOR_START_COLLECTING # set next state

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_START_COLLECTING):
                if self.col_start.get():
                    self.state = self.S2_COLLECTING_DATA # set next state

            ### 2: COLLECTING STATE --------------------------------------------
            elif (self.state == self.S2_COLLECTING_DATA):
                # Check if sample queue is full
                if not self.time_q.full() and not self.abort.get():
                    self.time_q.put(self.time_sh.get())
                    self.left_pos_q.put(self.left_pos_sh.get())
                    self.right_pos_q.put(self.right_pos_sh.get())
                    self.left_vel_q.put(self.left_vel_sh.get())
                    self.right_vel_q.put(self.right_vel_sh.get())
                
                else:
                    # Set flags
                    self.col_start.put(0)
                    self.col_done.put(1)
                    self.mtr_enable.put(0)
                    self.state = self.S3_COLLECTION_DONE
            
            ### 3: DONE STATE --------------------------------------------------
            elif (self.state == self.S3_COLLECTION_DONE):
                # Wait for col_start to be cleared
                if not self.col_start.get():
                    self.state = self.S1_WAIT_FOR_START_COLLECTING

            yield self.state