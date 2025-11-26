# data_task.py
#
# ==============================================================================
# DataCollectionTask
# ------------------------------------------------------------------------------
# This task logs samples (time, pos, and vel) from shares into queues for data 
# logging and plotting.
# ==============================================================================

### TO-DO: In state 2, consider setting col_done flag when buffers are full


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
                 time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q, obsv_time_q, obsv_sL_q, obsv_sR_q, obsv_psi_q, obsv_psi_dot_q,
                 time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh, obsv_time_sh, obsv_sL_sh, obsv_sR_sh, obsv_psi_sh, obsv_psi_dot_sh):

        # Flags
        self.col_start = col_start
        self.col_done = col_done
        self.mtr_enable = mtr_enable
        self.abort = abort

        # Shares
        self.time_sh = time_sh
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh

        self.obsv_time_sh = obsv_time_sh
        self.obsv_sL_sh = obsv_sL_sh
        self.obsv_sR_sh = obsv_sR_sh
        self.obsv_psi_sh = obsv_psi_sh
        self.obsv_psi_dot_sh = obsv_psi_dot_sh

        # Queues
        self.time_q = time_q
        self.left_pos_q = left_pos_q
        self.right_pos_q = right_pos_q
        self.left_vel_q = left_vel_q
        self.right_vel_q = right_vel_q

        self.obsv_time_q = obsv_time_q
        self.obsv_sL_q = obsv_sL_q
        self.obsv_sR_q = obsv_sR_q
        self.obsv_psi_q = obsv_psi_q
        self.obsv_psi_dot_q = obsv_psi_dot_q

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

                self.obsv_time_q.clear()
                self.obsv_sL_q.clear()
                self.obsv_sR_q.clear()
                self.obsv_psi_q.clear()
                self.obsv_psi_dot_q.clear()

                self.state = self.S1_WAIT_FOR_START_COLLECTING # set next state

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_START_COLLECTING):
                if self.col_start.get():
                    self.state = self.S2_COLLECTING_DATA # set next state

            ### 2: COLLECTING STATE --------------------------------------------
            elif (self.state == self.S2_COLLECTING_DATA):
                # print("Collecting data...")
                
                # Check if sample queue is full
                if not self.time_q.full() and not self.abort.get():    
                    # Move data from shares into queues
                    self.time_q.put(self.time_sh.get())
                    self.left_pos_q.put(self.left_pos_sh.get())
                    self.right_pos_q.put(self.right_pos_sh.get())
                    self.left_vel_q.put(self.left_vel_sh.get())
                    self.right_vel_q.put(self.right_vel_sh.get())

                if not self.obsv_time_q.full() and not self.abort.get():
                    self.obsv_time_q.put(self.obsv_time_sh.get())
                    self.obsv_sL_q.put(self.obsv_sL_sh.get())
                    self.obsv_sR_q.put(self.obsv_sR_sh.get())
                    self.obsv_psi_q.put(self.obsv_psi_sh.get())
                    self.obsv_psi_dot_q.put(self.obsv_psi_dot_sh.get())

                    yield self.state
                else:
                    print("Queues full or abort signal received, stopping data collection.")
                    # Set flags
                    # self.col_start.put(0)
                    self.col_done.put(1)
                    # self.mtr_enable.put(0)
                    self.state = self.S3_COLLECTION_DONE
            
            ### 3: DONE STATE --------------------------------------------------
            elif (self.state == self.S3_COLLECTION_DONE):
                # Wait for col_start to be cleared
                if not self.col_start.get():
                    self.state = self.S1_WAIT_FOR_START_COLLECTING

            yield self.state