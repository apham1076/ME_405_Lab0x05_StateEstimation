# state_estimation_task.py
#
# ==============================================================================
# StateEstimationTask
# ------------------------------------------------------------------------------
# This task predicts the future state of the system using a system model and sensor inputs
# ==============================================================================

from ulab import numpy as np

class StateEstimationTask:

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_ENABLE = 1
    S2_RUN = 2

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self):

        # Shares

        # Queues

        # Flags

        # Controllers


        self.state = self.S0_INIT # ensure FSM starts in state S0_INIT

    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------

    def state_eqn(t, x, u):
        '''!@brief      Implements the  state equations for the open loop system
            @param t    The value of time for a given simulation step
            @param x    The value of the state vector for a given simulation step
            @return     A tuple containing both the derivative of the state vector
                        and the output vector for a given simulation step
        '''
        # x is an array representing the state vector with dimensions (4,1) containing [OmegaL; OmegaR; s; psi]
        # u is an array representing the input vector with dimensions (2,1) containing [v_left; v_right]
        # xd is an array representing the derivative of the state vector with dimensions (4,1) containing [OmegaL_dot; OmegaR_dot; s_dot; psi_dot]

        # Placeholder parameters (to be replaced with actual system parameters)
        r = 0.03       # wheel radius (m)
        w = 0.15       # wheelbase (m)
        K = 1.0      # motor gain (rad/s per V)
        tau = 0.5    # motor time constant (s)
        
        A = np.array([[-1/tau], [0], [0], [0],
                      [0], [-1/tau], [0], [0],
                      [-r/2], [r/2], [0], [0],
                      [-r/w], [r/w], [0], [0]])

        B = np.array([[K/tau], [0],
                      [0], [K/tau],
                      [0], [0],
                      [0], [0]])

        xd = np.dot(A, x) + np.dot(B, u)
        
        return xd
    
    def output_eqn(t, x, u):
        '''!@brief      Implements the output equations for the system
            @param t    The value of time for a given simulation step
            @param x    The value of the state vector for a given simulation step
            @return     A tuple containing both the derivative of the state vector
                        and the output vector for a given simulation step
        '''

        # x is an array representing the state vector with dimensions (4,1) containing [OmegaL; OmegaR; s; psi]
        # u is an array representing the input vector with dimensions (2,1) containing [v_left; v_right]
        # y is an array representing the output vector with dimensions (4,1) containing [sL; sR; psi; psi_dot]

        # Placeholder parameters (to be replaced with actual system parameters)
        r = 0.03       # wheel radius (m)
        w = 0.15       # wheelbase (m)
        K = 1.0      # motor gain (rad/s per V)
        tau = 0.5    # motor time constant (s)

        C = np.array([[0], [0], [1], [-w/2],
                      [0], [0], [1], [w/2],
                      [0], [0], [0], [1],
                      [-r/w], [r/w], [0], [0]])
        
        D = np.array([[0], [0],
                      [0], [0],
                      [0], [0],
                      [0], [0]])

        y = np.dot(C, x) + np.dot(D, u)
        
        return y
    
    def RK4_solver(fcn1, fcn2, x, tstep):
        '''!@brief        Performs a single step of the RK4 solver
            @param x      The current value of the state vector
            @param fcn1   A function handle to the first function to solve
            @param fcn2   A function handle to the second function to solve
            @param tstep  The time step size to use for the integration algorithm
            @return       A tuple the value of the state and output vectors after one time step
        '''
        # tstep should be the same as the period of this task
        # Need to pass u into fcn1 and fcn2

        # Evaluate the function handle at the several times with the
        # value of the state vector to compute derivatives, k
        k1, y1 = fcn1(0, x), fcn2(0, x)
        k2, y2 = fcn1(0 + 0.5*tstep, x + 0.5*k1*tstep), fcn2(0 + 0.5*tstep, x + 0.5*k1*tstep)
        k3, y3 = fcn1(0 + 0.5*tstep, x + 0.5*k2*tstep), fcn2(0 + 0.5*tstep, x + 0.5*k2*tstep)
        k4, y4 = fcn1(0 + tstep, x + k3*tstep), fcn2(0 + tstep, x + k3*tstep)

        # Evaluate a weighted-average of derivatives
        xd = (k1 + 2*k2 + 2*k3 + k4)/6
        y = (y1 + 2*y2 + 2*y3 + y4)/6

        xout = x + xd.T*tstep
        yout = y.T

        # return tout, yout
        return xout, yout

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):

                self.state = self.S1_WAIT_FOR_ENABLE # set next state

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_ENABLE):
                
                self.state = self.S2_RUN # set next state
            
            ### 2: RUN STATE ---------------------------------------------------
            elif (self.state == self.S2_RUN):
                
                self.state = self.S2_RUN # remain in this state
            
            yield self.state