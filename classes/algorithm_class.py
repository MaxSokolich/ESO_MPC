
from ESOMPC_controller import solve_mpc
import numpy as np

class algorithm:
    def __init__(self):
        pass
        

    def run(self, robot_list):
        """
        input: data about robot. eg, velocity or position
        output: magnetic field action commands
        """
        
        
        #input:  robot_list which stores all the attributes for each robot you select

        pos_x = robot_list[-1].position_list[-1][0]
        pos_y = robot_list[-1].position_list[-1][1]
      
        
        #middle: algorithm
        
        
        X_current = np.array([pos_x, pos_y])
        print(X_current)
        X_desired = np.array([1000,1000])
        
        freq, alpha = solve_mpc(X_current, X_desired)
        
        alpha = alpha - np.pi/2
        print(freq, alpha)
        
        
        
        #output: actions which is the magetnic field commands applied to the arduino

        Bx = 1 #-1 -> 1
        By = 0 #-1 -> 1
        Bz = 0 #-1 -> 1
        alpha = alpha #0 -> 360 deg
        gamma = 90 #0 -> 180 deg
        freq = freq #0 -> 180 Hz
        psi = 0 #0 -> 90 deg
        gradient = 1 # gradient has to be 1 for the gradient thing to work
        acoustic_freq = 0
        
        
        return Bx, By, Bz, alpha, gamma, freq, psi, gradient, acoustic_freq