import cv2
from classes.ESOMPC_controller import solve_mpc

import numpy as np
import time as time



class algorithm:
    def __init__(self):
        pass
        

    def run(self, frame, robot_list):
        """
        input: data about robot. eg, velocity or position
        output: magnetic field action commands
        """
        
        
        #input:  robot_list which stores all the attributes for each robot you select

        pos_x = robot_list[-1].position_list[-1][0]
        pos_y = robot_list[-1].position_list[-1][1]

        print("robots position = ", (pos_x,pos_y))

        
      
        
        #middle: algorithm
        
        
        X_current = np.array([int(pos_x), int(pos_y)])
    
        X_desired = np.array([1000,1000])
        cv2.circle(frame,(X_desired[0],X_desired[1]),15,(0,0,0), -1,)
        
        start = time.time()
        frame, freq, alpha = solve_mpc(frame, X_current, X_desired) #freq in Hz and alpha in radians
        #X_current = update_position(X_current,freq,alpha)
        #best_u = mpc_control(X_current)
        #alpha = best_u[0]
        #freq = best_u[1]

        print(time.time()-start)
   

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        #output: actions which is the magetnic field commands applied to the arduino

        Bx = 0 #-1 -> 1
        By = 0 #-1 -> 1
        Bz = 0 #-1 -> 1
        alpha = alpha - np.pi/2 #0 -> 360 deg must be in radian
        gamma = np.radians(90) #0 -> 180 deg. must be in radian
        freq = freq #0 -> 180 Hz
        psi = np.radians(90) #0 -> 90 deg must be in radian. Must not be zero. DONT touch this
        gradient = 0 # gradient has to be 1 for the gradient thing to work
        equal_field = 0
        acoustic_freq = 0
        
        print("freq = {}, alpha ={}".format(freq, alpha))
        
        return frame, Bx, By, Bz, alpha, gamma, freq, psi, gradient, equal_field, acoustic_freq