import cv2
from classes.ESO_MPCcontroller import solve_mpc
from classes.ESO_MPCcontroller import eso
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
        #FOR ESO-MPC implementation
        
        #input:  robot_list which stores all the attributes for each robot you select

        pos_x = robot_list[-1].position_list[-1][0]
        pos_y = robot_list[-1].position_list[-1][1]

        trajectory = robot_list[-1].trajectory
        #print("trajectory = ", trajectory)

        print("robots position = ", (pos_x,pos_y))

        
      
        
        #middle: algorithm
        
        
        #X_current = np.array([int(pos_x), int(pos_y)])
        X_current = np.array([int(pos_x),0, int(pos_y),0]) #X_hat [posx,disx,posy,disy]
        freq_val = 0.0
        alpha_val = 0.0
        #X_current = np.array([int(pos_x), int(pos_y)])
        X_desired =trajectory[-1]
        # X_desired = np.array([1000,1000])

        while True:
        

            X_current = eso(X_current,pos_x,pos_y,freq_val,alpha_val) 
        
            cv2.circle(frame,(X_desired[0],X_desired[1]),15,(0,0,0), -1,)
            
            start = time.time()
            frame, freq, alpha = solve_mpc(frame, X_current, X_desired) #freq in Hz and alpha in radians
            
            freq_val = freq
            alpha_val = alpha

        
        #X_current = update_position(X_current,freq,alpha)
        #best_u = mpc_control(X_current)
        #alpha = best_u[0]
        #freq = best_u[1]

            print(time.time()-start)
            
            print("freq = {}, alpha ={} from mpc".format(freq, alpha))

            
    

            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
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
            
            print("freq = {}, alpha ={}sent to arduino".format(freq, alpha))
            
            return frame, Bx, By, Bz, alpha, gamma, freq, psi, gradient, equal_field, acoustic_freq
        
            
        
        


'''

    def run(self, frame, robot_list):
        #MPC execution function
        
        
        pos_x = robot_list[-1].position_list[-1][0]
        pos_y = robot_list[-1].position_list[-1][1]

        trajectory = robot_list[-1].trajectory
        #print("trajectory = ", trajectory)

        print("robots position = ", (pos_x,pos_y))

        
      
        
        #middle: algorithm
        
        
        #X_current = np.array([int(pos_x), int(pos_y)])
     
        X_current = np.array([int(pos_x), int(pos_y)])
        X_desired = trajectory[-1]  #np.array([1000,1000])
        print(X_desired)

        
        cv2.circle(frame,(X_desired[0],X_desired[1]),15,(0,0,0), -1,)
            
        start = time.time()
        frame, freq, alpha = solve_mpc(frame, X_current, X_desired)

        print(time.time()-start)
        print("freq = {}, alpha ={} from mpc".format(freq, alpha))
        
        
        
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
        
        print("freq = {}, alpha ={}sent to arduino".format(freq, alpha))
        #dist = np.linalg.norm([pos_x - X_desired[0], pos_y - X_desired[1]])
          #  if dist < 5.0:
         #       print("Target reached!")
          #      freq = 0
        return frame, Bx, By, Bz, alpha, gamma, freq, psi, gradient, equal_field, acoustic_freq

         '''