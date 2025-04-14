#just MPC controller 

#WORKED
import cv2
import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import scipy 
from scipy.interpolate import CubicSpline
from matplotlib import animation

N = 5  # Time horizon
f_max = 25
a_0 = 1
dt = 0.1
Q = np.diag([30, 30])  # Weight matrix
R = np.diag([20, 20])  # Penalty matrix



def dynamic_model(freq, alpha, PosX, PosY):
    """Discrete-time system dynamics for MPC prediction ignoring disturbance terms."""
    dPosX = a_0 * freq * np.cos(alpha) 
    dPosY = a_0 * freq * np.sin(alpha) 

    # Discrete-time update (Euler integration)
    PosX_next = PosX + dPosX * dt
    PosY_next = PosY + -(dPosY) * dt
    

    return PosX_next,  PosY_next




def generate_waypoints(start, target, M):
    """Generate interpolated waypoints from start to target using a curve fitting to accommodate curves"""
    t = np.linspace(0, 1, M)  # Normalized time steps
    x = np.linspace(start[0], target[0], M)  # Intermediate control points
    y = np.linspace(start[1], target[1], M)  

    # Create cubic splines
    #cs_x = CubicSpline(np.linspace(0, 1, M), x)
    #cs_y = CubicSpline(np.linspace(0, 1, M), y)

    waypoints_x = x
    waypoints_y = y

    return np.vstack((waypoints_x, waypoints_y))
    #waypoints = scipy.optimize.curve_fit(start, target, N).T  # Shape: (2, N)
    #return waypoints




def cost_func(waypoints, X, u):
    """Compute cost function to minimize tracking error to waypoints."""
    #cost = Q||current position(t) - way point position(t+1)||^2+||u(t|t-1)||^2.R 
    cost = 0
    
    for k in range(N):
        traj_error = X[:2, k] - waypoints[:, k]  # Position error to waypoint
        if k ==0:
            control_error = ca.DM.zeros(2, 1)
        else:
            control_error = u[:,k] - u[:,k-1]
        cost += ca.mtimes([traj_error.T, Q, traj_error]) + ca.mtimes([control_error.T, R, control_error])
    return cost


def solve_mpc(frame, X_current, X_desired):
    """Solves the MPC problem with ESO-estimated states."""
    opti = ca.Opti()
    

    # Decision variables
    u = opti.variable(2, N)  # [freq, alpha] for each step
    X = opti.variable(2, N + 1)  # [PosX,  PosY] for each step

    #waypoints = generate_waypoints([700,700], X_desired, N+1)
    waypoints = generate_waypoints(X_current, X_desired, N+3)
   

    for i in range(waypoints.shape[1]):
        x = waypoints[0, i]
        y = waypoints[1, i]
        cv2.circle(frame, (int(x), int(y)), radius=10, color=(0, 0, 255), thickness=-1)  # red filled circle
    
    

    # Initial condition constraint
    opti.subject_to(X[:, 0] == X_current)

    # Constraints
    for i in range(N):
        # Dynamics constraints
        PosX_next = X[0, i] + (a_0 * u[0, i] *( ca.cos(u[1, i])) ) * dt
        PosY_next = X[1, i] + (a_0 * u[0, i] *(- ca.sin(u[1, i])) )* dt
        
        
        #print(PosX_next, PosY_next)
        #cv2.circle(frame,(int(100), int(100)),15,(0,255,0), -1,)
         # Disturbance assumed constant
         

        opti.subject_to(X[0, i + 1] == PosX_next)
        opti.subject_to(X[1, i + 1] == PosY_next)
        #opti.subject_to(X[1, i + 1] == D_x_next)
        #opti.subject_to(X[3, i + 1] == D_y_next)

        # Control constraints
        opti.subject_to(0 <= u[0, i])  # freq >= 0
        opti.subject_to(u[0, i] <= f_max)  # freq <= f_max
        opti.subject_to(-np.pi <= u[1, i])  # alpha >= -pi
        opti.subject_to(u[1, i] <= np.pi)  # alpha <= pi

    # Cost function (tracking error + control effort)
  
    cost = cost_func( waypoints, X, u)
    #print("cost = ", cost)
    
    opti.minimize(cost)



    opts= {'print_time': 0, 'ipopt': {'print_level': 0}}

    opti.solver('ipopt',opts)


    # Set solver
    #opti.solver('ipopt')

    # Solve optimization problem
    solution = opti.solve()
    freq_value = round(solution.value(u[0,0]),2)

    return frame, freq_value, solution.value(u[1, 0])  # Return first control input (freq, alpha)


"""
X_current = np.array([500 ,1000])
X_desired = np.array([1000,1000])
frame = 1
print(solve_mpc(frame, X_current, X_desired))

"""