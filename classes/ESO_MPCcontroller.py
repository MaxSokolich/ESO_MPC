#ESO-MPC Combined

#WORKING
import numpy as np
import casadi as ca
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import time as time
import cv2

N = 10  # Time horizon
f_max = 3
a_0 = 1
dt = 0.04
Q = np.diag([30, 30])  # Weight matrix
R = np.diag([10, 10])  # Penalty matrix
zeta = np.array([8, 2, 8, 2]) 

target_points = None    # List of (x, y) tuples representing your multi-target sequence.
full_traj = None        # Full reference trajectory: a 2 x T numpy array.
current_traj_index = 0

def eso(X_hat, PosX, PosY,freq,alpha):
    """Extended State Observer (ESO) for disturbance estimation."""
    PosX_hat, D_x_hat, PosY_hat, D_y_hat = X_hat
   
    dPosX_hat = D_x_hat + a_0 * freq * np.cos(alpha) + (zeta[0]) * (PosX - PosX_hat)
    dD_x_hat = (zeta[1] ) * (PosX - PosX_hat)
    dPosY_hat = D_y_hat + a_0 * freq * np.sin(alpha) + (zeta[2]) * (PosY - PosY_hat)
    dD_y_hat = (zeta[3]) * (PosY - PosY_hat)

    X_hat = np.array([
        PosX_hat + dPosX_hat * dt,
        D_x_hat + dD_x_hat * dt,
        PosY_hat + dPosY_hat * dt,
        D_y_hat + dD_y_hat * dt
    ])
    return X_hat

def dynamic_model(freq, alpha, PosX, PosY):
    """Discrete-time system dynamics for MPC prediction ignoring disturbance terms."""
    dPosX = a_0 * freq * np.cos(alpha) 
    dPosY = a_0 * freq * np.sin(alpha) 

    # Discrete-time form 
    PosX_next = PosX + dPosX * dt
    PosY_next = PosY + dPosY * dt
    
    return PosX_next,  PosY_next

def generate_waypoints(start, target, N):
    """Generate interpolated waypoints from start to target using spline curve"""
    #targets = np.array(target)
    t = np.linspace(0, 1, N)  
    x = np.linspace(start[0], target[0], N)  
    y = np.linspace(start[1], target[1], N)  

    cs_x = CubicSpline(np.linspace(0, 1, len(x)), x)
    cs_y = CubicSpline(np.linspace(0, 1, len(y)), y)

    traj_points  = np.vstack((cs_x(t), cs_y(t)))

    return traj_points
'''

def generate_waypoints_multisegment(points, M_per_segment):
    """Generate a continuous trajectory through multiple target points.
       'points' is a list of (x, y) tuples.
       Returns a 2 x (M_per_segment*(n-1) - (n-2)) array of waypoints."""
    segments = []
    for i in range(len(points) - 1):
        seg = generate_waypoints(points[i], points[i+1], M_per_segment)
        # Remove overlapping points between segments (except for the first segment)
        if i > 0:
            seg = seg[:, 1:]
        segments.append(seg)
    full_trajectory = np.hstack(segments)
    return full_trajectory
'''
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

def solve_mpc(frame,X_current, X_desired):
    """Solves the MPC problem with ESO-estimated states."""
    global target_points, full_traj, current_traj_index

    opti = ca.Opti()

    # Decision variables
    u = opti.variable(2, N)  # [freq, alpha] for each step
    X = opti.variable(2, N + 1)  # [PosX, PosY] for each step
    X_known = X_current
    start = np.array([X_known[0], X_known[2]]) 
    target = X_desired[:]               
    #if target_points is None:
    #    target_points = []
     #   full_traj = generate_waypoints_multisegment(target_points, M_per_segment=20)
     #   current_traj_index = 0
    waypoints = generate_waypoints( start,target, N+1)
    #waypoints = full_traj[:, current_traj_index : current_traj_index + N + 1]

    for i in range(waypoints.shape[1]):
        x = waypoints[0, i]
        y = waypoints[1, i]
        cv2.circle(frame, (int(x), int(y)), radius=10, color=(0, 0, 255), thickness=-1)  # red filled circle

    # Initial condition constraint
    initial_pos = ca.vertcat(X_known[0], X_known[2])  # Extract positions only
    opti.subject_to(X[:, 0] == initial_pos)


    # Constraints
    for i in range(N):

        PosX_next = X[0, i] + (a_0 * u[0, i] * ca.cos(u[1, i]) +  X_known[1]) * dt
        PosY_next = X[1, i] + (a_0 * u[0, i] * (- ca.sin(u[1, i])) + X_known[3] )* dt
          
        opti.subject_to(X[0, i + 1] == PosX_next)
        opti.subject_to(X[1, i + 1] == PosY_next)
        
        # Control constraints
        opti.subject_to(0 <= u[0, i] )  # freq 0 to f_max
        opti.subject_to(u[0,i] <= f_max)
        opti.subject_to(-np.pi <= u[1, i] )
        opti.subject_to(u[1,i] <= np.pi)  # alpha [-pi,pi]
        

    # Cost function (tracking error + control effort)
    cost = cost_func(waypoints, X, u)
    opti.minimize(cost)
    
    opts= {'print_time': 0, 'ipopt': {'print_level': 0}}

    opti.solver('ipopt',opts)

    #print(opti.debug.value)
    
    solution = opti.solve()

    #del opti

    freq_value = round(solution.value(u[0, 0]), 2)
    alpha_value = solution.value(u[1, 0])
 
    return frame,freq_value, alpha_value  # control inputs (freq, alpha)

'''

start = time.time()
for i in range (100):
    start = time.time()
    X_current = np.array([500 ,1000])
    X_desired = np.array([1000,1000])
    freq_value, alpha_value = solve_mpc(X_current, X_desired)
    elapsed =  time.time() - start
    print(elapsed)

'''