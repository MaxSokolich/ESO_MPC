import numpy as np
import casadi as ca
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import time as time

N = 4  # Time horizon
f_max = 10
a_0 = 1
dt = 0.04
Q = np.diag([20, 20])  # Weight matrix
R = np.diag([10, 10])  # Penalty matrix
zeta = np.array([3, 1, 3, 1]) 


def eso(X_hat, freq, alpha, dt, zeta, PosX, PosY, a_0):
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

def dynamic_model(freq, alpha, PosX, PosY, dt, a_0):
    """Discrete-time system dynamics for MPC prediction ignoring disturbance terms."""
    dPosX = a_0 * freq * np.cos(alpha) 
    dPosY = a_0 * freq * np.sin(alpha) 

    # Discrete-time form 
    PosX_next = PosX + dPosX * dt
    PosY_next = PosY + dPosY * dt
    
    return PosX_next,  PosY_next

def generate_waypoints(start, target, N):
    """Generate interpolated waypoints from start to target using spline curve"""

    t = np.linspace(0, 1, N)  
    x = np.linspace(start[0], target[0], N)  
    y = np.linspace(start[1], target[1], N)  

    cs_x = CubicSpline(np.linspace(0, 1, len(x)), x)
    cs_y = CubicSpline(np.linspace(0, 1, len(y)), y)

    traj_points  = np.vstack((cs_x(t), cs_y(t)))

    return traj_points
    

def cost_func(Q, R, N, waypoints, X, u):
    """Compute cost function to minimize tracking error to waypoints."""
    #cost = Q||current position(t) - way point position(t+1)||^2+||u(t|t-1)||^2.R 
    cost = 0
    for k in range(N):
        traj_error = X[:2, k] - waypoints[:, k]  # Position error to waypoint
        cost += ca.mtimes([traj_error.T, Q, traj_error]) + ca.mtimes([u[:, k].T, R, u[:, k]])
    return cost

def solve_mpc(X_current, X_desired):
    """Solves the MPC problem with ESO-estimated states."""
    opti = ca.Opti()

    # Decision variables
    u = opti.variable(2, N)  # [freq, alpha] for each step
    X = opti.variable(2, N + 1)  # [PosX, PosY] for each step
    X_hat = X_current
    if len(X_hat) == 2:
        X_hat = np.array([X_current[0], 0.0, X_current[1], 0.0])

    start = np.array([X_hat[0], X_hat[2]]) 
    target = X_desired[:,]               
    if target.shape[0] != 2:
        raise ValueError(f"Expected 2D target, got shape {target.shape} with data: {target}")
    waypoints = generate_waypoints(start, target, N+1)

    # Initial condition constraint
    initial_pos = ca.vertcat(X_hat[0], X_hat[2])  # Extract positions only
    opti.subject_to(X[:, 0] == initial_pos)

    # Constraints
    for i in range(N):

        PosX_next = X[0, i] + (a_0 * u[0, i] * ca.cos(u[1, i]) +  X_hat[1]) * dt
        PosY_next = X[1, i] + (a_0 * u[0, i] * ca.sin(u[1, i]) + X_hat[3] )* dt
          
        opti.subject_to(X[0, i + 1] == PosX_next)
        opti.subject_to(X[1, i + 1] == PosY_next)
        
        # Control constraints
        opti.subject_to(0 <= u[0, i] )  # freq 0 to f_max
        opti.subject_to(u[0,i] <= f_max)
        opti.subject_to(-np.pi <= u[1, i] )
        opti.subject_to(u[1,i] <= np.pi)  # alpha [-pi,pi]
        

    # Cost function (tracking error + control effort)
    cost = cost_func(Q, R, N, waypoints, X, u)
    opti.minimize(cost)
    
    opts= {'print_time': 0, 'ipopt': {'print_level': 0}}

    opti.solver('ipopt',opts)

    #print(opti.debug.value)
    
    solution = opti.solve()

    #del opti

    freq_value = round(solution.value(u[0, 0]), 2)
    alpha_value = solution.value(u[1, 0])

    return freq_value, alpha_value  # control inputs (freq, alpha)



"""start = time.time()
for i in range (100):
    start = time.time()
    X_current = np.array([500 ,1000])
    X_desired = np.array([1000,1000])
    freq_value, alpha_value = solve_mpc(X_current, X_desired)
    elapsed = start - time.time()
    print(elapsed)
"""
