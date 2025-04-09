#ESO-MPC controller

import numpy as np
import casadi as ca
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

N = 30  # Time horizon
f_max = 10
a_0 = 1
dt = 0.25
Q = np.diag([20, 20])  # Weight matrix
R = np.diag([1, 1])  # Penalty matrix
zeta = np.array([1, 1, 1, 1]) 


def eso(X_hat, freq, alpha, dt, zeta, PosX, PosY, a_0):
    """Extended State Observer (ESO) for disturbance estimation."""
    PosX_hat, D_x_hat, PosY_hat, D_y_hat = X_hat

    # Observer dynamics, qx_m , qy_m are camera feedback
    dPosX_hat = D_x_hat + a_0 * freq * np.cos(alpha) + (zeta[0]) * (PosX - PosX_hat)
    dD_x_hat = (zeta[1] ) * (PosX - PosX_hat)
    dPosY_hat = D_y_hat + a_0 * freq * np.sin(alpha) + (zeta[2]) * (PosY - PosY_hat)
    dD_y_hat = (zeta[3]) * (PosY - PosY_hat)

    # Euler integration
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

    # Discrete-time update (Euler integration)
    PosX_next = PosX + dPosX * dt
    PosY_next = PosY + dPosY * dt
    
    return PosX_next,  PosY_next

def generate_waypoints(start, target, N):
    """Generate interpolated waypoints from start to target using spline curve"""
    t = np.linspace(0, 1, N)  # Normalized time steps
    x = np.linspace(start[0], target[0], N)  # Intermediate control points
    y = np.linspace(start[1], target[1], N)  

    # Create cubic splines
    cs_x = CubicSpline(np.linspace(0, 1, len(x)), x)
    cs_y = CubicSpline(np.linspace(0, 1, len(y)), y)

    waypoints_x = cs_x(t)
    waypoints_y = cs_y(t)

    return np.vstack((waypoints_x, waypoints_y))
    


def cost_func(Q, R, N, waypoints, X, u):
    """Compute cost function to minimize tracking error to waypoints."""
    #cost = Q||current position(t) - way point position(t+1)||^2+||u(t|t-1)||^2.R 
    cost = 0
    for k in range(N):
        traj_error = X[:2, k] - waypoints[:, k]  # Position error to waypoint
        cost += ca.mtimes([traj_error.T, Q, traj_error]) + ca.mtimes([u[:, k].T, R, u[:, k]])
    return cost

def solve_mpc(X_hat, X_desired):
    """Solves the MPC problem with ESO-estimated states."""
    opti = ca.Opti()

    # Decision variables
    u = opti.variable(2, N)  # [freq, alpha] for each step
    X = opti.variable(2, N + 1)  # [PosX, PosY] for each step

    if len(X_hat) == 2:
        X_hat = np.array([X_hat[0], 0.0, X_hat[1], 0.0])

    waypoints = generate_waypoints(X_hat[[0,2]], X_desired[:,], N+1)
    #print(X_hat.shape)

    # Initial condition constraint
    initial_pos = ca.vertcat(X_hat[0], X_hat[2])  # Extract positions only
    opti.subject_to(X[:, 0] == initial_pos)
    #opti.subject_to(X[0, 0] == X_hat[0])
    #opti.subject_to(X[1, 0] == X_hat[2])

    # Constraints
    for i in range(N):
         #X_hat = eso(X_hat, u[0, i], u[1, i], dt, zeta, X_hat[0], X_hat[2], a_0)

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

    return solution.value(u[0, 0]), solution.value(u[1, 0])  # control inputs (freq, alpha)

X_current = np.array([500 ,1000])
X_desired = np.array([1000,1000])

print(solve_mpc(X_current, X_desired))