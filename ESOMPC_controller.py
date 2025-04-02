#ESO-MPC controller

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import scipy 
from scipy.interpolate import CubicSpline

N = 30  # Time horizon
f_max = 10
a_0 = 1
dt = 0.01
Q = np.diag([10, 10])  # Weight matrix
R = np.diag([1, 1])  # Penalty matrix

def eso(X_hat, freq, alpha, dt, zeta, q_xm, q_ym, a_0):
    """Extended State Observer (ESO) for disturbance estimation."""
    PosX_hat, D_x_hat, PosY_hat, D_y_hat = X_hat

    # Observer dynamics
    dPosX_hat = D_x_hat + a_0 * freq * np.cos(alpha) + (zeta[0]) * (q_xm - PosX_hat)
    dD_x_hat = (zeta[1] ) * (q_xm - PosX_hat)
    dPosY_hat = D_y_hat + a_0 * freq * np.sin(alpha) + (zeta[2]) * (q_ym - PosY_hat)
    dD_y_hat = (zeta[3]) * (q_ym - PosY_hat)

    # Euler integration
    X_hat_new = np.array([
        PosX_hat + dPosX_hat * dt,
        D_x_hat + dD_x_hat * dt,
        PosY_hat + dPosY_hat * dt,
        D_y_hat + dD_y_hat * dt
    ])
    return X_hat_new

def dynamic_model(freq, alpha, PosX, PosY, dt, a_0):
    """Discrete-time system dynamics for MPC prediction ignoring disturbance terms."""
    dPosX = a_0 * freq * np.cos(alpha) 
    dPosY = a_0 * freq * np.sin(alpha) 

    # Discrete-time update (Euler integration)
    PosX_next = PosX + dPosX * dt
    PosY_next = PosY + dPosY * dt
    

    return PosX_next,  PosY_next

def generate_waypoints(start, target, N):
    """Generate interpolated waypoints from start to target using a curve fitting to accommodate curves"""
    t = np.linspace(0, 1, N)  # Normalized time steps
    x = np.linspace(start[0], target[0], N)  # Intermediate control points
    y = np.linspace(start[1], target[1], N)  

    # Create cubic splines
    cs_x = CubicSpline(np.linspace(0, 1, len(x)), x)
    cs_y = CubicSpline(np.linspace(0, 1, len(y)), y)

    waypoints_x = cs_x(t)
    waypoints_y = cs_y(t)

    return np.vstack((waypoints_x, waypoints_y))
    #waypoints = scipy.optimize.curve_fit(start, target, N).T  # Shape: (2, N)
    #return waypoints


def cost_func(Q, R, N, waypoints, X, u):
    """Compute cost function to minimize tracking error to waypoints."""
    cost = 0
    for k in range(N):
        traj_error = X[:2, k] - waypoints[:, k]  # Position error to waypoint
        cost += ca.mtimes([traj_error.T, Q, traj_error]) + ca.mtimes([u[:, k].T, R, u[:, k]])
    return cost

def solve_mpc(X_current, X_desired, Q, R, N, dt, a_0, f_max):
    """Solves the MPC problem with ESO-estimated states."""
    opti = ca.Opti()

    # Decision variables
    u = opti.variable(2, N)  # [freq, alpha] for each step
    X = opti.variable(4, N + 1)  # [PosX, D_x, PosY, D_y] for each step

    waypoints = generate_waypoints(X_current[:2], X_desired.reshape(2), N)

    # Initial condition constraint
    opti.subject_to(X[:, 0] == X_current)

    # Constraints
    for i in range(N):
        # Dynamics constraints
        PosX_next = X[0, i] + (a_0 * u[0, i] * ca.cos(u[1, i])  + X[1,i]) * dt
        PosY_next = X[2, i] + (a_0 * u[0, i] * ca.sin(u[1, i])  +  X[3,i])* dt
        D_x_next = X[1, i]  # Disturbance assumed constant
        D_y_next = X[3, i]

        opti.subject_to(X[0, i + 1] == PosX_next)
        opti.subject_to(X[2, i + 1] == PosY_next)
        opti.subject_to(X[1, i + 1] == D_x_next)
        opti.subject_to(X[3, i + 1] == D_y_next)

        # Control constraints
        opti.subject_to(0 <= u[0, i])  # freq >= 0
        opti.subject_to(u[0, i] <= f_max)  # freq <= f_max
        opti.subject_to(-np.pi <= u[1, i])  # alpha >= -pi
        opti.subject_to(u[1, i] <= np.pi)  # alpha <= pi

    # Cost function (tracking error + control effort)
    cost = cost_func(Q, R, N, waypoints, X, u)
    opti.minimize(cost)

    # Set solver
    opti.solver('ipopt')

    # Solve optimization problem
    solution = opti.solve()

    return solution.value(u[0, 0]), solution.value(u[1, 0])  # Return first control input (freq, alpha)


