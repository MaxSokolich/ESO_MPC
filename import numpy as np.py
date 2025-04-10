import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.interpolate import CubicSpline
import casadi as ca

# Constants (You can adjust these values as necessary)
f_max = 100  # Maximum frequency (in Hz)
a_0 = 1      # Speed scaling factor (adjust as necessary)
dt = 0.1     # Time step
N = 5        # Prediction horizon
Q = np.diag([20, 20])  # Weight matrix
R = np.diag([10, 10])  # Penalty matrix

def generate_waypoints(start, target, M):
    """Generate interpolated waypoints from start to target using cubic splines."""
    t = np.linspace(0, 1, M)
    x = np.linspace(start[0], target[0], M)
    y = np.linspace(start[1], target[1], M)

    # Create cubic splines
    cs_x = CubicSpline(np.linspace(0, 1, M), x)
    cs_y = CubicSpline(np.linspace(0, 1, M), y)

    # Generate waypoints using the spline
    waypoints_x = cs_x(t)
    waypoints_y = cs_y(t)

    return np.vstack((waypoints_x, waypoints_y))  # Shape: (2, M)

def cost_func(Q, R, N, waypoints, X, u):
    """Compute the cost function to minimize tracking error to waypoints."""
    cost = 0
    for k in range(N):
        traj_error = X[:2, k] - waypoints[:, k]  # Position error to waypoint
        cost += ca.mtimes([traj_error.T, Q, traj_error]) + ca.mtimes([u[:, k].T, R, u[:, k]])
    return cost

def solve_mpc(X_current, X_desired):
    """Solves the MPC problem."""
    opti = ca.Opti()

    # Decision variables
    u = opti.variable(2, N)  # [freq, alpha] for each step
    X = opti.variable(2, N + 1)  # [PosX, PosY] for each step

    # Generate waypoints
    waypoints = generate_waypoints(X_current[:2], X_desired[:2], N + 1)

    # Initial condition constraint
    opti.subject_to(X[:, 0] == X_current)

    # Constraints
    for i in range(N):
        # Dynamics constraints (for x and y position)
        PosX_next = X[0, i] + (a_0 * u[0, i] * ca.cos(u[1, i])) * dt
        PosY_next = X[1, i] + (a_0 * u[0, i] * ca.sin(u[1, i])) * dt

        opti.subject_to(X[0, i + 1] == PosX_next)
        opti.subject_to(X[1, i + 1] == PosY_next)

        # Control constraints
        opti.subject_to(0 <= u[0, i])  # freq >= 0
        opti.subject_to(u[0, i] <= f_max)  # freq <= f_max
        opti.subject_to(-np.pi <= u[1, i])  # alpha >= -pi
        opti.subject_to(u[1, i] <= np.pi)  # alpha <= pi

    # Cost function (tracking error + control effort)
    cost = cost_func(Q, R, N, waypoints, X, u)
    opti.minimize(cost)

    opts = {'print_time': 0, 'ipopt': {'print_level': 0}}
    opti.solver('ipopt', opts)

    # Solve optimization problem
    solution = opti.solve()

    # Return the first control input (frequency and alpha)
    return solution.value(u[0, 0]), solution.value(u[1, 0])

def update_position(X, freq, alpha):
    """Update the robot's position based on the control inputs."""
    new_pos_x = X[0] + a_0 * freq * np.cos(alpha) * dt
    new_pos_y = X[1] + a_0 * freq * np.sin(alpha) * dt
    return np.array([new_pos_x, new_pos_y])

# --- Simulation Loop ---
def simulate_and_animate(X_current, X_desired, frames=200):
    fig, ax = plt.subplots()
    ax.set_xlim(0, 2200)
    ax.set_ylim(0, 2200)

    # Generate waypoints
    waypoints = generate_waypoints(X_current, X_desired, N+1)
    ax.plot(waypoints[0], waypoints[1], 'b--', label="Waypoints")

    robot_pos, = ax.plot([], [], 'go', markersize=8, label="Robot")

    # Initial robot position
    X = np.array(X_current)

    def animate(frame):
        nonlocal X

        # Solve MPC to get control input
        freq, alpha = solve_mpc(X, X_desired)
        print(freq)
        # Update the robot's position
        X = update_position(X, freq, alpha)

        # Update the robot's position on the plot
        robot_pos.set_data(X[0], X[1])

        return robot_pos,

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=100, blit=True)

    plt.legend()
    plt.show()

# --- Run Simulation ---
X_current = np.array([500, 1500])
X_desired = np.array([2000, 1000])

simulate_and_animate(X_current, X_desired, frames=200)
