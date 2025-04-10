import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.interpolate import CubicSpline
import casadi as ca
from ESOMPC_controller import generate_waypoints
from ESOMPC_controller import solve_mpc
from ESOMPC_controller import cost_func
from ESOMPC_controller import dynamic_model
# Constants (You can adjust these values as necessary)
f_max = 100  # Maximum frequency (in Hz)
a_0 = 1      # Speed scaling factor (adjust as necessary)
dt = 0.1     # Time step
N = 5        # Prediction horizon
Q = np.diag([20, 20])  # Weight matrix
R = np.diag([10, 10])  # Penalty matrix



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
        f,freq, alpha = solve_mpc(1, X, X_desired)
        print(freq)
        # Update the robot's position
        X = dynamic_model(freq, alpha,X[0],X[1])

        # Update the robot's position on the plot
        robot_pos.set_data(X[0], X[1])

        return robot_pos,

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=100, blit=True)

    plt.legend()
    plt.show()

# --- Run Simulation ---
X_current = np.array([500, 1500])
X_desired = np.array([600, 1500])

simulate_and_animate(X_current, X_desired, frames=200)
