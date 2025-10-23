"""
Simple simulation of cooperative relative localization (inspired by V. SIMULATIONS - A. Cooperative RL Simulation Results)
- Two agents: leader and follower.
- True trajectories generated deterministically.
- Odometry (v, omega) measured with noise for each agent.
- UWB range measurements between agents at lower rate.
- EKF that estimates relative pose of follower in leader frame using odometry integration for prediction and range for correction.

Assumptions / simplifications:
- UWB provides only range (no bearing).
- Agents share odometry (cooperative) so estimator uses both odometry signals for prediction.
- Process Jacobian is approximated; process noise Q is tuned as a constant.

Run: python cooperative_rl_sim.py
"""

import numpy as np
import matplotlib.pyplot as plt

def generate_uav_trajectories(k, initial_positions):
    """
    Generate trajectories for 6 UAVs based on the given equations and initial positions.

    Parameters:
        k (float): Time step.
        initial_positions (list): List of initial positions for 6 UAVs.

    Returns:
        list: Positions of 6 UAVs as a list of numpy arrays.
    """
    # Calculate offsets to make trajectories pass through initial positions at k=0
    v1_k = initial_positions[0] + np.array([np.cos(k / 3) - 1, -(5 / 3) * np.sin(k / 3)])
    v2_k = initial_positions[1] + np.array([-2 * np.sin(k), 2 * np.sin(k)])  # Already passes through initial at k=0
    v3_k = initial_positions[2] + np.array([np.cos(k / 5) - np.sin(k / 5) - 1, np.sin(k / 5) + np.cos(k / 5) - 1])
    v4_k = initial_positions[3] + np.array([-3 * np.sin(k), 3 * np.cos(k) - 3])
    v5_k = initial_positions[4] + np.array([1 - 1, 0])  # Subtract initial offset
    v6_k = initial_positions[5] + np.array([-(10 / 3) * np.sin(k / 3), (5 / 3) * np.cos(k / 3) - (5 / 3)])

    return [v1_k, v2_k, v3_k, v4_k, v5_k, v6_k]

def plot_uav_trajectories(time_steps, initial_positions):
    """
    Plot the trajectories of 6 UAVs over the given time steps with initial positions.

    Parameters:
        time_steps (list): List of time steps.
        initial_positions (list): List of initial positions for 6 UAVs.
    """
    trajectories = {i: [] for i in range(1, 7)}

    for k in time_steps:
        positions = generate_uav_trajectories(k, initial_positions)
        for i, pos in enumerate(positions, start=1):
            trajectories[i].append(pos)

    plt.figure(figsize=(10, 10))
    for i in range(1, 7):
        trajectory = np.array(trajectories[i])
        plt.plot(trajectory[:, 0], trajectory[:, 1], label=f'UAV {i}')

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('UAV Trajectories with Initial Positions')
    plt.legend()
    plt.grid()
    plt.show()



if __name__ == "__main__":
    # Define time steps for simulation
    time_steps = np.linspace(0, 300, 1000)  # Example: 1000 steps from t=0 to t=20

    # Define initial positions for UAVs
    initial_positions = [
        np.array([0, 0]),
        np.array([2, -30]),
        np.array([20, -15]),
        np.array([-20, 8]),
        np.array([-14, 8]),
        np.array([-10, -30])
    ]

    plot_uav_trajectories(time_steps, initial_positions)
