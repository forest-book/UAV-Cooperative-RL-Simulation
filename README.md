# thesis-UWB-and-Odometry-Based-Cooperative-Relative-Localization

Cooperative Relative Localization Simulation

This repository contains a simple Python simulation implementing an EKF-based cooperative relative localization inspired by the paper's V. SIMULATIONS - A. Cooperative RL Simulation Results.

Files:
- `cooperative_rl_sim.py`: main simulation script. Runs a two-agent scenario (leader and follower), noisy odometry and UWB range, EKF estimation of relative pose, and Matplotlib plots.

Requirements:
- Python 3.8+
- numpy
- matplotlib

Run:
1. Install requirements:
	pip install -r requirements.txt
2. Run the simulation:
	python cooperative_rl_sim.py

The script saves `cooperative_rl_results.png` and prints mean position errors for odometry-only vs EKF.