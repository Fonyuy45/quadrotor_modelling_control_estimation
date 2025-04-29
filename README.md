# Quadrotor Modelling, Control, and State Estimation
![image](https://github.com/user-attachments/assets/1a1c7923-8f44-47f4-a9c8-7d688e103cf1)


This repository contains the modeling, control design, and state estimation of a quadrotor UAV using MATLAB and Simulink. The work covers both linear and nonlinear models, optimal control strategies, disturbance rejection, and full-state estimation with an Extended Kalman Filter (EKF).

## ✈️ Project Highlights
- Full 6-DOF **nonlinear quadrotor dynamics** implemented in Simulink and MATLAB Function blocks.
- **Aerodynamic drag** effects included in the model.
- **Linearization** of the nonlinear model around hover equilibrium.
- **LQR controller** for stabilization using linearized models.
- **LQR with integral action** to eliminate steady-state errors during trajectory tracking.
- **Extended Kalman Filter (EKF)** to estimate the full 12-dimensional state vector from noisy position and angular velocity measurements.
- **3D trajectory tracking** tasks including circular, rectangular, and spiral trajectories.
![3D Rectangual-trajectory-Tracking](https://github.com/user-attachments/assets/df116c1e-3387-410e-9b5d-3463d3112383)
![3D circular trajectory tracking with kalman filter](https://github.com/user-attachments/assets/74470cac-5d03-4e3b-a951-a2acc81fb67c)
![Spiral-Helix-Trajectory-Tracking](https://github.com/user-attachments/assets/693e8158-5969-4bb6-ab6d-57b9c0313354)

- **Robustness testing** under disturbances (wind gusts, payload mass variation) and noisy measurements.
![Spiral Trajectory tarcking under heavy disturbances](https://github.com/user-attachments/assets/9f3be7ce-bef9-47e0-9845-2efee398f0ef)
![Screenshot 2025-04-19 195344](https://github.com/user-attachments/assets/45caec14-d58a-4870-99f2-8f9941832096)

## 🛠️ Repository Structure
```plaintext
quadrotor_modelling_control_estimation/
├── simulink_models/       # Simulink files (.slx) for linear and nonlinear models
├── matlab scripts/        # MATLAB scripts for control design, linearization, analysis
├── ekf_scripts/           # EKF-related MATLAB functions
├── ekf_estimator/         # Simulink models for EKF estimation
└── .gitignore             # Files/folders ignored by Git


🧠 Methods
State-Space Modeling: Derived from first principles (Newton-Euler dynamics).

Linear Quadratic Regulator (LQR): Designed for the linearized system to achieve optimal stabilization.

Integral Action: Augmented the system to guarantee zero steady-state error for position tracking.

Extended Kalman Filter (EKF): Implemented to perform real-time state estimation under noisy sensor conditions.

📈 Simulations
Regulation from initial roll, pitch, yaw disturbances.

Full 3D point-to-point reference tracking.

Spiral, Rectangular, Linear, and circular trajectory tracking under parameter variations external disturbances.

Hover position hold under parametr variations and external disturbances

Robustness validation under measurement noise and model uncertainty.


🖥️ Software Requirements
MATLAB R2024a or newer

Simulink (with Control System Toolbox)

(Optional) Simscape for 3D visualization

🚀 To Run Simulations
Open simulink_models/quadrotor_nonlinear_model.slx for nonlinear model simulations.

Open lqr_integral_nonlinear_simulation.slx to simulate the LQR + integral controller.

Use trajectory_tracking.slx for 3D path tracking.

Launch ekf_estimator/ekf_estimator_model.slx for state estimation with noisy measurements.

📜 License
This project is licensed under the MIT License.

✍️ Author
Dieudonne Fonyuy
