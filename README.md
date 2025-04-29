# Quadrotor Modelling, Control, and State Estimation

This repository contains the modeling, control design, and state estimation of a quadrotor UAV using MATLAB and Simulink. The work covers both linear and nonlinear models, optimal control strategies, disturbance rejection, and full-state estimation with an Extended Kalman Filter (EKF).

## ✈️ Project Highlights
- Full 6-DOF **nonlinear quadrotor dynamics** implemented in Simulink and MATLAB Function blocks.
- **Aerodynamic drag** effects included in the model.
- **Linearization** of the nonlinear model around hover equilibrium.
- **LQR controller** for stabilization using linearized models.
- **LQR with integral action** to eliminate steady-state errors during trajectory tracking.
- **Extended Kalman Filter (EKF)** to estimate the full 12-dimensional state vector from noisy position and angular velocity measurements.
- **3D trajectory tracking** tasks including circular, rectangular, and spiral trajectories.
- **Robustness testing** under disturbances (wind gusts, payload mass variation) and noisy measurements.

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

Spiral and circular trajectory tracking under external disturbances.

Robustness validation under measurement noise and model uncertainty.

🖥️ Software Requirements
MATLAB R2023a or newer

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
