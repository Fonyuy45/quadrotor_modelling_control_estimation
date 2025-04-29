# 📄 Project Overview: Quadrotor State Estimation and Control

## 🔍 Objective
This project focuses on modeling, control, and **state estimation** of a quadrotor UAV using MATLAB and Simulink. The goal is to:
- Develop a full nonlinear 12-state model of the quadrotor.
- Design optimal controllers (LQR + integral action).
- Estimate all states from limited noisy sensor data using an Extended Kalman Filter (EKF).

---

## 🧠 What Was Estimated
The **Extended Kalman Filter (EKF)** was used to estimate the full state vector:

```
x = [x, y, z, phi, theta, psi, u, v, w, p, q, r]'
```

This includes:
- **Position** (x, y, z)
- **Orientation** (roll `phi`, pitch `theta`, yaw `psi`)
- **Linear velocities** (u, v, w) in body frame
- **Angular rates** (p, q, r)

---

## 🎯 What Was Measured
The estimator assumes the following sensor measurements (with noise):
- **Position** from motion capture / GPS: `(x, y, z)`
- **Angular velocities** from IMU: `(p, q, r)`

---

## ❓ Why EKF?
The **Extended Kalman Filter** was chosen because:
- The quadrotor dynamics are nonlinear.
- Linear filters (like Luenberger) would not perform well under large attitude changes.
- EKF allows real-time recursive estimation by linearizing around current state estimates.
- MATLAB/Simulink supports symbolic Jacobian computation, easing EKF implementation.

---

## 🧭 Reference Frames
The quadrotor is modeled using two coordinate systems:

- **Global Frame (G):** Inertial world reference
- **Body Frame (b):** Attached to quadrotor center of mass

![Quadrotor Reference Frames](../simulink_models/plots/quadrotor_frames.png)

Rotations between frames are handled via ZYX Euler angles: `yaw (ψ)`, `pitch (θ)`, `roll (ϕ)`.

---

## 📁 Where to Find This
- EKF design: `ekf_scripts/`
- EKF Simulink model: `ekf_estimator/`
- Result plots: `simulink_models/plots/`
- Model details: `simulink_models/quadrotor_nonlinear_model.slx`

---



