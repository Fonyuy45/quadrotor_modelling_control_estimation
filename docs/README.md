
| Variable | Description           | Unit      |
|----------|-----------------------|-----------|
| x, y, z  | Position in inertial frame {G} | m     |
| φ, θ, ψ  | Roll, Pitch, Yaw angles         | rad   |
| u, v, w  | Linear velocities in body frame | m/s   |
| p, q, r  | Angular velocities in body frame| rad/s |

---

## 🧪 Measured States
- **Position (x, y, z)** from motion capture or GPS
- **Angular velocities (p, q, r)** from IMU

---

## 🔍 Estimated States
The EKF estimates:
- **Full orientation (φ, θ, ψ)** from angular velocity and dynamics
- **Linear velocities (u, v, w)** not directly measurable
- Optionally: bias terms or external forces if modeled

---

## 📊 Noise Modeling
- **Process noise covariance (Q)**: captures model uncertainty
- **Measurement noise covariance (R)**: reflects sensor noise

---

## 🧰 EKF Implementation Notes
- EKF is implemented using Simulink blocks and custom MATLAB Function blocks.
- Key files:
  - `ekf_scripts/initialize_quadrotor_ekf.m`
  - `ekf_scripts/myStateTransitionFcn.m`
  - `ekf_scripts/myMeasurementFcn.m`
  - `ekf_estimator/final_ekf_simulation.slx`

---

## 📎 Use in Control
- The estimated state is fed back into the **LQR controller**.
- Simulation scenarios include **disturbance rejection**, **trajectory tracking**, and **noise robustness**.

