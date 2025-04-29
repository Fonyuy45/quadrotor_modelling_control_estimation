function dx = quadrotor_dynamics_ekf(x, u)
    % Lightweight quadrotor dynamics for EKF
    % No wind considered during prediction

    F_wind_body = zeros(3,1);  % No wind during EKF prediction

    % Call the real dynamics function correctly
    dx = quadrotor_dynamics(u, x, F_wind_body);
end
