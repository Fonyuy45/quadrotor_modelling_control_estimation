% Extract actual states and reference trajectory from simulation
t = out.tout;
X = squeeze(out.X_states)'; % [N x 12]

% Get reference trajectory data (correctly handling the 3×1×N format)
r_data = squeeze(out.r_traj.Data); % Should become 3×N

% Actual position
x = X(:, 1);
y = X(:, 2);
z = X(:, 3);

% Reference position (transpose to match actual trajectory)
x_ref = r_data(1, :)';
y_ref = r_data(2, :)';
z_ref = r_data(3, :)';

% Plot 3D trajectory
figure;
plot3(x_ref, y_ref, z_ref, 'b--', 'LineWidth', 2); hold on;
plot3(x, y, z, 'r', 'LineWidth', 2);
legend('Reference Trajectory', 'Actual Trajectory');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('3D Helix-Spiral Trajectory Tracking Nonlinear with EKF');
grid on; axis equal; view(45, 30);


%2D Trajectory tracking

x_actual = X(:,1);
y_actual = X(:,2);
z_actual = X(:,3);

figure;
title('2D Plot of Trajectory Tracking with Kalman Filter under Limitted Band Noise')

subplot(3,1,1);
plot(t, x_ref, 'r--', t, x_actual, 'b'); ylabel('X (m)');
legend('ref', 'actual'); grid on;

subplot(3,1,2);
plot(t, y_ref, 'r--', t, y_actual, 'b'); ylabel('Y (m)');
legend('ref', 'actual'); grid on;

subplot(3,1,3);
plot(t, z_ref, 'r--', t, z_actual, 'b'); ylabel('Z (m)');
xlabel('Time (s)');
legend('ref', 'actual'); grid on;