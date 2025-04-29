% linearize_quadcopter_with_drag.m
clear; clc;

% Declare symbolic variables
syms x y z phi theta psi u v w p q r real  % States
syms U1 U2 U3 U4 real                      % Inputs
syms g m rho Cd Ax Ay Az Ix Iy Iz z_s real % Constants
syms eps_x eps_y eps_z real                % Drag damping terms





% State and input vectors
X = [x; y; z; phi; theta; psi; u; v; w; p; q; r];
U = [U1; U2; U3; U4];

% Define dynamics with LINEARIZED drag (symbolic damping)
xdot = [
    u*cos(theta)*cos(psi) + v*(cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi)) + w*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
    u*cos(theta)*sin(psi) + v*(sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi)) + w*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));
    -u*sin(theta) + v*cos(theta)*sin(phi) + w*cos(theta)*cos(phi);
    p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    q*cos(phi) - r*sin(phi);
    q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    g*sin(theta) - q*w + r*v - eps_x*u;
    -g*cos(theta)*sin(phi) - r*u + p*w - eps_y*v;
    -g*cos(theta)*cos(phi) + U1/m - p*v + q*u - eps_z*w;
    U2/Ix + q*r*(Iy - Iz)/Ix;
    U3/Iy + r*p*(Iz - Ix)/Iy;
    U4/Iz + p*q*(Ix - Iy)/Iz;
];



% Define hover equilibrium (hover at height z_s, flat orientation, 0 velocities)
X_eq = [0; 0; z_s; 0; 0; 0; 0; 0; 0; 0; 0; 0];
U_eq = [m*g; 0; 0; 0];

% Compute Jacobians
A = simplify(jacobian(xdot, X));
B = simplify(jacobian(xdot, U));

% Evaluate A and B at hover equilibrium
A_hover = simplify(subs(A, [X; U], [X_eq; U_eq]));
B_hover = simplify(subs(B, [X; U], [X_eq; U_eq]));
% Substitute damping terms numerically
% Display matrices
disp('✅ Linearized A matrix at hover with drag:');
pretty(A_hover)

disp('✅ Linearized B matrix at hover:');
pretty(B_hover)

m_val   = 0.547;
rho_val = 1.225;
Cd_val  = 1.0;
Ax_val  = 0.011;
Ay_val  = 0.011;
Az_val  = 0.022;

eps_x_val = (1/m_val)*rho_val*Cd_val*Ax_val;
eps_y_val = (1/m_val)*rho_val*Cd_val*Ay_val;
eps_z_val = (1/m_val)*rho_val*Cd_val*Az_val;

A_hover = subs(A_hover, {eps_x, eps_y, eps_z}, {eps_x_val, eps_y_val, eps_z_val});

% Also substitute remaining constants
g_val  = 9.81;
Ix_val = 3.3e-3;
Iy_val = 3.3e-3;
Iz_val = 5.8e-3;

A_hover = subs(A_hover, [g, m, Ix, Iy, Iz], [g_val, m_val, Ix_val, Iy_val, Iz_val]);
B_hover = subs(B_hover, [g, m, Ix, Iy, Iz], [g_val, m_val, Ix_val, Iy_val, Iz_val]);

% Convert to double matrices
A = double(A_hover);
B = double(B_hover);

% Display matrices
disp('Linearized A matrix at hover with drag:');
disp(A)

disp('Linearized B matrix at hover:');
disp(B)

% ---CHECK  Controllability ---

Co = ctrb(A, B);
rank_C = rank(Co);
fprintf('Rank of controllability matrix: %d\n', rank_C);
if rank_C == size(A,1)
    disp('The system is CONTROLLABLE ');
else
    disp('The system is NOT controllable');
end

% --- CHECK Observability ---

C = eye(size(A));

Ob = obsv(A, C);

rank_O = rank(Ob);
fprintf('Rank of observability matrix: %d\n', rank_O);
if rank_O == size(A,1)
    disp('The system is OBSERVABLE');
else
    disp('The system is NOT observable');
end

%define D matrix
D = zeros(12,4);

% LQR gain
Q = diag([10 10 10 10 10 10 1 1 1 0.5 0.5 0.5]);
R = 3 * eye(4);
K = lqr(A, B, Q, R);

% Check closed-loop eigenvalues after LQR design
eig_cl = eig(A - B*K);
if any(real(eig_cl) > 0)
    error('Controller makes system unstable!');
else
    disp('Closed-loop system is stable');
end

disp(K)

% Initial pitch disturbance (theta = 5 degrees)
% x0 = zeros(12,1);
% x0(5) = deg2rad(5);  % small pitch angle

% % Define initial conditions for different test cases
% deg = @(x) deg2rad(x); % helper
% 
% % CASE 1: Small pitch disturbance
% x0_pitch5 = zeros(12,1);
% x0_pitch5(5) = deg(5);  % theta = 5°
% 
% % CASE 2: Medium pitch disturbance
% x0_pitch15 = zeros(12,1);
% x0_pitch15(5) = deg2rad(15);  % theta = 15°
% 
% % CASE 3: Large pitch disturbance
% x0_pitch30 = zeros(12,1);
% x0_pitch30(5) = deg2rad(30);  % theta = 30°
% 
% % CASE 4: Roll disturbance
% x0_roll15 = zeros(12,1);
% x0_roll15(4) = deg2rad(15);  % phi = 15°
% 
% % CASE 5: Yaw disturbance
% x0_yaw20 = zeros(12,1);
% x0_yaw20(6) = deg2rad(20);  % psi = 20°
% 
% CASE 6: Position offset in x
% x0_xoffset = zeros(12,1);
% x0_xoffset(1) = 1;  % x = 1m
% 
% % CASE 7: Altitude offset
x0_zoffset = zeros(12,1);
x0_zoffset(3) = 10;  % z = 1m
% 
% % CASE 8: Angular velocity in pitch
% x0_q_spin = zeros(12,1);
% x0_q_spin(11) = 1;  % q = 1 rad/s (pitch rate)
