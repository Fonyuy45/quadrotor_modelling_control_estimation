% LQR with Integral Action - MATLAB Implementation
clear; clc;

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

disp(A)
disp(B)
%% --- Output Matrix for Position (x, y, z) ---
C_pos = zeros(3, 12);
C_pos(1,1) = 1;  % x
C_pos(2,2) = 1;  % y
C_pos(3,3) = 1;  % z

%% --- Setup Augmented System ---
A_aug = [A, zeros(12,3);
         -C_pos, zeros(3,3)];
B_aug = [B;
         zeros(3,4)];
C_aug = [eye(12), zeros(12, 3)]; % Output only x
D_aug = zeros(size(C_aug,1), size(B_aug,2));

%% --- Initial Reference and State Setup ---
r_pos = [1; 15; 12];  % Desired position [x; y; z]

% Initial conditions
x0 = zeros(12,1);
x0_aug = [x0; 0; 0; 0];  % Augmented state includes integral states

%% --- LQR Weight Matrix Design ---
% Initialize Q matrix with block diagonal structure
Q = zeros(15);

% Position states (1-3)
Q(1,1) = 200;  % Emphasize position in Xn
Q(2,2) = 200;  % Y position 
Q(3,3) = 1000;  % Z position

% Orientation states (4-6)
Q(4,4) = 100;
Q(5,5) = 100;
Q(6,6) = 100;

% Velocity states (7-12)
Q(7,7) = 20;  % X velocity
Q(8,8) = 20;  % Y velocity
Q(9,9) = 7000;  % Z velocity

% Angular velocity states (10-12)
Q(10,10) = 1;
Q(11,11) = 1;
Q(12,12) = 1;

% Integral states (13-15)
Q(13,13) = 10;  % X integral
Q(14,14) = 10;  % Y integral
Q(15,15) = 1;  % Z integral

% Cross-coupling terms
Q(1,7) = 5;   % Link X position and velocity
Q(7,1) = 5;
Q(3,9) = 5;  % Link Z position and velocity
Q(9,3) = 10;


%Q(3,3) = 200;
%Q(9,9) = 500;
% Control input weights
R = 0.1 * eye(4);  % All control inputs equally weighted

%% --- LQR Gain Calculation ---
% Calculate optimal gain matrices
[K_aug, ~, ~] = lqr(A_aug, B_aug, Q, R);

% Extract feedback and integral gains
Kx = K_aug(:, 1:12);    % State feedback gains
Ki = K_aug(:, 13:15);   % Integral gains



% Define your time step (match with EKF)
dt = 0.001; 

% Discretize the augmented system
[A_aug_d, B_aug_d] = c2d(A_aug, B_aug, dt);

% Calculate discrete LQR gains
[K_aug_d, ~, ~] = dlqr(A_aug_d, B_aug_d, Q, R);

% Extract feedback and integral gains
Kx = K_aug_d(:, 1:12); % Discrete state feedback gains
Ki = K_aug_d(:, 13:15); % Discrete integral gains

%% --- Manual Gain Adjustments (if needed) ---
% Uncomment and modify these lines only if you want to override LQR calculations
% Ki(1,3) = -31.6228/5;  % Adjust Z integral action on X control
% Ki(2,2) = 31.6228/5;   % Adjust Y integral action on Y control
% Ki(3,1) = 31.6228/5;   % Adjust X integral action on Z control

%% --- Save Gains for Simulink Use ---
save('LQR_Integral_Gains.mat', 'Kx', 'Ki');

%% --- Display Results ---
disp('✅ LQR with integral action designed and saved.');
disp('State Feedback Gains (Kx):');
disp(Kx);
disp('Integral Gains (Ki):');
%disp(Ki);

Ki = [
   0     0    -16
   0    17     0
   -14     0     0
   0     0     0
];

r_pos = [5; 8; 10];  % Desired position [x; y; z]

Ki


disturbance = zeros(12,1);
disturbance(9) = 1; % Force in body x-direction (u)
sineGain = zeros(12,1);
sineGain(7) = 0.5; % Applies to body y-direction velocity (v)
randomGain = zeros(12,1);
randomGain(8) = 1; % Applies to body z-direction velocity (w)
randomGain(7) = 1; % Applies to body z-direction velocity (w)
Kd = [
   0     0    1
   0    1     0
   1     0     0
   0     0     0
];

% Ki = [     0.0000   -0.0000  -10.5409
%    -0.0000   22.3607   -0.0000
%   -22.3607   -0.0000    0.0000
%     0.0000    0.0000    0.0000   
% 
% ]






% Create the EKF initial state (hovering position)
EKF_x0 = zeros(12, 1);

% Create var_vector explicitly as a row vector
var_vector = [0.1^2, 0.1^2, 0.1^2, 0.05^2, 0.05^2, 0.05^2, 0.2^2, 0.2^2, 0.2^2, 0.1^2, 0.1^2, 0.1^2];

% Check length
disp('Length of variance vector:');
disp(length(var_vector)); % Should be 12

% Create P0 manually as a diagonal matrix
P0 = zeros(12, 12);
for i = 1:12
    P0(i,i) = var_vector(i);
end

% Check size
disp('Size of P0 matrix:');
disp(size(P0)); % Should be [12 12]

% Vectorize
EKF_P0_vec = reshape(P0, [], 1);
disp('Size of vectorized P0:');
disp(size(EKF_P0_vec)); % Should be [144 1]

% Same approach for Q
q_vector = [0.5^2, 0.5^2, 0.5^2, 0.05^2, 0.05^2, 0.05^2, 0.05^2, 0.05^2, 0.05^2, 0.1^2, 0.1^2, 0.1^2];


% In your initialization, increase these values:
q_vector(7:9) = 0.2^2;   % Higher process noise for velocities
q_vector(10:12) = 0.3^2; % Higher process noise for angular velocities
q_vector(7:12) = 0.01^2; % Reduce process noise for velocities and angular velocities
Q = zeros(12, 12);
for i = 1:12
    Q(i,i) = q_vector(i);
end

Q = diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.1, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1]); 
disp('Size of Q matrix:');
disp(size(Q));

EKF_Q_vec = reshape(Q, [], 1);
disp('Size of vectorized Q:');
disp(size(EKF_Q_vec));

% And for R
% In your initialization function:
r_vector = [
    0.05^2, 0.05^2, 0.05^2;  
    0.005^2, 0.005^2, 0.005^2  % Angular velocity measurement noise
];

% Create R matrix

% Create each element individually to avoid any issues with line breaks or commas
r_vector = zeros(6, 1);
r_vector(1) = 0.05^2;  % x position noise
r_vector(2) = 0.05^2;  % y position noise
r_vector(3) = 0.05^2;  % z position noise
r_vector(4) = 0.01^2; % p angular velocity noise
r_vector(5) = 0.01^2; % q angular velocity noise
r_vector(6) = 0.015^2; % r angular velocity noise

% Check length of r_vector
disp('Length of r_vector:');
disp(length(r_vector)); % Should display 6

% Create diagonal R matrix
R = diag(r_vector);
% Ensure no zeros in measurement noise
R = diag(max(diag(R), 1e-10));
% R = diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.1, 0.1, 0.1]); % x,y,z,φ,θ,ψ,p,q,r
% Check size of R matrix
disp('Size of R matrix:');
disp(size(R)); % Should display [6 6]

% Vectorize R for Simulink
R_vec = reshape(R, [], 1);

% Check size of vectorized R
disp('Size of vectorized R:');
disp(size(R_vec)); % Should display [36 1]



EKF_R_vec = reshape(R, [], 1);
disp('Size of vectorized R:');
disp(size(EKF_R_vec));

% Set time step
EKF_dt = 0.001;

% Assign to base workspace
assignin('base', 'EKF_x0', EKF_x0);
assignin('base', 'EKF_P0_vec', EKF_P0_vec);
assignin('base', 'EKF_Q_vec', EKF_Q_vec);
assignin('base', 'EKF_R_vec', EKF_R_vec);
assignin('base', 'EKF_dt', EKF_dt);