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