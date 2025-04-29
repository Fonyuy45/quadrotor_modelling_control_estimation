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

%% --- Augmented System ---
A_aug = [A,           zeros(12,3);
         -C_pos,      zeros(3,3)];
B_aug = [B;
         zeros(3,4)];

C_aug = [eye(12), zeros(12, 3)];   % Output only x

D_aug = zeros(size(C_aug,1), size(B_aug,2));



%% --- LQR Design ---
Q = blkdiag(10*eye(3), 100*eye(3), eye(6), 100*eye(3));  % 15x15
R = 0.1 * eye(4);  % 4x4

K_aug = lqr(A_aug, B_aug, Q, R);
Kx = K_aug(:, 1:12);  % Original states
Ki = K_aug(:, 13:15); % Integral states

% Ki(3,1) = -31.6228 * 0.6;
Ki(1,3) = -31.6228/40;  % or even -1.0 to start


%% --- Save Gains for Simulink Use ---
save('LQR_Integral_Gains.mat', 'Kx', 'Ki');
disp('✅ LQR with integral action designed and saved.');
disp(Kx)
disp(Ki)


% x0_zoffset = zeros(12,1);
% x0_zoffset(3) = 1;  % z = 1m

x0 = zeros(12,1);           % or your desired initial state
x0_aug = [x0; 0; 0; 0];     % matches the 15x1 augmented system

r_pos = [10; 15; 20];

x0_zoffset = zeros(12,1);
x0_zoffset(3) = 10;  % z = 1m
x0_zoffset(1) = 10;
x0_zoffset(2) = 10;

%Ki(:,3) = [0.5; 0; 0; 0];

Q(3,3) = 50;


disturbance = zeros(12,1);
disturbance(9) = 1; % Force in body x-direction (u)


sineGain = zeros(12,1);
sineGain(7) = 0.5; % Applies to body y-direction velocity (v)

randomGain = zeros(12,1);
randomGain(9) = 0.1; % Applies to body z-direction velocity (w)

inputdis = [1;0;0;0];
% Increase weights for velocity states (7, 8, 9)
Q(7,7) = 50;  % Increase from 1 to 10 for u (x-velocity)
Q(8,8) = 50;  % Increase from 1 to 10 for v (y-velocity)
Q(9,9) = 10;  % Increase from 1 to 10 for w (z-velocity)


% Create coupling between Z position and velocity
Q(3,9) = 50; % Negative coupling between Z position and Z velocity
Q(9,3) = 100; % Symmetric term

Q(15,15) = 50;



%today

% Q(9,9) = 30;  % Increase W (vertical velocity) weight from 10 to 30
% 
% Q(15,15) = 150;  % Increase from 100 to 150 for stronger integral action on Z
% 
% R(1,1) = R(1,1) * 2;  % Double the cost on thrust input

