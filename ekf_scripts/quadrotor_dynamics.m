function dx = quadrotor_dynamics(u, x, F_wind_body)
    % Force column vectors
    u = reshape(u, [], 1);
    x = reshape(x, [], 1);
    F_wind_body = reshape(F_wind_body, [], 1);
    % Quadrotor nonlinear dynamics function
    % u: control inputs [4x1]
    % x: states [12x1]
    % F_wind_body: external wind force in body frame [3x1]
    
    % Physical parameters
    m = 0.547;     % mass [kg]
    g = 9.81;      % gravity [m/s^2]
    Cd = 1.0;      % drag coefficient
    Ix = 3.3e-3;   % inertia x
    Iy = 3.3e-3;   % inertia y
    Iz = 5.8e-3;   % inertia z
    Ax = 0.011;    % cross-section x
    Ay = 0.011;    % cross-section y
    Az = 0.022;    % cross-section z
    rho = 1.225;   % air density [kg/m^3]

    % State unpacking
    phi   = x(4); 
    theta = x(5); 
    psi   = x(6);
    u_b   = x(7); 
    v_b   = x(8); 
    w_b   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    
    % Wind acceleration
    a_wind_body = F_wind_body / m;
    
    % Saturate control inputs
    U1 = max(0, min(30*m*g, u(1))); % total thrust
    U2 = max(-10, min(10, u(2)));   % roll torque
    U3 = max(-10, min(10, u(3)));   % pitch torque
    U4 = max(-10, min(10, u(4)));   % yaw torque

    % Trigonometric helpers
    s = @sin; c = @cos; t = @tan;

    % Protection from division by 0
    eps_val = 1e-4;
    cos_theta_safe = max(eps_val, abs(c(theta)));
    
    dx = zeros(12,1);
    
    % Translational kinematics (Inertial frame)
    dx(1) = u_b*(c(theta)*c(psi)) + v_b*(s(phi)*s(theta)*c(psi) - c(phi)*s(psi)) + w_b*(c(phi)*s(theta)*c(psi) + s(phi)*s(psi));
    dx(2) = u_b*(c(theta)*s(psi)) + v_b*(s(phi)*s(theta)*s(psi) + c(phi)*c(psi)) + w_b*(c(phi)*s(theta)*s(psi) - s(phi)*c(psi));
    dx(3) = -u_b*s(theta) + v_b*s(phi)*c(theta) + w_b*c(phi)*c(theta);
    
    % Rotational kinematics
    if abs(theta) > 0.95*pi/2
        % Near singularity protection
        dx(4) = p;
        dx(5) = c(phi)*q - s(phi)*r;
        dx(6) = 0;
    else
        dx(4) = p + s(phi)*t(theta)*q + c(phi)*t(theta)*r;
        dx(5) = c(phi)*q - s(phi)*r;
        dx(6) = s(phi)/cos_theta_safe*q + c(phi)/cos_theta_safe*r;
    end
    
    % Translational dynamics (Body frame)
    dx(7) = g*s(theta) - q*w_b + r*v_b - (rho*Cd*Ax*u_b*abs(u_b))/(2*m) + a_wind_body(1);
    dx(8) = -g*c(theta)*s(phi) - r*u_b + p*w_b - (rho*Cd*Ay*v_b*abs(v_b))/(2*m) + a_wind_body(2);
    dx(9) = -g*c(theta)*c(phi) + U1/m - p*v_b + q*u_b - (rho*Cd*Az*w_b*abs(w_b))/(2*m) + a_wind_body(3);

    % Rotational dynamics
    dx(10) = (U2 + (Iy - Iz)*q*r)/Ix;
    dx(11) = (U3 + (Iz - Ix)*r*p)/Iy;
    dx(12) = (U4 + (Ix - Iy)*p*q)/Iz;
end
