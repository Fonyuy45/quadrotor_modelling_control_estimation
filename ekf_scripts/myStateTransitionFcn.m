function xNext = myStateTransitionFcn(x, u)
    % State transition function for quadrotor EKF
    % x: Current state [12×1]
    % u: Combined [4×1] controls + [3×1] wind forces
    
    % At the beginning of myStateTransitionFcn
    if any(isnan(x)) || any(isinf(x))
        xNext = x; % Return unchanged state if input is invalid
        return;
    end
    
    % Split inputs
    u_ctrl = u(1:4);
    F_wind = u(5:7);
    
    % Time step
    dt = 0.01; % Matches your simulation step
    
    % Perform RK4 integration of full nonlinear dynamics
    k1 = quadrotor_dynamics(u_ctrl, x, F_wind);
    % Check for invalid intermediate results
    if any(isnan(k1)) || any(isinf(k1))
        k1 = zeros(size(k1));
    end
    k2 = quadrotor_dynamics(u_ctrl, x + dt*k1/2, F_wind);
    if any(isnan(k2)) || any(isinf(k2))
        k2 = zeros(size(k2));
    end
    k3 = quadrotor_dynamics(u_ctrl, x + dt*k2/2, F_wind);
    if any(isnan(k3)) || any(isinf(k3))
        k3 = zeros(size(k3));
    end
    k4 = quadrotor_dynamics(u_ctrl, x + dt*k3, F_wind);
    if any(isnan(k4)) || any(isinf(k4))
        k4 = zeros(size(k4));
    end
    
    xNext = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
    
    % Apply optional state constraints (for safety)
    xNext(1) = min(max(xNext(1), -50), 50); % x
    xNext(2) = min(max(xNext(2), -50), 50); % y
    xNext(3) = min(max(xNext(3), 0), 50); % z
    xNext(4) = min(max(xNext(4), -pi/4), pi/4);
    xNext(5) = min(max(xNext(5), -pi/4), pi/4);
    xNext(6) = min(max(xNext(6), -pi), pi);
   
    xNext(7:9) = min(max(xNext(7:9), -10), 10);
    xNext(10:12) = min(max(xNext(10:12), -5), 5);
    
    % In myStateTransitionFcn, add:
    dPhi = xNext(4) - x(4);
    if abs(dPhi) > 0.05  % Even tighter constraint for roll
        xNext(4) = x(4) + sign(dPhi) * 0.05;
    end
    
    dTheta = xNext(5) - x(5);
    if abs(dTheta) > 0.05  % Same for pitch
        xNext(5) = x(5) + sign(dTheta) * 0.05;
    end
end