function state_new = FcnQuad_dynamics(state_true, T_actual, F_external, tau_external, dt, params)

    % Extract current states
    phi = state_true(7);   p = state_true(8);
    theta = state_true(9); q = state_true(10);
    psi = state_true(11);  r = state_true(12);
    
    %% Translational Dynamics
    % Compute inertial thrust
    Fz = sum(T_actual);
    R13 = -cos(phi)*sin(theta)*cos(psi) - sin(phi)*sin(psi);
    R23 = -cos(phi)*sin(theta)*sin(psi) + sin(phi)*cos(psi);
    R33 = cos(phi)*cos(theta);
    F_inertial = [R13; R23; R33] * Fz;
    
    % Total force and acceleration
    F_total = F_inertial + F_external;
    acc = F_total / params.total_mass - [0; 0; params.g];
    ax = acc(1); ay = acc(2); az = acc(3);
    
    %% Rotational Dynamics
    % Motor moments
    tau_phi_motor = params.l*(-T_actual(2)+T_actual(4));
    tau_theta_motor = params.l*(T_actual(1)-T_actual(3));
    tau_psi_motor = params.c*(T_actual(1)-T_actual(2)+T_actual(3)-T_actual(4));
    
    % Total moments (motor + external)
    tau_phi_actual = tau_phi_motor + tau_external(1);
    tau_theta_actual = tau_theta_motor + tau_external(2);
    tau_psi_actual = tau_psi_motor + tau_external(3);
    
    % Angular accelerations
    dp = tau_phi_actual / params.Ixx;
    dq = tau_theta_actual / params.Iyy;
    dr = tau_psi_actual / params.Izz;
    
    %% Integration
    % Translational states
    dx_new = state_true(2) + ax*dt;
    x_new = state_true(1) + dx_new*dt;
    
    dy_new = state_true(4) + ay*dt;
    y_new = state_true(3) + dy_new*dt;
    
    dz_new = state_true(6) + az*dt;
    z_new = state_true(5) + dz_new*dt;
    
    % Rotational states
    p_new = p + dp*dt;
    phi_new = phi + p_new*dt;
    
    q_new = q + dq*dt;
    theta_new = theta + q_new*dt;
    
    r_new = r + dr*dt;
    psi_new = psi + r_new*dt;
    
    %% Return updated state vector
    state_new = [x_new; dx_new; y_new; dy_new; z_new; dz_new; ...
                 phi_new; p_new; theta_new; q_new; psi_new; r_new];
end