function T_command = rotor_thrust_mapping(arm_length, yaw_coef, T_total, tau_phi, tau_theta, tau_psi, T_min, T_max)

    % --- Rotor Thrust Mapping ---
    A = [1          1             1          1;
         0          -arm_length   0          arm_length;
         arm_length   0           -arm_length  0;
         yaw_coef   -yaw_coef     yaw_coef   -yaw_coef];
    
    b = [T_total; tau_phi; tau_theta; tau_psi];
    T_command = A\b;
    
    % Thrust limits
    T_command = max(T_min, min(T_max, T_command));

end