clc; clear; close all
% Controller type 0: PID, 1: SMC
controller_type = 1;

% === Parameters ===
params.m = 1.0; % mass (kg)
params.g = 9.81; % gravity (m/s^2)
params.l = 0.25; % arm length (m)
params.Ixx = 0.02; 
params.Iyy = 0.02; 
params.Izz = 0.04; % inertia
params.c = 0.05; % yaw drag coefficient

% Motor dynamics parameters
params.tau_motor = 0.05; % motor time constant (s)
params.T_max = 15.0; % maximum motor thrust (N)
params.T_min = 0.0; % minimum motor thrust (N)

% Sensor noise parameters
sensor.sigma_z = 0.02 * randn(); % altitude measurement noise std dev (m)
sensor.sigma_dz = 0.01 * randn(); % velocity measurement noise std dev (m/s)
sensor.sigma_angle = 0.005 * randn(); % angle measurement noise std dev (rad)
sensor.sigma_rate = 0.01 * randn(); % angular rate measurement noise std dev (rad/s)

% PID gains
pid.Kp_z = 6.0;     pid.Ki_z = 1.0;     pid.Kd_z = 3.0; % altitude
pid.Kp_phi = 4.0;   pid.Kd_phi = 1.5; % roll
pid.Kp_theta = 4.0; pid.Kd_theta = 1.5; % pitch
pid.Kp_psi = 1.0;   pid.Kd_psi = 0.5; % yaw

% Sliding Mode Control parameters
smc.lambda_z = 2.5;       % Altitude SMC parameter
smc.K_z = 1.5;            % Altitude SMC gain
smc.lambda_phi = 5.0;     % Roll SMC parameter
smc.K_phi = 2.0;          % Roll SMC gain
smc.lambda_theta = 5.0;   % Pitch SMC parameter
smc.K_theta = 2.0;        % Pitch SMC gain
smc.lambda_psi = 3.0;     % Yaw SMC parameter
smc.K_psi = 1.0;          % Yaw SMC gain

% Desired states
z_des = 5.0; % desired altitude (m)
phi_des = 0.1; theta_des = -0.1; psi_des = 0; % desired angles (rad)

% === Simulation setup ===
dt = 0.01; T_end = 10;
N = floor(T_end/dt);

% State vector: [z; dz; phi; p; theta; q; psi; r]
state_true = [0; 0; 0; 0; 0; 0; 0; 0]; % true state
state_measured = state_true; % measured state (with noise)

% Motor states (actual thrust values with dynamics)
T_actual = [0; 0; 0; 0]; % actual motor thrusts
T_command = [0; 0; 0; 0]; % commanded motor thrusts

int_ez = 0;

% Logging
log.z_log = zeros(1,N);
log.z_measured_log = zeros(1,N);
log.phi_log = zeros(1,N);
log.theta_log = zeros(1,N);
log.psi_log = zeros(1,N);
log.T1_log = zeros(1,N);
log.T2_log = zeros(1,N);
log.T3_log = zeros(1,N);
log.T4_log = zeros(1,N);
log.time = (0:N-1)*dt;

% === Simulation loop ===
for i = 1:N
    % Extract true state
    z_true = state_true(1);     dz_true = state_true(2);
    phi_true = state_true(3);   p_true = state_true(4);
    theta_true = state_true(5); q_true = state_true(6);
    psi_true = state_true(7);   r_true = state_true(8);
    
    % === Sensor measurements with noise ===
    z_meas = z_true + sensor.sigma_z;
    dz_meas = dz_true + sensor.sigma_dz;
    phi_meas = phi_true + sensor.sigma_angle;
    p_meas = p_true + sensor.sigma_rate;
    theta_meas = theta_true + sensor.sigma_angle;
    q_meas = q_true + sensor.sigma_rate;
    psi_meas = psi_true + sensor.sigma_angle;
    r_meas = r_true + sensor.sigma_rate;
    
    % Store measured state
    state_measured = [z_meas; dz_meas; phi_meas; p_meas; theta_meas; q_meas; psi_meas; r_meas];
    
    if controller_type == 0
        % === PID Control ===
        % Altitude control
        ez = z_des - z_meas; 
        dez = -dz_meas;
        int_ez = int_ez + ez*dt;
        T_total = params.m*(params.g + pid.Kp_z*ez + pid.Kd_z*dez + pid.Ki_z*int_ez);
        
        % Attitude errors
        e_phi = phi_des - phi_meas; de_phi = -p_meas;
        e_theta = theta_des - theta_meas; de_theta = -q_meas;
        e_psi = psi_des - psi_meas; de_psi = -r_meas;
        
        % Torques
        tau_phi = params.Ixx*(pid.Kp_phi*e_phi + pid.Kd_phi*de_phi);
        tau_theta = params.Iyy*(pid.Kp_theta*e_theta + pid.Kd_theta*de_theta);
        tau_psi = params.Izz*(pid.Kp_psi*e_psi + pid.Kd_psi*de_psi);
    elseif controller_type == 1
        % === Sliding Mode Control ===
        % Altitude control
        ez = z_des - z_meas;
        dez = -dz_meas;
        sz = smc.lambda_z*ez + dez;
        T_total = params.m*(params.g + smc.lambda_z*dez + smc.K_z*sign(sz));
        
        % Attitude control
        % Roll control
        e_phi = phi_des - phi_meas;
        de_phi = -p_meas;
        s_phi = smc.lambda_phi*e_phi + de_phi;
        tau_phi = params.Ixx*(smc.lambda_phi*de_phi + smc.K_phi*sat(s_phi, 0.1)); 
        
        % Pitch control
        e_theta = theta_des - theta_meas;
        de_theta = -q_meas;
        s_theta = smc.lambda_theta*e_theta + de_theta;
        tau_theta = params.Iyy*(smc.lambda_theta*de_theta + smc.K_theta*sat(s_theta, 0.1));
        
        % Yaw control
        e_psi = psi_des - psi_meas;
        de_psi = -r_meas;
        s_psi = smc.lambda_psi*e_psi + de_psi;
        tau_psi = params.Izz*(smc.lambda_psi*de_psi + smc.K_psi*sat(s_psi, 0.1));

    end
        
    % === Rotor Thrust Mapping ===
    % Solve for commanded T1, T2, T3, T4 using linear equations
    A = [1          1             1          1;
         0          -params.l     0          params.l;
         params.l   0             -params.l  0;
         params.c   -params.c     params.c   -params.c];

    b = [T_total; tau_phi; tau_theta; tau_psi];
    T_command = A\b; % commanded rotor thrusts
    
    % Enforce thrust limits on commands
    T_command = max(params.T_min, min(params.T_max, T_command));
    
    % === Motor Dynamics ===
    % First-order dynamics: T_dot = (T_command - T_actual) / params.tau_motor
    dT_dt = (T_command - T_actual) / params.tau_motor;
    T_actual = T_actual + dT_dt * dt;
    
    % Enforce physical thrust limits on actual values
    T_actual = max(params.T_min, min(params.T_max, T_actual));
    
    % Compute net vertical thrust using actual motor outputs
    Fz = sum(T_actual); % body-frame z thrust
    
    % Compute actual torques using actual motor thrusts
    tau_phi_actual = params.l * (-T_actual(2) + T_actual(4));
    tau_theta_actual = params.l * (T_actual(1) - T_actual(3));
    tau_psi_actual = params.c * (T_actual(1) - T_actual(2) + T_actual(3) - T_actual(4));
    
    % === Dynamics (using true physics) ===
    % Translational acceleration (z only)
    az = Fz/params.m - params.g;
    
    % Rotational accelerations
    dp = tau_phi_actual / params.Ixx;
    dq = tau_theta_actual / params.Iyy;
    dr = tau_psi_actual / params.Izz;
    
    % Integrate true state
    dz_new = dz_true + az*dt; 
    z_new = z_true + dz_new*dt;
    p_new = p_true + dp*dt; 
    phi_new = phi_true + p_new*dt;
    q_new = q_true + dq*dt; 
    theta_new = theta_true + q_new*dt;
    r_new = r_true + dr*dt; 
    psi_new = psi_true + r_new*dt;
    
    % Store true state
    state_true = [z_new; dz_new; phi_new; p_new; theta_new; q_new; psi_new; r_new];
    
    % Logging
    log.z_log(i) = z_true;
    log.z_measured_log(i) = z_meas;
    log.phi_log(i) = phi_true;
    log.theta_log(i) = theta_true;
    log.psi_log(i) = psi_true;
    log.T1_log(i) = T_actual(1);
    log.T2_log(i) = T_actual(2);
    log.T3_log(i) = T_actual(3);
    log.T4_log(i) = T_actual(4);
end

% === Plotting ===
figure('Position', [100, 100, 1200, 800]);

% Altitude response with noise
subplot(3,2,1);
plot(log.time, log.z_log, 'b', 'LineWidth', 1.5); hold on;
plot(log.time, log.z_measured_log, 'b--', 'LineWidth', 1.0, 'Color', [0.5 0.5 1]);
plot(log.time, z_des*ones(size(log.time)), 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Altitude z (m)');
title('Altitude Response');
legend('True', 'Measured', 'Desired', 'Location', 'southeast');
grid on;

% Attitude response
subplot(3,2,2);
plot(log.time, log.phi_log, 'r', log.time, log.theta_log, 'g', log.time, log.psi_log, 'm', 'LineWidth', 1.5);
hold on;
plot(log.time, phi_des*ones(size(log.time)), 'r--', 'LineWidth', 1.0);
plot(log.time, theta_des*ones(size(log.time)), 'g--', 'LineWidth', 1.0);
plot(log.time, psi_des*ones(size(log.time)), 'm--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('Roll \phi', 'Pitch \theta', 'Yaw \psi', 'Location', 'northeast');
title('Attitude Response');
grid on;

% Motor thrust outputs
subplot(3,2,3);
plot(log.time, log.T1_log, 'r', log.time, log.T2_log, 'g', log.time, log.T3_log, 'b', log.time, log.T4_log, 'm', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Thrust (N)');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');
title('Motor Thrust Outputs');
grid on;

% Total thrust
subplot(3,2,4);
T_total_log = log.T1_log + log.T2_log + log.T3_log + log.T4_log;
plot(log.time, T_total_log, 'k', 'LineWidth', 1.5);
hold on;
plot(log.time, params.m*params.g*ones(size(log.time)), 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Total Thrust (N)');
legend('Actual', 'Hover Thrust', 'Location', 'southeast');
title('Total Thrust');
grid on;

% Altitude error
subplot(3,2,5);
altitude_error = z_des - log.z_log;
plot(log.time, altitude_error, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude Error (m)');
title('Altitude Tracking Error');
grid on;

% Attitude errors
subplot(3,2,6);
phi_error = phi_des - log.phi_log;
theta_error = theta_des - log.theta_log;
psi_error = psi_des - log.psi_log;
plot(log.time, phi_error, 'r', log.time, theta_error, 'g', log.time, psi_error, 'm', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle Error (rad)');
legend('Roll Error', 'Pitch Error', 'Yaw Error');
title('Attitude Tracking Errors');
grid on;

% Performance metrics
% fprintf('\n=== Performance Metrics ===\n');
% fprintf('Final altitude error: %.3f m\n', abs(z_des - z_log(end)));
% fprintf('Final roll error: %.3f rad (%.1f deg)\n', abs(phi_des - phi_log(end)), abs(phi_des - phi_log(end))*180/pi);
% fprintf('Final pitch error: %.3f rad (%.1f deg)\n', abs(theta_des - theta_log(end)), abs(theta_des - theta_log(end))*180/pi);
% fprintf('Final yaw error: %.3f rad (%.1f deg)\n', abs(psi_des - psi_log(end)), abs(psi_des - psi_log(end))*180/pi);
% fprintf('RMS altitude error: %.3f m\n', sqrt(mean(altitude_error.^2)));
% fprintf('RMS roll error: %.3f rad (%.1f deg)\n', sqrt(mean(phi_error.^2)), sqrt(mean(phi_error.^2))*180/pi);

sgtitle('Quadcopter Simulation');

% Saturation function to reduce chattering
function out = sat(in, boundary)
    if abs(in) <= boundary
        out = in/boundary;
    else
        out = sign(in);
    end
end