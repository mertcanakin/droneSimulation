clc; clear;
% === Parameters ===
m = 1.0; % mass (kg)
g = 9.81; % gravity (m/s^2)
l = 0.25; % arm length (m)
Ixx = 0.02; Iyy = 0.02; Izz = 0.04; % inertia
c = 0.05; % yaw drag coefficient

% Motor dynamics parameters
tau_motor = 0.05; % motor time constant (s)
T_max = 15.0; % maximum motor thrust (N)
T_min = 0.0; % minimum motor thrust (N)

% Sensor noise parameters
sigma_z = 0.02; % altitude measurement noise std dev (m)
sigma_dz = 0.01; % velocity measurement noise std dev (m/s)
sigma_angle = 0.005; % angle measurement noise std dev (rad)
sigma_rate = 0.01; % angular rate measurement noise std dev (rad/s)

% PID gains
Kp_z = 6.0; Ki_z = 1.0; Kd_z = 3.0; % altitude
Kp_phi = 4.0; Kd_phi = 1.5; % roll
Kp_theta = 4.0; Kd_theta = 1.5; % pitch
Kp_psi = 1.0; Kd_psi = 0.5; % yaw

% Desired states
z_des = 5.0; % desired altitude (m)
phi_des = 0.1; theta_des = 0; psi_des = 0; % desired angles (rad)

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
z_log = zeros(1,N);
z_measured_log = zeros(1,N);
phi_log = zeros(1,N);
theta_log = zeros(1,N);
psi_log = zeros(1,N);
T1_log = zeros(1,N);
T2_log = zeros(1,N);
T3_log = zeros(1,N);
T4_log = zeros(1,N);
time = (0:N-1)*dt;

% === Simulation loop ===
for i = 1:N
    % Extract true state
    z_true = state_true(1); dz_true = state_true(2);
    phi_true = state_true(3); p_true = state_true(4);
    theta_true = state_true(5); q_true = state_true(6);
    psi_true = state_true(7); r_true = state_true(8);
    
    % === Sensor measurements with noise ===
    z_meas = z_true + sigma_z * randn();
    dz_meas = dz_true + sigma_dz * randn();
    phi_meas = phi_true + sigma_angle * randn();
    p_meas = p_true + sigma_rate * randn();
    theta_meas = theta_true + sigma_angle * randn();
    q_meas = q_true + sigma_rate * randn();
    psi_meas = psi_true + sigma_angle * randn();
    r_meas = r_true + sigma_rate * randn();
    
    % Store measured state
    state_measured = [z_meas; dz_meas; phi_meas; p_meas; theta_meas; q_meas; psi_meas; r_meas];
    
    % === PID Control (using measured states) ===
    % Altitude control
    ez = z_des - z_meas; 
    dez = -dz_meas;
    int_ez = int_ez + ez*dt;
    T_total = m*(g + Kp_z*ez + Kd_z*dez + Ki_z*int_ez);
    
    % Attitude errors
    e_phi = phi_des - phi_meas; de_phi = -p_meas;
    e_theta = theta_des - theta_meas; de_theta = -q_meas;
    e_psi = psi_des - psi_meas; de_psi = -r_meas;
    
    % Torques
    tau_phi = Ixx*(Kp_phi*e_phi + Kd_phi*de_phi);
    tau_theta = Iyy*(Kp_theta*e_theta + Kd_theta*de_theta);
    tau_psi = Izz*(Kp_psi*e_psi + Kd_psi*de_psi);
    
    % === Rotor Thrust Mapping ===
    % Solve for commanded T1, T2, T3, T4 using linear equations
    A = [1  1  1  1;
         0 -l  0  l;
         l  0 -l  0;
         c -c  c -c];
    b = [T_total; tau_phi; tau_theta; tau_psi];
    T_command = A\b; % commanded rotor thrusts
    
    % Enforce thrust limits on commands
    T_command = max(T_min, min(T_max, T_command));
    
    % === Motor Dynamics ===
    % First-order dynamics: T_dot = (T_command - T_actual) / tau_motor
    dT_dt = (T_command - T_actual) / tau_motor;
    T_actual = T_actual + dT_dt * dt;
    
    % Enforce physical thrust limits on actual values
    T_actual = max(T_min, min(T_max, T_actual));
    
    % Compute net vertical thrust using actual motor outputs
    Fz = sum(T_actual); % body-frame z thrust
    
    % Compute actual torques using actual motor thrusts
    tau_phi_actual = l * (-T_actual(2) + T_actual(4));
    tau_theta_actual = l * (T_actual(1) - T_actual(3));
    tau_psi_actual = c * (T_actual(1) - T_actual(2) + T_actual(3) - T_actual(4));
    
    % === Dynamics (using true physics) ===
    % Translational acceleration (z only)
    az = Fz/m - g;
    
    % Rotational accelerations
    dp = tau_phi_actual/Ixx;
    dq = tau_theta_actual/Iyy;
    dr = tau_psi_actual/Izz;
    
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
    z_log(i) = z_true;
    z_measured_log(i) = z_meas;
    phi_log(i) = phi_true;
    theta_log(i) = theta_true;
    psi_log(i) = psi_true;
    T1_log(i) = T_actual(1);
    T2_log(i) = T_actual(2);
    T3_log(i) = T_actual(3);
    T4_log(i) = T_actual(4);
end

% === Plotting ===
figure('Position', [100, 100, 1200, 800]);

% Altitude response with noise
subplot(3,2,1);
plot(time, z_log, 'b', 'LineWidth', 1.5); hold on;
plot(time, z_measured_log, 'b--', 'LineWidth', 1.0, 'Color', [0.5 0.5 1]);
plot(time, z_des*ones(size(time)), 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Altitude z (m)');
title('Altitude Response');
legend('True', 'Measured', 'Desired', 'Location', 'southeast');
grid on;

% Attitude response
subplot(3,2,2);
plot(time, phi_log, 'r', time, theta_log, 'g', time, psi_log, 'm', 'LineWidth', 1.5);
hold on;
plot(time, phi_des*ones(size(time)), 'r--', 'LineWidth', 1.0);
plot(time, theta_des*ones(size(time)), 'g--', 'LineWidth', 1.0);
plot(time, psi_des*ones(size(time)), 'm--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('Roll \phi', 'Pitch \theta', 'Yaw \psi', 'Location', 'northeast');
title('Attitude Response');
grid on;

% Motor thrust outputs
subplot(3,2,3);
plot(time, T1_log, 'r', time, T2_log, 'g', time, T3_log, 'b', time, T4_log, 'm', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Thrust (N)');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');
title('Motor Thrust Outputs');
grid on;

% Total thrust
subplot(3,2,4);
T_total_log = T1_log + T2_log + T3_log + T4_log;
plot(time, T_total_log, 'k', 'LineWidth', 1.5);
hold on;
plot(time, m*g*ones(size(time)), 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Total Thrust (N)');
legend('Actual', 'Hover Thrust', 'Location', 'southeast');
title('Total Thrust');
grid on;

% Altitude error
subplot(3,2,5);
altitude_error = z_des - z_log;
plot(time, altitude_error, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude Error (m)');
title('Altitude Tracking Error');
grid on;

% Attitude errors
subplot(3,2,6);
phi_error = phi_des - phi_log;
theta_error = theta_des - theta_log;
psi_error = psi_des - psi_log;
plot(time, phi_error, 'r', time, theta_error, 'g', time, psi_error, 'm', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle Error (rad)');
legend('Roll Error', 'Pitch Error', 'Yaw Error');
title('Attitude Tracking Errors');
grid on;

% Performance metrics
fprintf('\n=== Performance Metrics ===\n');
fprintf('Final altitude error: %.3f m\n', abs(z_des - z_log(end)));
fprintf('Final roll error: %.3f rad (%.1f deg)\n', abs(phi_des - phi_log(end)), abs(phi_des - phi_log(end))*180/pi);
fprintf('Final pitch error: %.3f rad (%.1f deg)\n', abs(theta_des - theta_log(end)), abs(theta_des - theta_log(end))*180/pi);
fprintf('Final yaw error: %.3f rad (%.1f deg)\n', abs(psi_des - psi_log(end)), abs(psi_des - psi_log(end))*180/pi);
fprintf('RMS altitude error: %.3f m\n', sqrt(mean(altitude_error.^2)));
fprintf('RMS roll error: %.3f rad (%.1f deg)\n', sqrt(mean(phi_error.^2)), sqrt(mean(phi_error.^2))*180/pi);

sgtitle('Enhanced Quadrotor Simulation with Motor Dynamics and Sensor Noise');