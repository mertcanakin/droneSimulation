clc; clear; close all

% Select controller type --- PID: 0,   SMC: 1,   INDI: 2
controller_type = 2;

% === Parameters ===
params.m = 1.0; % mass (kg)
params.g = 9.81; % gravity (m/s2)
params.l = 0.25; % arm length (m)
params.Ixx = 0.02;  params.Iyy = 0.02;  params.Izz = 0.04; % inertia
params.c = 0.05; % yaw drag coefficient

% Motor dynamics parameters
params.tau_motor = 0.05; % motor time constant (s)
params.T_max = 15.0; 
params.T_min = 0.0; 

% Sensor noise parameters
sensor.sigma_z = 0.02; % altitude measurement noise std dev (m)
sensor.sigma_dz = 0.01; % velocity measurement noise std dev (m/s)
sensor.sigma_angle = 0.005; % angle measurement noise std dev (rad)
sensor.sigma_rate = 0.01; % angular rate measurement noise std dev (rad/s)

% Wind disturbance parameters
wind.sigma_Fz = 0.3;   % Vertical force disturbance (N)
wind.sigma_tau = 0.02; % Torque disturbance (Nm)

%% PID gains
pid.Kp_z = 8.0;     pid.Ki_z = 2.0;     pid.Kd_z = 6.0; 

% Outer loop 
pid.Kp_phi = 3.0;   pid.Kp_theta = 3.0;  pid.Kp_psi = 2;   

% Inner loop
pid.Kp_p = 6;    pid.Ki_p = 0.05;    pid.Kd_p = 0.01; 
pid.Kp_q = 6;    pid.Ki_q = 0.05;    pid.Kd_q = 0.01;
pid.Kp_r = 10;    pid.Ki_r = 0.1;     pid.Kd_r = 0.02;
pid.int_lim = 0.5;   

%% Sliding Mode Control parameters
smc.lambda_phi = 10.0;      smc.K_phi = 12.0;          
smc.lambda_theta = 10.0;    smc.K_theta = 12.0;        
smc.lambda_psi = 6.0;       smc.K_psi = 5.0;         

%% INDI parameters
indi.Kp_phi = 3;   indi.Kp_theta = 3;  indi.Kp_psi = 3;  
indi.Kp_p = 12;    indi.Ki_p = 0.05;   indi.Kd_p = 0.5;   
indi.Kp_q = 12;    indi.Ki_q = 0.05;   indi.Kd_q = 0.5;    
indi.Kp_r = 16;    indi.Ki_r = 0.1;    indi.Kd_r = 0.02;    

%% === Desired states ===
z_des = 5.0;
phi_des = 0; theta_des = 0; psi_des = 0;

%% === Simulation setup ===
dt = 0.01; T_end = 10;
N = floor(T_end/dt);

% State vector: [z; dz; phi; p; theta; q; psi; r]
state_true = [0; 0; 0; 0; 0; 0; 0; 0]; % true state
state_measured = state_true; % measured state

% Thrust states
T_actual = [0; 0; 0; 0];  % actual motor thrusts
T_command = [0; 0; 0; 0]; % commanded motor thrusts

% INDI variables
prev_p = 0; prev_q = 0; prev_r = 0; 

% PID variables
int_ez = 0;
integral_p = 0;
integral_q = 0;
integral_r = 0;

%% === Logging ===
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
log.phi_des_log = zeros(1,N);
log.theta_des_log = zeros(1,N);
log.psi_des_log = zeros(1,N);

%% === Simulation loop ===

for i = 1:N
    if i > 150 && i < 600 
        phi_des = 0.5;
        theta_des = 0.3;
        psi_des = 0.1;
    elseif  i>=600
        phi_des = -0.5;    
        theta_des = -0.3;
        psi_des = -0.1;
    end
    
    % Random wind forces and moments
    Fz_wind = wind.sigma_Fz * randn();          
    tau_phi_wind = wind.sigma_tau * randn();    
    tau_theta_wind = wind.sigma_tau * randn(); 
    tau_psi_wind = wind.sigma_tau * randn();  
    
    % Extract true state
    z_true = state_true(1);     dz_true = state_true(2);
    phi_true = state_true(3);   p_true = state_true(4);
    theta_true = state_true(5); q_true = state_true(6);
    psi_true = state_true(7);   r_true = state_true(8);
    
    % Sensor measurements
    z_meas = z_true + sensor.sigma_z * randn();
    dz_meas = dz_true + sensor.sigma_dz * randn();
    phi_meas = phi_true + sensor.sigma_angle * randn();
    p_meas = p_true + sensor.sigma_rate * randn();
    theta_meas = theta_true + sensor.sigma_angle * randn();
    q_meas = q_true + sensor.sigma_rate * randn();
    psi_meas = psi_true + sensor.sigma_angle * randn();
    r_meas = r_true + sensor.sigma_rate * randn();
    
    % Store measured state
    state_measured = [z_meas; dz_meas; phi_meas; p_meas; theta_meas; q_meas; psi_meas; r_meas];
    
    % === PID Altitude Control ===
    ez = z_des - z_meas; 
    dez = -dz_meas;
    int_ez = int_ez + ez*dt;
    T_total = params.m*(params.g + pid.Kp_z*ez + pid.Kd_z*dez + pid.Ki_z*int_ez);

    switch controller_type
        
        case 0
            % === PID Attitude Control ===
            % Outer loop: Angle control 
            p_des = pid.Kp_phi * (phi_des - phi_meas);
            q_des = pid.Kp_theta * (theta_des - theta_meas);
            r_des = pid.Kp_psi * (psi_des - psi_meas);
            
            % Inner loop: Rate control
            e_p = p_des - p_meas;
            e_q = q_des - q_meas;
            e_r = r_des - r_meas;
            
            % Integrator
            integral_p = integral_p + e_p * dt;
            integral_q = integral_q + e_q * dt;
            integral_r = integral_r + e_r * dt;
            
            % Anti-windup
            integral_p = max(-pid.int_lim, min(pid.int_lim, integral_p));
            integral_q = max(-pid.int_lim, min(pid.int_lim, integral_q));
            integral_r = max(-pid.int_lim, min(pid.int_lim, integral_r));
            
            % Compute torques
            tau_phi = params.Ixx * (pid.Kp_p*e_p + pid.Ki_p*integral_p + pid.Kd_p*( - p_meas));
            tau_theta = params.Iyy * (pid.Kp_q*e_q + pid.Ki_q*integral_q + pid.Kd_q*( - q_meas));
            tau_psi = params.Izz * (pid.Kp_r*e_r + pid.Ki_r*integral_r + pid.Kd_r*( - r_meas));
            
            % === Rotor Thrust Mapping ===
            A = [1          1             1          1;
                 0          -params.l     0          params.l;
                 params.l   0             -params.l  0;
                 params.c   -params.c     params.c   -params.c];
            
            b = [T_total; tau_phi; tau_theta; tau_psi];
            T_command = A\b;
            
            % Enforce thrust limits
            T_command = max(params.T_min, min(params.T_max, T_command));

        case 1
            % === SMC Attitude Control ===
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
            e_r = psi_des - psi_meas;
            de_psi = -r_meas;
            s_psi = smc.lambda_psi*e_r + de_psi;
            tau_psi = params.Izz*(smc.lambda_psi*de_psi + smc.K_psi*sat(s_psi, 0.1));
            
            % === Rotor Thrust Mapping ===
            % Solve for commanded T1, T2, T3, T4
            A = [1          1             1          1;
                 0          -params.l     0          params.l;
                 params.l   0             -params.l  0;
                 params.c   -params.c     params.c   -params.c];
            
            b = [T_total; tau_phi; tau_theta; tau_psi];
            T_command = A\b; % commanded rotor thrusts
            
            % Enforce thrust limits on commands
            T_command = max(params.T_min, min(params.T_max, T_command));

        case 2
            % === INDI Attitude Control ===
            % Outer loop: angle control
            p_des = indi.Kp_phi*(phi_des - phi_meas);
            q_des = indi.Kp_theta*(theta_des - theta_meas);
            r_des = indi.Kp_psi*(psi_des - psi_meas);
            
            % Inner loop: angular rate control 
            p_dot = (p_meas - prev_p)/dt;
            q_dot = (q_meas - prev_q)/dt;
            r_dot = (r_meas - prev_r)/dt;
            
            % Desired angular accelerations from PID control
            p_dot_des = indi.Kp_p*(p_des - p_meas) + indi.Ki_p*integral_p - indi.Kd_p*p_dot;
            q_dot_des = indi.Kp_q*(q_des - q_meas) + indi.Ki_q*integral_q - indi.Kd_q*q_dot;
            r_dot_des = indi.Kp_r*(r_des - r_meas) + indi.Ki_r*integral_r - indi.Kd_r*r_dot;
            
            % Update integral terms
            integral_p = integral_p + (p_des - p_meas)*dt;
            integral_q = integral_q + (q_des - q_meas)*dt;
            integral_r = integral_r + (r_des - r_meas)*dt;
            
            % INDI control increment
            delta_p_dot = p_dot_des - p_dot;
            delta_q_dot = q_dot_des - q_dot;
            delta_r_dot = r_dot_des - r_dot;
            
            % Control effectiveness matrix
            G = [0, -params.l/params.Ixx, 0, params.l/params.Ixx;
                 params.l/params.Iyy, 0, -params.l/params.Iyy, 0;
                 params.c/params.Izz, -params.c/params.Izz, params.c/params.Izz, -params.c/params.Izz];
            
            % Calculate required thrust increments for attitude
            delta_T_att = G \ [delta_p_dot; delta_q_dot; delta_r_dot];
            
            % Combine altitude and attitude control
            % Distribute base thrust equally to all motors
            T_base = T_total/4 * ones(4,1);
            
            % Update motor commands
            T_command = T_base + delta_T_att;
            
            % Enforce thrust limits on commands
            T_command = max(params.T_min, min(params.T_max, T_command));
    end
    
    % === Motor Dynamics ===
    % First-order dynamics
    T_d = (T_command - T_actual) / params.tau_motor;
    T_actual = T_actual + T_d * dt;
    
    % Thrust limits
    T_actual = max(params.T_min, min(params.T_max, T_actual));
    
    % Compute net vertical thrust
    Fz = sum(T_actual) + Fz_wind; 
    
    % Compute actual torques
    tau_phi_actual = params.l * (-T_actual(2) + T_actual(4)) + tau_phi_wind;
    tau_theta_actual = params.l * (T_actual(1) - T_actual(3)) + tau_theta_wind;
    tau_psi_actual = params.c * (T_actual(1) - T_actual(2) + T_actual(3) - T_actual(4)) + tau_psi_wind;
    
    % === Dynamics ===
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
    
    % Update INDI variables for next step
    prev_p = p_meas;
    prev_q = q_meas;
    prev_r = r_meas;
    prev_T_actual = T_actual;
    prev_dz = dz_meas;
    prev_T_total = sum(T_actual);
    
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
    log.phi_des_log(i) = phi_des;
    log.theta_des_log(i) = theta_des;
    log.psi_des_log(i) = psi_des;
end

figure('Name', 'Results', 'Units', 'centimeters', 'Position', [10 2 20 20])

t = tiledlayout(2,1);
% nexttile;
% plot(log.time, log.z_log, 'b', 'LineWidth', 1.5); hold on;
% plot(log.time, z_des*ones(size(log.time)), 'r--', 'LineWidth', 1.0);
% xlabel('Time (s)'); ylabel('Altitude (m)');
% legend('Actual', 'Desired', 'Location', 'best');

nexttile;
plot(log.time, rad2deg(log.phi_log), 'r', log.time, rad2deg(log.theta_log), 'g', log.time, rad2deg(log.psi_log), 'b', 'LineWidth', 1.5);
hold on;
plot(log.time, rad2deg(log.phi_des_log), 'r--', 'LineWidth', 1.0);
plot(log.time, rad2deg(log.theta_des_log), 'g--', 'LineWidth', 1.0);
plot(log.time, rad2deg(log.psi_des_log), 'b--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Angle (°)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'northeast');
grid minor

nexttile;
plot(log.time, log.T1_log, 'r', log.time, log.T2_log, 'g', log.time, log.T3_log, 'b', log.time, log.T4_log, 'm', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Thrust (N)');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Location', 'northeast');
grid minor
set(findobj(gcf,'type','axes'),'FontName', 'Arial', 'FontSize', 12);
arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))

t.TileSpacing = 'compact';
t.Padding = 'compact';

if controller_type == 0
    controller_name = 'PID';
elseif controller_type == 1
    controller_name = 'SMC';
elseif controller_type == 2
    controller_name = 'INDI';
end

sgtitle(['Results - ' controller_name ' Controller']);
exportgraphics(gcf,['results_' controller_name '.png'], 'Resolution', 600)

function out = sat(in, boundary)
    if abs(in) <= boundary
        out = in/boundary;
    else
        out = sign(in);
    end
end