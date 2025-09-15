% ===========================================================
% Project   : Quadcopter Position Control
% Revision  : Slung load added
% ===========================================================
% Description:
% This script implements position control for a quadcopter
% using different algorithms.
%
% Features:
% - 3D position tracking
% - PID, SMC and INDI controllers
% - Slung load dynamics and compensation

clc; clear; close all
addpath('functions');

%% Select controller type --- PID: 0,   SMC: 1,   INDI: 2
controller_type = 0;

trajectory = 0;  % Waypoint: 0, Helix: 1
radius = 5;

%% Flags for simulation scenario
flag.wind_active        = 0;
flag.sensor_active      = 0;
flag.slung_active       = 1;
flag.slung_compensator  = 0;

%% Drone Parameters 
params.m = 1.0;  % mass (kg)
params.g = 9.81; % gravity (m/s2)
params.l = 0.25; % arm length (m)
params.Ixx = 0.02;  params.Iyy = 0.02;  params.Izz = 0.04; % inertia
params.c = 0.05; % yaw drag coefficient

% Motor dynamics parameters
params.tau_motor = 0.05; % motor time constant (s)
params.T_max = 15.0; 
params.T_min = 0.0; 

% Maximum desired angle in rad
params.max_ang = 40*pi/180;

%% Sensor noise parameters
sensor.sigma_pos = 0.01;    % position measurement noise std dev (m)
sensor.sigma_vel = 0.05;    % velocity measurement noise std dev (m/s)
sensor.sigma_angle = 0.01;  % angle measurement noise std dev (rad)
sensor.sigma_rate = 0.01;   % angular rate measurement noise std dev (rad/s)

%% Wind disturbance parameters
wind.sigma_Fx = 0.5;   
wind.sigma_Fy = 0.5;   
wind.sigma_Fz = 0;   
wind.sigma_tau = 0.05; % Torque disturbance (Nm)

%% PID gains
pid.Kp_x = 0.1;     pid.Ki_x = 0.0;     pid.Kd_x = 0.15;
pid.Kp_y = 0.1;     pid.Ki_y = 0.0;     pid.Kd_y = 0.15;
pid.Kp_z = 8.0;     pid.Ki_z = 3.0;     pid.Kd_z = 5.0; 
pid.Kp_phi = 6.0;   pid.Kp_theta = 6.0; pid.Kp_psi = 2;   
pid.Kp_p = 12;      pid.Ki_p = 0.05;    pid.Kd_p = 0.01; 
pid.Kp_q = 12;      pid.Ki_q = 0.05;    pid.Kd_q = 0.01;
pid.Kp_r = 5;       pid.Ki_r = 0.1;     pid.Kd_r = 0.02;
pid.int_lim = 0.5;   

%% SMC gains
smc.lambda_phi = 10.0;      smc.K_phi = 12.0;          
smc.lambda_theta = 10.0;    smc.K_theta = 12.0;        
smc.lambda_psi = 6.0;       smc.K_psi = 5.0;         
smc.lambda_xy = 0.45;       smc.K_xy = 0.5; 
smc.lambda_z = 1.5;         smc.K_z = 6;

%% INDI gains
indi.Kp_phi = 5;   indi.Kp_theta = 5;  indi.Kp_psi = 6;  
indi.Kp_p = 12;    indi.Ki_p = 0.05;   indi.Kd_p = 0.01;   
indi.Kp_q = 12;    indi.Ki_q = 0.05;   indi.Kd_q = 0.01;    
indi.Kp_r = 8;     indi.Ki_r = 0.1;    indi.Kd_r = 0.02;    
indi.Kp_x = 0.08;  indi.Kd_x = 0.12;
indi.Kp_y = 0.08;  indi.Kd_y = 0.12;
indi.Kp_z = 5;     indi.Kd_z = 3;

%%  Slung load parameters
slung.mL = 0.7;   % load mass (kg)
slung.L  = 0.5;    % cable length (m)
slung.d = 0.75;       % damping
slung.h = 0.025;   % slung connection position on the drone (m)

% State vector: alpha, dalpha, beta, dbeta
slung.state = [0; 0; 0; 0];

% Compensator gains
slung.Kp = 2; slung.Kd = 1;

%% States and variables
total_mass = params.m + slung.mL*flag.slung_active;

phi_des = 0; theta_des = 0; psi_des = 0;
x_des = 0; y_des = 0; z_des = 5.0;

% State vector: [x; dx; y; dy; z; dz; phi; p; theta; q; psi; r]
state_true = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % true state
state_measured = state_true; % measured state

% Thrust states
T_actual = [0; 0; 0; 0];  % actual motor thrusts
T_command = [0; 0; 0; 0]; % commanded motor thrusts
prev_T_actual = 0;

% INDI variables
prev_p = 0; prev_q = 0; prev_r = 0; 
prev_dz = 0; prev_T_total = 0;

% PID variables
integral_p = 0;    integral_q = 0;    integral_r = 0;
int_ex = 0;    int_ey = 0;    int_ez = 0; 
p_dot_prev  = 0; q_dot_prev  = 0; r_dot_prev  = 0;
az_f = 0; p_dot_f = 0; q_dot_f = 0; r_dot_f = 0;

%% --- Simulation setup ---
dt = 0.01; T_end = 40;
N = floor(T_end/dt);
t = 0:dt:T_end;

%%  Simulation loop 
% Waypoints x y z psi
waypoints = [
    0, 0, 5, 0;
    10, 0, 5, 0;
    10, 10, 10, 0;
    0, 10, 10, 0;
    0, 0, 8, 0
];

% TO-DO: something else for waypoints...
waypoint_times = 0:8:40; 

%% --- Logging ---
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
log.x_log = zeros(1,N);
log.y_log = zeros(1,N);
log.x_des_log = zeros(N,1);
log.y_des_log = zeros(N,1);
log.z_des_log = zeros(N,1);
log.accel_log = zeros(N,3);
log.accel_raw_log = zeros(N,3);
log.gyro_log = zeros(N,3);
log.alpha_log = zeros(1,N);
log.beta_log  = zeros(1,N);
log.phi_comp_log = zeros(1,N);
log.theta_comp_log = zeros(1,N);

for i = 1:N
    current_time = (i-1)*dt;
    
    if trajectory == 0
        % Waypoint
        if current_time < waypoint_times(2)
            % WP1
            x_des = waypoints(2,1);
            y_des = waypoints(2,2);
            z_des = waypoints(2,3);
            psi_des = waypoints(2,4);
        elseif current_time < waypoint_times(3)
            % WP2
            x_des = waypoints(3,1);
            y_des = waypoints(3,2);
            z_des = waypoints(3,3);
            psi_des = waypoints(3,4);
        elseif current_time < waypoint_times(4)
            % WP3
            x_des = waypoints(4,1);
            y_des = waypoints(4,2);
            z_des = waypoints(4,3);
            psi_des = waypoints(4,4);
        else
            % WP4
            x_des = waypoints(5,1);
            y_des = waypoints(5,2);
            z_des = waypoints(5,3);
            psi_des = waypoints(5,4);
        end
    else
        [x_des, y_des, z_des, psi_des] = helix_trajectory(current_time, radius);
    end

    log.x_des_log(i) = x_des;
    log.y_des_log(i) = y_des;
    log.z_des_log(i) = z_des;
    
    % Random wind forces and moments
    Fx_wind = wind.sigma_Fx         * randn() * flag.wind_active;
    Fy_wind = wind.sigma_Fy         * randn() * flag.wind_active;
    Fz_wind = wind.sigma_Fz         * randn() * flag.wind_active;          
    tau_phi_wind = wind.sigma_tau   * randn() * flag.wind_active;    
    tau_theta_wind = wind.sigma_tau * randn() * flag.wind_active; 
    tau_psi_wind = wind.sigma_tau   * randn() * flag.wind_active*0;

    % Extract true state
    x_true = state_true(1);      dx_true = state_true(2);
    y_true = state_true(3);      dy_true = state_true(4);
    z_true = state_true(5);      dz_true = state_true(6);
    phi_true = state_true(7);    p_true = state_true(8);
    theta_true = state_true(9);  q_true = state_true(10);
    psi_true = state_true(11);   r_true = state_true(12);
    
    % Sensor measurements
    x_meas = x_true         + (sensor.sigma_pos * randn() * flag.sensor_active);
    dx_meas = dx_true       + (sensor.sigma_vel * randn() * flag.sensor_active);
    y_meas = y_true         + (sensor.sigma_pos * randn() * flag.sensor_active);
    dy_meas = dy_true       + (sensor.sigma_vel * randn() * flag.sensor_active);
    z_meas = z_true         + (sensor.sigma_pos * randn() * flag.sensor_active);
    dz_meas = dz_true       + (sensor.sigma_vel * randn() * flag.sensor_active);
    phi_meas = phi_true     + (sensor.sigma_angle * randn() * flag.sensor_active);
    p_meas = p_true         + (sensor.sigma_rate * randn() * flag.sensor_active);
    theta_meas = theta_true + (sensor.sigma_angle * randn() * flag.sensor_active);
    q_meas = q_true         + (sensor.sigma_rate * randn() * flag.sensor_active);
    psi_meas = psi_true     + (sensor.sigma_angle * randn() * flag.sensor_active);
    r_meas = r_true;

    % Store measured state
    state_measured = [x_meas; dx_meas; y_meas; dy_meas; z_meas; dz_meas; phi_meas; p_meas; theta_meas; q_meas; psi_meas; r_meas];
    
    % --- Swing Compensator ---
    if flag.slung_compensator == 1 && flag.slung_active == 1
        [phi_comp, theta_comp] = slung_compensator(slung.state(1), slung.state(2), ...
                                           slung.state(3), slung.state(4), ...
                                           slung.L, slung.Kp, slung.Kd, params.g);
    else
        phi_comp = 0; theta_comp = 0;
    end

    switch controller_type
        
        case 0
                
            % --- PID Altitude Control ---
            ez = z_des - z_meas; 
            dez = -dz_meas;
            int_ez = int_ez + ez*dt;
            T_total = total_mass*(params.g + pid.Kp_z*ez + pid.Kd_z*dez + pid.Ki_z*int_ez);

            % --- PID Position Control ---
            % Position errors
            ex = x_des - x_meas;
            ey = y_des - y_meas;  

            % X-position control
            dex = -dx_meas;
            int_ex = int_ex + ex*dt;
            theta_des = -(pid.Kp_x*ex + pid.Kd_x*dex + pid.Ki_x*int_ex);
            
            % Y-position control
            dey = -dy_meas;
            int_ey = int_ey + ey*dt;
            phi_des = (pid.Kp_y*ey + pid.Kd_y*dey + pid.Ki_y*int_ey);

            % Add compensator contribution
            theta_des = theta_des + theta_comp;
            phi_des   = phi_des   + phi_comp;

            theta_des = max(-params.max_ang, min(params.max_ang, theta_des));
            phi_des   = max(-params.max_ang, min(params.max_ang, phi_des));
            
            % --- PID Attitude Control ---
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
           
            integral_p = max(-pid.int_lim, min(pid.int_lim, integral_p));
            integral_q = max(-pid.int_lim, min(pid.int_lim, integral_q));
            integral_r = max(-pid.int_lim, min(pid.int_lim, integral_r));

            % Compute torques
            tau_phi = params.Ixx * (pid.Kp_p*e_p + pid.Ki_p*integral_p + pid.Kd_p*( - p_meas));
            tau_theta = params.Iyy * (pid.Kp_q*e_q + pid.Ki_q*integral_q + pid.Kd_q*( - q_meas));
            tau_psi = params.Izz * (pid.Kp_r*e_r + pid.Ki_r*integral_r + pid.Kd_r*( - r_meas));
            
            % Thrust mapping
            T_command = rotor_thrust_mapping(params.l, params.c, T_total, tau_phi, tau_theta, tau_psi, params.T_min, params.T_max);

        case 1
            % --- SMC Altitude Control ---
            ez = z_des - z_meas;
            s_z = smc.lambda_z*ez + (-dz_meas);
            T_total = total_mass*(params.g + smc.K_z*sat(s_z, 0.2));
                   
            % Sliding surfaces
            ex = x_des - x_meas; 
            ey = y_des - y_meas;  
            s_x = smc.lambda_xy*ex + (-dx_meas);
            s_y = smc.lambda_xy*ey + (-dy_meas);
            
            % Control outputs
            theta_des = -(smc.K_xy*sat(s_x, 1));
            phi_des   =  (smc.K_xy*sat(s_y, 1));
                    
            % Keep inside tilt limits
            theta_des = max(-params.max_ang, min(params.max_ang, theta_des));
            phi_des   = max(-params.max_ang, min(params.max_ang, phi_des));       
                        
            % --- SMC Attitude Control ---
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

            % Thrust mapping
            T_command = rotor_thrust_mapping(params.l, params.c, T_total, tau_phi, tau_theta, tau_psi, params.T_min, params.T_max);

        case 2

            % --- INDI Altitude Control ---
            az_raw = (dz_meas - prev_dz)/dt;
            current_az = lpf(az_raw, az_f, 0.85);
            az_f = current_az;
            
            ez = z_des - z_meas;
            az_des = indi.Kp_z*ez + indi.Kd_z*(-dz_meas);
            delta_az = az_des - current_az;
            
            % Total thrust change
            delta_T_total = (total_mass) * delta_az / (cos(phi_meas)*cos(theta_meas));
            
            % Increase total thrust
            T_total = prev_T_total + delta_T_total;
            
            % --- INDI Position Control ---
            ex = x_des - x_meas;
            ey = y_des - y_meas;
            dex = -dx_meas;
            dey = -dy_meas;
            
            theta_des = -(indi.Kp_x*ex + indi.Kd_x*dex);
            phi_des = (indi.Kp_y*ey + indi.Kd_y*dey);

            theta_des = theta_des + theta_comp;
            phi_des   = phi_des   + phi_comp;

            theta_des = max(-params.max_ang, min(params.max_ang, theta_des));
            phi_des = max(-params.max_ang, min(params.max_ang, phi_des));
            
            % --- INDI Attitude Control ---
            p_des = indi.Kp_phi*(phi_des - phi_meas);
            q_des = indi.Kp_theta*(theta_des - theta_meas);
            r_des = indi.Kp_psi*(psi_des - psi_meas);
            
            p_dot_meas = (p_meas - prev_p)/dt;
            q_dot_meas = (q_meas - prev_q)/dt;
            r_dot_meas = (r_meas - prev_r)/dt;
            
            integral_p = max(-pid.int_lim, min(pid.int_lim, ...
                         integral_p + (p_des - p_meas)*dt));
            integral_q = max(-pid.int_lim, min(pid.int_lim, ...
                         integral_q + (q_des - q_meas)*dt));
            integral_r = max(-pid.int_lim, min(pid.int_lim, ...
                         integral_r + (r_des - r_meas)*dt));
            
            p_dot_des = indi.Kp_p*(p_des - p_meas) + indi.Ki_p*integral_p - indi.Kd_p*p_dot_f;
            q_dot_des = indi.Kp_q*(q_des - q_meas) + indi.Ki_q*integral_q - indi.Kd_q*q_dot_f;
            r_dot_des = indi.Kp_r*(r_des - r_meas) + indi.Ki_r*integral_r - indi.Kd_r*r_dot_f;
            
            % INDI increment
            delta_p_dot = p_dot_des - p_dot_meas;
            delta_q_dot = q_dot_des - q_dot_meas;
            delta_r_dot = r_dot_des - r_dot_meas;
            
            % Control effectiveness matrix
            G = [0, -params.l/params.Ixx, 0, params.l/params.Ixx; ...
                 params.l/params.Iyy, 0, -params.l/params.Iyy, 0; ...
                 params.c/params.Izz, -params.c/params.Izz, params.c/params.Izz, -params.c/params.Izz];
            
            delta_T_att = pinv(G) * [delta_p_dot; delta_q_dot; delta_r_dot]; 
            T_command = prev_T_actual + delta_T_att;
            sum_err = T_total - sum(T_command);
            T_command = T_command + (sum_err/4)*ones(4,1);
            
            T_command = max(params.T_min, min(params.T_max, T_command));
    end
    
    % --- Motor Dynamics ---
    % First-order dynamics
    T_d = (T_command - T_actual) / params.tau_motor;
    T_actual = T_actual + T_d * dt;
    
    % Thrust limits
    T_actual = max(params.T_min, min(params.T_max, T_actual));

    phi   = state_true(7);  p   = state_true(8);
    theta = state_true(9);  q   = state_true(10);
    psi   = state_true(11); r  = state_true(12);
    
    Fz = sum(T_actual);  % body z-thrust sum

    R13 = -cos(phi)*sin(theta)*cos(psi) - sin(phi)*sin(psi);
    R23 = -cos(phi)*sin(theta)*sin(psi) + sin(phi)*cos(psi);
    R33 = cos(phi)*cos(theta);

    Fx_thrust = R13 * Fz;
    Fy_thrust = R23 * Fz;
    Fz_thrust = R33 * Fz;

    F_inertial = [Fx_thrust; Fy_thrust; Fz_thrust];

    %% --- Estimate vehicle accel (without slung load) for pendulum input
    a_est = (F_inertial + [Fx_wind; Fy_wind; Fz_wind]) / params.m - [0; 0; params.g];

    %% --- Slung Dynamics
    if flag.slung_active == 1
        [slung.state, F_slung] = slung_load_dynamics(slung.state, slung.L, slung.d, slung.mL, params.g, ...
            a_est(1), a_est(2), dt);
    else
        F_slung = [0; 0; 0];
        slung.state = [0; 0; 0; 0];
    end
    tau_phi_slung = -slung.h*F_slung(2);
    tau_theta_slung = slung.h*F_slung(1);

    %% --- Translational acceleration (include slung)
    F_total = F_inertial + [Fx_wind; Fy_wind; Fz_wind] + F_slung;
    acc = F_total / total_mass - [0; 0; params.g];
    ax = acc(1); ay = acc(2); az = acc(3);

    %% --- Moments from slung load
    tau_phi_actual   = params.l*(-T_actual(2)+T_actual(4)) + tau_phi_wind + tau_phi_slung;
    tau_theta_actual = params.l*( T_actual(1)-T_actual(3)) + tau_theta_wind + tau_theta_slung;
    tau_psi_actual   = params.c*( T_actual(1)-T_actual(2)+T_actual(3)-T_actual(4)) + tau_psi_wind;
    
    %% --- Rotational accelerations
    dp = tau_phi_actual   / params.Ixx;
    dq = tau_theta_actual / params.Iyy;
    dr = tau_psi_actual   / params.Izz;
    
    %% --- Integrate translational & rotational states
    dx_new = state_true(2) + ax*dt; 
    x_new = state_true(1) + dx_new*dt;

    dy_new = state_true(4) + ay*dt; 
    y_new = state_true(3) + dy_new*dt;

    dz_new = state_true(6) + az*dt; 
    z_new = state_true(5) + dz_new*dt;
    
    p_new = p + dp*dt; 
    phi_new = phi + p_new*dt;

    q_new = q + dq*dt; 
    theta_new = theta + q_new*dt;

    r_new = r + dr*dt; 
    psi_new = psi + r_new*dt;
    
    %% --- Update states
    state_true = [x_new; dx_new; y_new; dy_new; z_new; dz_new; phi_new; p_new; theta_new; q_new; psi_new; r_new];

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
    log.x_log(i) = x_true;
    log.y_log(i) = y_true;
    log.accel_log(i,:) = [ax; ay; az];
    log.accel_raw_log(i,:) = [a_est(1); a_est(2); a_est(3)];
    log.gyro_log(i,:) = [p_new; q_new; r_new];
    log.alpha_log(i) = rad2deg(slung.state(1));
    log.beta_log(i)  = rad2deg(slung.state(3));
    log.phi_comp_log(i) = phi_comp;
    log.theta_comp_log(i) = theta_comp;
end

log_and_plot();