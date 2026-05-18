% ===========================================================
% Project   : Quadcopter Position Control
% Revision  : 
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
addpath(genpath('functions'));

%% Select controller
controller_type = "PID";   % "PID", "SMC", "INDI"

%% Flags for simulation scenario
flag.wind_active        = 1;
flag.sensor_active      = 1;
flag.slung_active       = 0;
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
params.pid.Kp_x = 0.1;     params.pid.Ki_x = 0.0;     params.pid.Kd_x = 0.15;
params.pid.Kp_y = 0.1;     params.pid.Ki_y = 0.0;     params.pid.Kd_y = 0.15;
params.pid.Kp_z = 8.0;     params.pid.Ki_z = 3.0;     params.pid.Kd_z = 5.0; 
params.pid.Kp_phi = 6.0;   params.pid.Kp_theta = 6.0; params.pid.Kp_psi = 2;   
params.pid.Kp_p = 12;      params.pid.Ki_p = 0.05;    params.pid.Kd_p = 0.01; 
params.pid.Kp_q = 12;      params.pid.Ki_q = 0.05;    params.pid.Kd_q = 0.01;
params.pid.Kp_r = 5;       params.pid.Ki_r = 0.1;     params.pid.Kd_r = 0.02;
params.pid.int_lim = 0.5;   

%% SMC gains
params.smc.lambda_phi = 10.0;      params.smc.K_phi = 12.0;          
params.smc.lambda_theta = 10.0;    params.smc.K_theta = 12.0;        
params.smc.lambda_psi = 6.0;       params.smc.K_psi = 5.0;         
params.smc.lambda_xy = 0.45;       params.smc.K_xy = 0.5; 
params.smc.lambda_z = 1.5;         params.smc.K_z = 6;

%% INDI gains
params.indi.Kp_phi = 5;   params.indi.Kp_theta = 5;  params.indi.Kp_psi = 6;  
params.indi.Kp_p = 12;    params.indi.Ki_p = 0.05;   params.indi.Kd_p = 0.01;   
params.indi.Kp_q = 12;    params.indi.Ki_q = 0.05;   params.indi.Kd_q = 0.01;    
params.indi.Kp_r = 8;     params.indi.Ki_r = 0.1;    params.indi.Kd_r = 0.02;    
params.indi.Kp_x = 0.08;  params.indi.Kd_x = 0.12;
params.indi.Kp_y = 0.08;  params.indi.Kd_y = 0.12;
params.indi.Kp_z = 5;     params.indi.Kd_z = 3;
params.indi.int_lim = 0.5;   

%%  Slung load parameters
slung.mL = 0.7;   % load mass (kg)
slung.L  = 0.5;    % cable length (m)
slung.d = 0.75;       % damping
slung.h = 0.025;   % slung connection position on the drone (m)

% State vector: alpha, dalpha, beta, dbeta
slung.state = [0; 0; 0; 0];

% Compensator gains
slung.Kp = 2; slung.Kd = 1;

params.total_mass = params.m + slung.mL*flag.slung_active;

switch controller_type
    case "PID"
        ctrl = PIDController(params);
    case "INDI"
        ctrl = INDIController(params);
    case "SMC"
        ctrl = SMCController(params);
    otherwise
        error("Unknown");
end

trajectory = 0;  % Waypoint: 0, Helix: 1
radius = 5;

%% States and variables
phi_des = 0; theta_des = 0; psi_des = 0;
x_des = 0;   y_des = 0;     z_des = 5.0;

% State vector: [x; dx; y; dy; z; dz; phi; p; theta; q; psi; r]
state_true = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % true state
state_measured = state_true; % measured state

% Thrust states
T_actual = [0; 0; 0; 0];  % actual motor thrusts
T_command = [0; 0; 0; 0]; % commanded motor thrusts
prev_T_actual = 0;

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
    0, 0, 8, 0;
];

% TO-DO: something else for waypoints...
waypoint_times = 0:8:40; 

%% --- Logging ---
log_init();

%% Simulation Loop

for i = 1:N
    %% Desired states
    current_time = (i-1)*dt;
    if trajectory == 0
       [x_des, y_des, z_des, psi_des] = waypoint_trajectory(current_time, waypoints, waypoint_times);
    else
        [x_des, y_des, z_des, psi_des] = helix_trajectory(current_time, radius);
    end

    %% States without noise
    pos_true    = state_true([1, 3, 5]);        % [x, y, z]
    vel_true    = state_true([2, 4, 6]);        % [dx, dy, dz]
    angles_true = state_true([7, 9, 11]);       % [phi, theta, psi]
    rates_true  = state_true([8, 10, 12]);      % [p, q, r]
    
    %% Sensor Measurements
    pos_meas    = pos_true    + sensor.sigma_pos   * randn(3,1) * flag.sensor_active;
    vel_meas    = vel_true    + sensor.sigma_vel   * randn(3,1) * flag.sensor_active; 
    angles_meas = angles_true + sensor.sigma_angle * randn(3,1) * flag.sensor_active;
    rates_meas  = rates_true  + sensor.sigma_rate  * randn(3,1) * flag.sensor_active;
    state_measured = [pos_meas; vel_meas; angles_meas; rates_meas];
    
    %% Random wind forces and moments
    F_wind   = [wind.sigma_Fx; wind.sigma_Fy; wind.sigma_Fz]    .* randn(3,1) * flag.wind_active;
    tau_wind = [wind.sigma_tau; wind.sigma_tau; wind.sigma_tau] .* randn(3,1) * flag.wind_active;

    %% Swing Compensator
    if flag.slung_compensator == 1 && flag.slung_active == 1
        [phi_comp, theta_comp] = FcnQuad_slung_compensator(slung.state(1), slung.state(2), ...
                                           slung.state(3), slung.state(4), ...
                                           slung.L, slung.Kp, slung.Kd, params.g);
    else
        phi_comp = 0; theta_comp = 0;
    end

    %% Controller
    % Altitude Control
    T_total = ctrl.altitude(z_des, pos_meas(3), vel_meas(3), dt);
    
    % Position Control
    [phi_des, theta_des] = ctrl.position(x_des, y_des, pos_meas(1), pos_meas(2), vel_meas(1), vel_meas(2), dt);

    % Add compensator contribution
    theta_des = theta_des + theta_comp;     
    phi_des   = phi_des   + phi_comp;

    theta_des = max(-params.max_ang, min(params.max_ang, theta_des));
    phi_des   = max(-params.max_ang, min(params.max_ang, phi_des));
    
    % Attitude Control
    output = ctrl.attitude(phi_des, theta_des, psi_des, ...
                            angles_meas(1), angles_meas(2), angles_meas(3), ...
                            rates_meas(1), rates_meas(2), rates_meas(3), dt);

    % Thrust allocation
    T_command = ctrl.thrust_allocation(T_total, output, prev_T_actual);
    
    %% Motor Dynamics
    T_d = (T_command - T_actual) / params.tau_motor;
    T_actual = T_actual + T_d * dt;
    T_actual = max(params.T_min, min(params.T_max, T_actual));
    
    %% Slung Dynamics
    phi   = state_true(7);  p   = state_true(8);
    theta = state_true(9);  q   = state_true(10);
    psi   = state_true(11); r  = state_true(12);
    
    %  Compute inertial thrust
    Fz = sum(T_actual);
    R13 = -cos(phi)*sin(theta)*cos(psi) - sin(phi)*sin(psi);
    R23 = -cos(phi)*sin(theta)*sin(psi) + sin(phi)*cos(psi);
    R33 = cos(phi)*cos(theta);
    F_inertial = [R13; R23; R33] * Fz;
    
    %  Estimate acceleration
    a_est = (F_inertial + F_wind) / params.m - [0; 0; params.g];

    % Slung forces and moments
    if flag.slung_active == 1
        [slung.state, F_slung] = FcnQuad_slung_load_dynamics(slung.state, slung.L, slung.d, slung.mL, params.g, ...
            a_est(1), a_est(2), dt);
    else
        F_slung = [0; 0; 0];
        slung.state = [0; 0; 0; 0];
    end
    tau_slung = [-slung.h*F_slung(2); slung.h*F_slung(1); 0];
    
    % Calculate external forces and moments
    F_external   = F_wind + F_slung;
    tau_external = tau_wind + tau_slung;

    %% Quadcopter Dynamics
    state_true = FcnQuad_dynamics(state_true, T_actual, F_external, tau_external, dt, params);

    % Update INDI variables for next step
    prev_T_actual = T_actual;
    
    log_update();
end

log_and_plot();