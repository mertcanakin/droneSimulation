clc; clear; close all

% Select controller type --- PID: 0,   SMC: 1,   INDI: 2
controller_type = 1;

% === Parameters ===
params.m = 1.0;  % mass (kg)
params.g = 9.81; % gravity (m/s2)
params.l = 0.25; % arm length (m)
params.Ixx = 0.02;  params.Iyy = 0.02;  params.Izz = 0.04; % inertia
params.c = 0.05; % yaw drag coefficient

% Motor dynamics parameters
params.tau_motor = 0.05; % motor time constant (s)
params.T_max = 15.0; 
params.T_min = 0.0; 

% Sensor noise parameters
sensor.sigma_x = 0.01; % x axis measurement noise std dev (m)
sensor.sigma_y = 0.01; % y axis measurement noise std dev (m)
sensor.sigma_z = 0.05; % altitude measurement noise std dev (m)
sensor.sigma_dx = 0.05; % x axis measurement noise std dev (m/s)
sensor.sigma_dy = 0.05; % y axis measurement noise std dev (m/s)
sensor.sigma_dz = 0.005; % velocity measurement noise std dev (m/s)
sensor.sigma_angle = 0.01; % angle measurement noise std dev (rad)
sensor.sigma_rate = 0.01; % angular rate measurement noise std dev (rad/s)

% Wind disturbance parameters
wind.sigma_Fx = 1;   % Vertical force disturbance (N)
wind.sigma_Fy = 1;   
wind.sigma_Fz = 0.3;   
wind.sigma_tau = 0.4; % Torque disturbance (Nm)

% Maximum desired angle in rad
max_tilt = 30*pi/180;

%% PID gains
pid.Kp_x = 0.2;     pid.Ki_x = 0.0;     pid.Kd_x = 0.2;
pid.Kp_y = 0.2;     pid.Ki_y = 0.0;     pid.Kd_y = 0.2;
pid.Kp_z = 5.0;     pid.Ki_z = 2.0;     pid.Kd_z = 3.0; 
pid.Kp_phi = 4.0;   pid.Kp_theta = 4.0; pid.Kp_psi = 8;   
pid.Kp_p = 12;      pid.Ki_p = 0.05;    pid.Kd_p = 0.01; 
pid.Kp_q = 12;      pid.Ki_q = 0.05;    pid.Kd_q = 0.01;
pid.Kp_r = 10;      pid.Ki_r = 0.1;     pid.Kd_r = 0.02;
pid.int_lim = 0.5;   

%% Sliding Mode Control parameters
smc.lambda_phi = 10.0;      smc.K_phi = 12.0;          
smc.lambda_theta = 10.0;    smc.K_theta = 12.0;        
smc.lambda_psi = 6.0;       smc.K_psi = 5.0;         
smc.lambda_xy = 0.6;        smc.K_xy = 0.5; 
smc.lambda_z = 1.5;         smc.K_z = 6;

%% INDI parameters
indi.Kp_phi = 5;  indi.Kp_theta = 5;  indi.Kp_psi = 6;  
indi.Kp_p = 12;   indi.Ki_p = 0.05;   indi.Kd_p = 0.01;   
indi.Kp_q = 12;   indi.Ki_q = 0.05;   indi.Kd_q = 0.01;    
indi.Kp_r = 8;   indi.Ki_r = 0.1;   indi.Kd_r = 0.02;    
indi.Kp_x = 0.2;  indi.Kd_x = 0.22;
indi.Kp_y = 0.2;  indi.Kd_y = 0.22;
indi.Kp_z = 5;    indi.Kd_z = 3;
alpha = 0.2;

%% === Desired states ===
phi_des = 0; theta_des = 0; psi_des = 0;
x_des = 0; y_des = 0; z_des = 5.0;

% State vector: [x; dx; y; dy; z; dz; phi; p; theta; q; psi; r]
state_true = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % true state
state_measured = state_true; % measured state

% Thrust states
T_actual = [0; 0; 0; 0];  % actual motor thrusts
T_command = [0; 0; 0; 0]; % commanded motor thrusts

% INDI variables
prev_p = 0; prev_q = 0; prev_r = 0; 
prev_dz = 0; prev_T_total = 0;

% PID variables
integral_p = 0;    integral_q = 0;    integral_r = 0;
int_ex = 0;    int_ey = 0;    int_ez = 0; 
p_dot_prev  = 0; q_dot_prev  = 0; r_dot_prev  = 0;

%% === Simulation setup ===
dt = 0.01; T_end = 40;
N = floor(T_end/dt);

%% === Simulation loop ===
% Waypoints x y z psi
waypoints = [
    0, 0, 5, 0;
    10, 0, 5, -pi/8;
    10, 10, 10, 0;
    0, 10, 10, pi/8;
    0, 0, 8, 0
];

% TO-DO: something else for waypoints...
waypoint_times = 0:8:40; 

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
log.x_log = zeros(1,N);
log.y_log = zeros(1,N);
log.x_des_log = zeros(N,1);
log.y_des_log = zeros(N,1);
log.z_des_log = zeros(N,1);

for i = 1:N
    current_time = (i-1)*dt;
    
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

    log.x_des_log(i) = x_des;
    log.y_des_log(i) = y_des;
    log.z_des_log(i) = z_des;
    
    if i >= N/3
        % Random wind forces and moments
        Fx_wind = wind.sigma_Fx * randn();
        Fy_wind = wind.sigma_Fy * randn();
        Fz_wind = wind.sigma_Fz * randn();          
        tau_phi_wind = wind.sigma_tau * randn();    
        tau_theta_wind = wind.sigma_tau * randn(); 
        tau_psi_wind = wind.sigma_tau * randn()/10;

    else
        Fx_wind = 0;
        Fy_wind = 0;
        Fz_wind = 0;          
        tau_phi_wind = 0;    
        tau_theta_wind = 0; 
        tau_psi_wind = 0;        
    end

    % Extract true state
    x_true = state_true(1);      dx_true = state_true(2);
    y_true = state_true(3);      dy_true = state_true(4);
    z_true = state_true(5);      dz_true = state_true(6);
    phi_true = state_true(7);    p_true = state_true(8);
    theta_true = state_true(9);  q_true = state_true(10);
    psi_true = state_true(11);   r_true = state_true(12);
    
    % Sensor measurements
    x_meas = x_true + sensor.sigma_x * randn();
    dx_meas = dx_true + sensor.sigma_dx * randn();
    y_meas = y_true + sensor.sigma_y * randn();
    dy_meas = dy_true + sensor.sigma_dy * randn();
    z_meas = z_true + sensor.sigma_z * randn();
    dz_meas = dz_true + sensor.sigma_dz * randn();
    phi_meas = phi_true + sensor.sigma_angle * randn();
    p_meas = p_true +sensor.sigma_rate * randn();
    theta_meas = theta_true + sensor.sigma_angle * randn();
    q_meas = q_true + sensor.sigma_rate * randn();
    psi_meas = psi_true + sensor.sigma_angle * randn();
    r_meas = r_true;

    % Store measured state
    state_measured = [x_meas; dx_meas; y_meas; dy_meas; z_meas; dz_meas; phi_meas; p_meas; theta_meas; q_meas; psi_meas; r_meas];
    
    switch controller_type
        
        case 0
                
            % === PID Altitude Control ===
            ez = z_des - z_meas; 
            dez = -dz_meas;
            int_ez = int_ez + ez*dt;
            T_total = params.m*(params.g + pid.Kp_z*ez + pid.Kd_z*dez + pid.Ki_z*int_ez);

            % === PID Position Control ===
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
            
            theta_des = max(-max_tilt, min(max_tilt, theta_des));
            phi_des = max(-max_tilt, min(max_tilt, phi_des));

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
            % === SMC Altitude Control ===
            ez = z_des - z_meas;
            s_z = smc.lambda_z*ez + (-dz_meas);
            T_total = params.m*(params.g + smc.K_z*sat(s_z, 0.2));
                   
            % Sliding surfaces
            ex = x_des - x_meas; 
            ey = y_des - y_meas;  
            s_x = smc.lambda_xy*ex + (-dx_meas);
            s_y = smc.lambda_xy*ey + (-dy_meas);
            
            % Control outputs
            theta_des = -(smc.K_xy*sat(s_x, 1));
            phi_des   =  (smc.K_xy*sat(s_y, 1));
            
            % Angle saturation
            theta_des = max(-max_tilt, min(max_tilt, theta_des));
            phi_des = max(-max_tilt, min(max_tilt, phi_des));
                        
            % === SMC Attitude Control ===
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
            % === INDI Altitude Control ===
            ez = z_des - z_meas;
            az_des = indi.Kp_z*ez + indi.Kd_z*(-dz_meas);
            current_az = (dz_meas - prev_dz)/dt;
            delta_az = az_des - current_az;
            delta_T_total = params.m * delta_az / (cos(phi_meas)*cos(theta_meas));
            T_total = prev_T_total + delta_T_total;

            % === INDI Position Control ===       
            ex = x_des - x_meas;
            ey = y_des - y_meas;

            % X-position control
            dex = -dx_meas;
            theta_des = -(indi.Kp_x*ex + indi.Kd_x*dex);

            % Y-position control
            dey = -dy_meas;
            phi_des = (indi.Kp_y*ey + indi.Kd_y*dey);
            
            theta_des = max(-max_tilt, min(max_tilt, theta_des));
            phi_des = max(-max_tilt, min(max_tilt, phi_des));

            % === INDI Attitude Control ===
            % Outer loop: angle control
            p_des = indi.Kp_phi*(phi_des - phi_meas);
            q_des = indi.Kp_theta*(theta_des - theta_meas);
            r_des = indi.Kp_psi*(psi_des - psi_meas);
            
            % Apply low-pass filter
            p_dot = alpha * ((p_meas - prev_p)/dt) + (1-alpha)*p_dot_prev;
            p_dot_prev = p_dot;
            q_dot = alpha * ((q_meas - prev_q)/dt) + (1-alpha)*q_dot_prev;
            q_dot_prev = q_dot;
            r_dot = alpha * ((r_meas - prev_r)/dt) + (1-alpha)*r_dot_prev;
            r_dot_prev = r_dot;

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
    tau_psi_actual = params.c * (T_actual(1) - T_actual(2) + T_actual(3) - T_actual(4)) + tau_psi_wind*0;
    
    % === Dynamics ===
    % Translational acceleration
    ax = (-sin(theta_true)*Fz + Fx_wind)/params.m;
    ay = (sin(phi_true)*Fz + Fy_wind)/params.m;
    az = Fz/params.m - params.g;

    % Rotational accelerations
    dp = tau_phi_actual / params.Ixx;
    dq = tau_theta_actual / params.Iyy;
    dr = tau_psi_actual / params.Izz;
    
    % Integrate true state
    dx_new = dx_true + ax*dt;
    x_new = x_true + dx_new*dt;
    dy_new = dy_true + ay*dt;
    y_new = y_true + dy_new *dt;
    dz_new = dz_true + az*dt; 
    z_new = z_true + dz_new*dt;
    p_new = p_true + dp*dt; 
    phi_new = phi_true + p_new*dt;
    q_new = q_true + dq*dt; 
    theta_new = theta_true + q_new*dt;
    r_new = r_true + dr*dt; 
    psi_new = psi_true + r_new*dt;
    
    % Store true state
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
end

figure('Position', [100, 100, 500, 500], 'Color', 'w');

t = tiledlayout(2,1);
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
exportgraphics(gcf, 'ang.png', 'Resolution', 300)

figure('Position', [600, 100, 500, 500], 'Color', 'w')
t = tiledlayout(3,1);
nexttile;
plot(log.time, log.x_log, 'b', 'LineWidth', 1.5); hold on;
plot(log.time, log.x_des_log, 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('X (m)');
legend('Actual', 'Desired', 'Location', 'best');
nexttile;
plot(log.time, log.y_log, 'b', 'LineWidth', 1.5); hold on;
plot(log.time, log.y_des_log, 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Y (m)');
legend('Actual', 'Desired', 'Location', 'best');
nexttile;
plot(log.time, log.z_log, 'b', 'LineWidth', 1.5); hold on;
plot(log.time, log.z_des_log, 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Z (m)');
legend('Actual', 'Desired', 'Location', 'best');
set(findobj(gcf,'type','axes'),'FontName', 'Arial', 'FontSize', 12);
arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
t.TileSpacing = 'compact';
t.Padding = 'compact';
exportgraphics(gcf, 'pos.png', 'Resolution', 300)

if controller_type == 0
    controller_name = 'PID';
    save('log_PID.mat');
elseif controller_type == 1
    controller_name = 'SMC';
    save('log_SMC.mat');
elseif controller_type == 2
    controller_name = 'INDI';
    save('log_INDI.mat');
end

function out = sat(in, boundary)
    if abs(in) <= boundary
        out = in/boundary;
    else
        out = sign(in);
    end
end
