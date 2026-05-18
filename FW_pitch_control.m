clc; clear; close all
addpath(genpath('functions'));

% Aircraft Parameters
params.m = 1.56;
params.Iyy = 0.0576;
params.S = 0.259;
params.c = 0.33;
params.g = 9.81;

% Atmosphere parameters
params.rho0 = 1.225;       % sea level density (kg/m3)
params.T0   = 288.15;      % sea level temperature (K)
params.L    = 0.0065;      % temperature lapse rate (K/m)
params.R    = 287.05;      % specific gas constant for air (J/kg/K)
params.h_limit = 11000;    % troposphere limit (m)

% Aerodynamic coefficients
params.CL_0      = 0.28;
params.CL_alpha  = 3.45;
params.CL_q      = 0.0;
params.CL_de     = -0.36;
params.CD_0      = 0.03;
params.CD_alpha  = 0.30;
params.CD_q      = 0.0;
params.CD_de     = 0.0;
params.Cm_0      = -0.02338;
params.Cm_alpha  = -0.38;
params.Cm_q      = -3.6;
params.Cm_de     = -0.5; 

% Servo parameters
params.servo_tau = 0.01;                % servo time constant (s)
params.servo_rate_limit = deg2rad(200); % maximum servo rate (rad/s)

% Simulation setup
x0 = [10; 0; 0; 0; 1000];    % [u; w; q; theta; h]
delta_e0 = 0;
delta_e_actual0 = 0;  % actual servo position
Tnom = 20;            % nominal thrust
u_input = [delta_e0; Tnom];

% Time
dt = 0.01;
t_end = 40;
t = 0:dt:t_end;
N = length(t);

x                 = zeros(N,5);
dx_prev           = zeros(5,1);
x(1,:)            = x0';
delta_e_cmd       = zeros(N,1);
delta_e_actual    = zeros(N,1);
delta_e_cmd(1)    = delta_e0;
delta_e_actual(1) = delta_e_actual0;
Tcmd              = zeros(N,1);
Tcmd(1)           = Tnom;
q_dot_meas = 0;

% INDI pitch control gains
Kp_theta = 3;
Kp_q = 15.0;

% Elevator limits
delta_max = deg2rad(25);
delta_min = -deg2rad(25);

% Airspeed controller gains
Kp_u = 10.0;
Ki_u = 0.0;
T_int = 0;
T_max = 100;
T_min = 0; 

% Desired airspeed profile
V_ref = 10;

% Logging
log.alpha = zeros(N,1);
log.Va = zeros(N,1);
log.rho = zeros(N,1);
log.q_ref = zeros(N,1);

% Theta commands
t_cmd = [0 5 10 20 30 35 40];
theta_cmd = deg2rad([0 -10 0 20 0 -20 0]);
theta_ref = interp1(t_cmd, theta_cmd, t, 'previous'); 

% Simulation loop
for i = 1:N-1
    xi = x(i,:)';
    u_b = xi(1); w_b = xi(2); q = xi(3); theta = xi(4); h = xi(5);

    % Calculate air density
    rho = FcnPlane_compute_density(h, params);
    
    % Generate wind
    V_wind = FcnPlane_wind_model(t(i),h);
    
    % Relative speed
    u_r = u_b - V_wind(1);
    w_r = w_b - V_wind(2);

    % Airspeed and alpha calculation
    V_air = sqrt(u_r^2 + w_r^2);
    alpha = atan2(w_r, u_r);
    q_dyn = 0.5*rho*V_air^2;

    % Speed controller 
    V_err = V_ref - V_air;
    T_int = T_int + Ki_u * V_err * dt;
    T_unsat = Tnom + Kp_u * V_err + T_int;
    T_new = min(max(T_unsat, T_min), T_max);
    Tcmd(i+1) = T_new;
    
    % INDI Pitch Controller
    q_dot_meas = dx_prev(3);
    
    theta_err = theta_ref(i) - theta;
    q_ref = Kp_theta * theta_err;
    q_dot_des = Kp_q * (q_ref - q); 
    
    % Control effectiveness
    G_est = (q_dyn * params.S * params.c * params.Cm_de) / params.Iyy;
    
    % INDI control law
    delta_increment = (q_dot_des - q_dot_meas)/G_est;
    delta_cmd_new = delta_e_cmd(i) + delta_increment;
    delta_e_cmd(i+1) = min(max(delta_cmd_new, delta_min), delta_max);
    
    % Servo Dynamics
    delta_dot = (delta_e_cmd(i+1) - delta_e_actual(i)) / params.servo_tau;
    delta_dot = min(max(delta_dot, -params.servo_rate_limit), params.servo_rate_limit);
    
    % Update servo position
    delta_e_actual(i+1) = delta_e_actual(i) + delta_dot * dt;
    delta_e_actual(i+1) = min(max(delta_e_actual(i+1), delta_min), delta_max);
    
    %  Integrate states
    dx = FcnPlane_longitudinal_dynamics(t(i), xi, [delta_e_actual(i+1); T_new], params, V_air, q_dyn, alpha);
    x(i+1,:) = xi' + dx'*dt;
    dx_prev = dx;

    log.alpha(i) = alpha;
    log.Va(i) = V_air;
    log.rho(i) = rho;
    log.q_ref(i) = q_ref;
end
    
figure('Position', [200, 100, 900, 650], 'Color', 'w')
tl = tiledlayout(3,2);
nexttile;
plot(t, rad2deg(x(:,4)), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(theta_ref), '--r', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Theta (deg)');
legend('Actual', 'Desired','Location','northwest');

nexttile;
plot(t, rad2deg(x(:,3)), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(log.q_ref), '--r', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('q (deg/s)');
legend('Actual', 'Desired','Location','northwest');

nexttile;
plot(t, x(:,5), 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude (m)');

nexttile;
plot(t, rad2deg(delta_e_cmd), 'b', 'LineWidth',1.5); hold on;
plot(t, rad2deg(delta_e_actual), '--r', 'LineWidth',1.2);
ylabel('\delta_e (deg)'); grid on;
legend('Actual', 'Desired','Location','northwest');

nexttile;
plot(t, rad2deg(log.alpha), 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\alpha (deg)');

nexttile;
plot(t, log.Va, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('TAS (m/s)');

set(findobj(gcf,'type','axes'),'FontName', 'Arial', 'FontSize', 12);
arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
tl.TileSpacing = 'compact';
tl.Padding = 'compact';
exportgraphics(gcf,'plane_result.png', 'Resolution', 300)