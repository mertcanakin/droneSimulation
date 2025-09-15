function [slung_state_new, F_slung] = slung_load_dynamics(slung_state, L, d, mL, g, ax_est, ay_est, dt)

%% Call current states
alpha  = slung_state(1);
dalpha = slung_state(2);
beta   = slung_state(3);
dbeta  = slung_state(4);

%% Equations of motion
% Rotational acceleration in x axis
ddalpha = - (g / L) * sin(alpha) ...  % gravity effect
          - d * dalpha ...            % damping
          - ax_est / L;               % quad acceleration

% Rotational acceleration in y axis
% Normally coupled but due to singularity it was assumed as decoupled
ddbeta = - (g / L) * sin(beta) ...    
         - d * dbeta ...           
         - ay_est / L;   

%% Update states with Euler integration
dalpha_new = dalpha + ddalpha * dt;
alpha_new  = alpha + dalpha_new * dt;

dbeta_new  = dbeta + ddbeta * dt;
beta_new   = beta + dbeta_new * dt;

%% Saturation for angles
lim = deg2rad(80);
alpha_new = max(-lim, min(lim, alpha_new));
beta_new  = max(-lim, min(lim, beta_new));

%% Cable tension and load forces
cos_a = cos(alpha_new);
cos_b = cos(beta_new);
sin_a = sin(alpha_new);
sin_b = sin(beta_new);

T_cable = mL * (g*cos_a*cos_b + L*(dalpha_new^2 +  dbeta_new^2));
T_cable = max(0, T_cable);

% Load forces
F_slung = [-T_cable * sin_a * cos_b;
           -T_cable * sin_b;
            T_cable * cos_a * cos_b];

%% 6) New state vector
slung_state_new = [alpha_new; dalpha_new; beta_new; dbeta_new];

end
