function dx = FcnPlane_longitudinal_dynamics(~, x, u, params, V_air, q_dyn, alpha)
    u_b = x(1); w_b = x(2); q = x(3); theta = x(4); h = x(5);
    delta_e = u(1); T = u(2);
    
    CL = params.CL_0 + params.CL_alpha*alpha + params.CL_q*params.c/(2*V_air)*q + params.CL_de*delta_e;
    CD = params.CD_0 + params.CD_alpha*alpha + params.CD_q*params.c/(2*V_air)*q + params.CD_de*delta_e;
    Cm = params.Cm_0 + params.Cm_alpha*alpha + params.Cm_q*params.c/(2*V_air)*q + params.Cm_de*delta_e;
    
    F_lift = q_dyn*params.S*CL;
    F_drag = q_dyn*params.S*CD;
    M = q_dyn*params.S*params.c*Cm;
   
    Fx_aero = sin(alpha)*(F_lift) - cos(alpha)*(F_drag);
    Fz_aero = -cos(alpha)*(F_lift) - sin(alpha)*(F_drag);

    u_dot = (Fx_aero - params.m*params.g*sin(theta) + T*cos(theta)) / params.m;
    w_dot = (Fz_aero + params.m*params.g*cos(theta) - T*sin(theta)) / params.m;
    q_dot = M / params.Iyy;
    theta_dot = q;
    z_dot = u_b*sin(theta) + w_b*cos(theta);
    dx = [u_dot; w_dot; q_dot; theta_dot; z_dot];    
end