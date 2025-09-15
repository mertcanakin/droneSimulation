function [phi_comp, theta_comp] = slung_compensator(alpha, dalpha, beta, dbeta, l, Kp, Kd, g)

    ax_comp = - l * ( Kp * alpha + Kd * dalpha );
    ay_comp = - l * ( Kp * beta + Kd * dbeta );
    
    theta_comp =  atan(ax_comp / g);   % pitch correction
    phi_comp   = -atan(ay_comp / g);   % roll correction
    
end