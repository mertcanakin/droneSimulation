function [x_des, y_des, z_des, psi_des] = helix_trajectory(t, R)

    psi0 = 0;
    
    t_transition = 5.0;
    
    if t < t_transition
        alpha = (1 - cos(pi * t / t_transition)) / 2;
        x_des = alpha * R * cos(t);
        y_des = alpha * R * sin(t);
        z_des = alpha * t/4;
        psi_des = psi0;
    else
        x_des = R * cos(t);
        y_des = R * sin(t);
        z_des = t/4;
        psi_des = psi0;
    end
end