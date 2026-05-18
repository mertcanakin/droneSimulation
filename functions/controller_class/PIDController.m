classdef PIDController < BaseController
    properties
        integrators
    end
    
    methods
        function obj = PIDController(params)
            obj.params = params;
            obj.allocationMode = "PID";
            obj.integrators = struct( ...
                'ez', 0, ...
                'ex', 0, ...
                'ey', 0, ...
                'p',  0, ...
                'q',  0, ...
                'r',  0);
        end
        
        % Altitude Control
        function T_total = altitude(obj, z_des, z_meas, dz_meas, dt)
            ez  = z_des - z_meas;
            dez = -dz_meas;
            obj.integrators.ez = obj.integrators.ez + ez*dt;
            
            T_total = obj.params.total_mass * ( ...
                obj.params.g + ...
                obj.params.pid.Kp_z * ez + ...
                obj.params.pid.Kd_z * dez + ...
                obj.params.pid.Ki_z * obj.integrators.ez );
        end
        
        % Position Control
        function [phi_des, theta_des] = position(obj, x_des, y_des, x_meas, y_meas, dx_meas, dy_meas, dt)
            ex = x_des - x_meas;
            ey = y_des - y_meas;
            dex = -dx_meas;
            dey = -dy_meas;
            
            obj.integrators.ex = obj.integrators.ex + ex*dt;
            obj.integrators.ey = obj.integrators.ey + ey*dt;
            
            theta_des = -(obj.params.pid.Kp_x*ex + obj.params.pid.Kd_x*dex + obj.params.pid.Ki_x*obj.integrators.ex);
            phi_des   =  (obj.params.pid.Kp_y*ey + obj.params.pid.Kd_y*dey + obj.params.pid.Ki_y*obj.integrators.ey);
           
        end
        
        % Attitude Control
        function output = attitude(obj, phi_des, theta_des, psi_des, ...
                                                          phi_meas, theta_meas, psi_meas, ...
                                                          p_meas, q_meas, r_meas, dt)
            % Outer loop
            p_des = obj.params.pid.Kp_phi * (phi_des   - phi_meas);
            q_des = obj.params.pid.Kp_theta * (theta_des - theta_meas);
            r_des = obj.params.pid.Kp_psi * (psi_des   - psi_meas);
            
            e_p = p_des - p_meas;
            e_q = q_des - q_meas;
            e_r = r_des - r_meas;
            
            % Integrators
            obj.integrators.p = obj.integrators.p + e_p*dt;
            obj.integrators.q = obj.integrators.q + e_q*dt;
            obj.integrators.r = obj.integrators.r + e_r*dt;
            obj.integrators.p = max(-obj.params.pid.int_lim, min(obj.params.pid.int_lim, obj.integrators.p));
            obj.integrators.q = max(-obj.params.pid.int_lim, min(obj.params.pid.int_lim, obj.integrators.q));
            obj.integrators.r = max(-obj.params.pid.int_lim, min(obj.params.pid.int_lim, obj.integrators.r));
            
            % Torques
            tau_phi   = obj.params.Ixx * (obj.params.pid.Kp_p*e_p + obj.params.pid.Ki_p*obj.integrators.p + obj.params.pid.Kd_p*(-p_meas));
            tau_theta = obj.params.Iyy * (obj.params.pid.Kp_q*e_q + obj.params.pid.Ki_q*obj.integrators.q + obj.params.pid.Kd_q*(-q_meas));
            tau_psi   = obj.params.Izz * (obj.params.pid.Kp_r*e_r + obj.params.pid.Ki_r*obj.integrators.r + obj.params.pid.Kd_r*(-r_meas));
            output = [tau_phi; tau_theta; tau_psi];
        
        end
    end
end
