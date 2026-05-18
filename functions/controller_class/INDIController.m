classdef INDIController < BaseController
    properties
        % Previous values
        prev_dz
        prev_p
        prev_q  
        prev_r
        prev_T_total
        
        % Filtered values
        az_f
        p_dot_f
        q_dot_f  
        r_dot_f
        
        % Integrators
        integral_p
        integral_q
        integral_r
        
        % Current angles
        current_phi
        current_theta
    end
    
    methods
        % Constructor 
        function obj = INDIController(params)
            obj.params = params;
            obj.allocationMode = "INDI";
            
            % Initialize all values to zero
            obj.prev_dz = 0;
            obj.prev_p = 0;
            obj.prev_q = 0;
            obj.prev_r = 0;
            obj.prev_T_total = 0;
            
            obj.az_f = 0;
            obj.p_dot_f = 0;
            obj.q_dot_f = 0;
            obj.r_dot_f = 0;
            
            obj.integral_p = 0;
            obj.integral_q = 0;
            obj.integral_r = 0;
            
            obj.current_phi = 0;
            obj.current_theta = 0;
        end
        
        % Altitude Control
        function T_total = altitude(obj, z_des, z_meas, dz_meas, dt)
            az_raw = (dz_meas - obj.prev_dz)/dt;
            obj.az_f = obj.lpf(az_raw, obj.az_f, 0.3);
            % obj.az_f = az_raw;

            ez = z_des - z_meas;
            az_des = obj.params.indi.Kp_z*ez + obj.params.indi.Kd_z*(-dz_meas);
            delta_az = az_des - obj.az_f;
            
            % Total thrust change
            delta_T_total = (obj.params.total_mass) * delta_az / (cos(obj.current_phi)*cos(obj.current_theta));
            
            % Increase total thrust
            T_total = obj.prev_T_total + delta_T_total;
            
            % Update previous values
            obj.prev_dz = dz_meas;
            obj.prev_T_total = T_total;
        end
        
        % Position Control
        function [phi_des, theta_des] = position(obj, x_des, y_des, x_meas, y_meas, dx_meas, dy_meas, dt)
            ex = x_des - x_meas;
            ey = y_des - y_meas;
            dex = -dx_meas;
            dey = -dy_meas;
            
            theta_des = -(obj.params.indi.Kp_x*ex + obj.params.indi.Kd_x*dex);
            phi_des = (obj.params.indi.Kp_y*ey + obj.params.indi.Kd_y*dey);
        end
        
        % Attitude Control
        function output = attitude(obj, phi_des, theta_des, psi_des, ...
                                              phi_meas, theta_meas, psi_meas, ...
                                              p_meas, q_meas, r_meas, dt)
            obj.current_phi = phi_meas;
            obj.current_theta = theta_meas;
            
            % Outer loop
            p_des = obj.params.indi.Kp_phi*(phi_des - phi_meas);
            q_des = obj.params.indi.Kp_theta*(theta_des - theta_meas);
            r_des = obj.params.indi.Kp_psi*(psi_des - psi_meas);
            
            % Measured angular accelerations
            p_dot_meas = (p_meas - obj.prev_p)/dt;
            q_dot_meas = (q_meas - obj.prev_q)/dt;
            r_dot_meas = (r_meas - obj.prev_r)/dt;
            
            % Integrators
            obj.integral_p = max(-obj.params.indi.int_lim, min(obj.params.indi.int_lim, ...
                         obj.integral_p + (p_des - p_meas)*dt));
            obj.integral_q = max(-obj.params.indi.int_lim, min(obj.params.indi.int_lim, ...
                         obj.integral_q + (q_des - q_meas)*dt));
            obj.integral_r = max(-obj.params.indi.int_lim, min(obj.params.indi.int_lim, ...
                         obj.integral_r + (r_des - r_meas)*dt));
            
            % Desired angular accelerations
            p_dot_des = obj.params.indi.Kp_p*(p_des - p_meas) + obj.params.indi.Ki_p*obj.integral_p - obj.params.indi.Kd_p*obj.p_dot_f;
            q_dot_des = obj.params.indi.Kp_q*(q_des - q_meas) + obj.params.indi.Ki_q*obj.integral_q - obj.params.indi.Kd_q*obj.q_dot_f;
            r_dot_des = obj.params.indi.Kp_r*(r_des - r_meas) + obj.params.indi.Ki_r*obj.integral_r - obj.params.indi.Kd_r*obj.r_dot_f;
            
            % Update filtered values
            obj.p_dot_f = obj.lpf(p_dot_meas, obj.p_dot_f, 0.85);
            obj.q_dot_f = obj.lpf(q_dot_meas, obj.q_dot_f, 0.85);
            obj.r_dot_f = obj.lpf(r_dot_meas, obj.r_dot_f, 0.85);
            
            % INDI increments
            delta_p_dot = p_dot_des - p_dot_meas;
            delta_q_dot = q_dot_des - q_dot_meas;
            delta_r_dot = r_dot_des - r_dot_meas;
            
            % Update previous values
            obj.prev_p = p_meas;
            obj.prev_q = q_meas;
            obj.prev_r = r_meas;
            
            % Return delta rates for thrust allocation
            output = [delta_p_dot; delta_q_dot; delta_r_dot];
        end
        
        % Low-pass filter helper function
        function filtered = lpf(~, raw, prev_filtered, alpha)
            filtered = alpha * prev_filtered + (1 - alpha) * raw;
        end
        
    end
end