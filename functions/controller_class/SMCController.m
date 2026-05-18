classdef SMCController < BaseController
    properties
        integrators
    end
    
    methods
        function obj = SMCController(params)
            obj.params = params;
            obj.allocationMode = "SMC";
            obj.integrators = struct( ...
                'ez', 0, ...
                'ex', 0, ...
                'ey', 0, ...
                'p',  0, ...
                'q',  0, ...
                'r',  0);
        end
        
        % Altitude Control
        function T_total = altitude(obj, z_des, z_meas, dz_meas, varargin)
            ez  = z_des - z_meas;
            s_z = obj.params.smc.lambda_z*ez + (-dz_meas);
            T_total = obj.params.total_mass*(obj.params.g + obj.params.smc.K_z*sat(s_z, 0.2));
        end
        
        % Position Control
        function [phi_des, theta_des] = position(obj, x_des, y_des, x_meas, y_meas, dx_meas, dy_meas, varargin)

            ex = x_des - x_meas; 
            ey = y_des - y_meas;  
            s_x = obj.params.smc.lambda_xy*ex + (-dx_meas);
            s_y = obj.params.smc.lambda_xy*ey + (-dy_meas);
            
            % Control outputs
            theta_des = -(obj.params.smc.K_xy*sat(s_x, 1));
            phi_des   =  (obj.params.smc.K_xy*sat(s_y, 1));
           
        end
        
        % Attitude Control
        function output = attitude(obj, phi_des, theta_des, psi_des, ...
                                                          phi_meas, theta_meas, psi_meas, ...
                                                          p_meas, q_meas, r_meas, varargin)
            e_phi = phi_des - phi_meas;
            de_phi = -p_meas;
            s_phi = obj.params.smc.lambda_phi*e_phi + de_phi;
            tau_phi = obj.params.Ixx*(obj.params.smc.lambda_phi*de_phi + obj.params.smc.K_phi*sat(s_phi, 0.1)); 

            % Pitch control
            e_theta = theta_des - theta_meas;
            de_theta = -q_meas;
            s_theta = obj.params.smc.lambda_theta*e_theta + de_theta;
            tau_theta = obj.params.Iyy*(obj.params.smc.lambda_theta*de_theta + obj.params.smc.K_theta*sat(s_theta, 0.1));

            % Yaw control
            e_psi = psi_des - psi_meas;
            de_psi = -r_meas;
            s_psi = obj.params.smc.lambda_psi*e_psi + de_psi;
            tau_psi = obj.params.Izz*(obj.params.smc.lambda_psi*de_psi + obj.params.smc.K_psi*sat(s_psi, 0.1));

            output = [tau_phi; tau_theta; tau_psi];
        end
    end
end
