classdef (Abstract) BaseController < handle
    
    properties
        params
        allocationMode   % "PID", "SMC", "INDI"
    end
    
    %% Constructor
    methods
        % function obj = ControllerBase(params, allocationMode)
        %     obj.params = params;
        %     obj.allocationMode = allocationMode;
        % end

        function T_command = thrust_allocation(obj, T_total, output, varargin)

            switch obj.allocationMode
                case {"PID","SMC"}
                    tau = output;
                    A = [1  1   1  1;
                         0 -obj.params.l 0 obj.params.l;
                         obj.params.l 0 -obj.params.l 0;
                         obj.params.c -obj.params.c obj.params.c -obj.params.c];
                    
                    b = [T_total; tau(:)];
                    T_command = A \ b;
                    
                case "INDI"
                    delta_rates = output;
                    prev_T_actual = varargin{1};
                    G = [0, -obj.params.l/obj.params.Ixx, 0, obj.params.l/obj.params.Ixx; ...
                         obj.params.l/obj.params.Iyy, 0, -obj.params.l/obj.params.Iyy, 0; ...
                         obj.params.c/obj.params.Izz, -obj.params.c/obj.params.Izz, obj.params.c/obj.params.Izz, -obj.params.c/obj.params.Izz];
                            
                    delta_T_att = pinv(G) * delta_rates(:);
                    T_command = prev_T_actual + delta_T_att;
                    
                    % total thrust correction
                    sum_err = T_total - sum(T_command);
                    T_command = T_command + (sum_err/4)*ones(4,1);

                otherwise
                    error("Unknown thrust allocation mode: %s", obj.allocationMode)
            end
            
            % thrust limits
            T_command = max(obj.params.T_min, min(obj.params.T_max, T_command));
        end
    end
    
    methods (Abstract)
        % Altitude control
        T_total = altitude(obj, z_des, z_meas, dz_meas, varargin);
        
        % Position control
        [phi_des, theta_des] = position(obj, x_des, y_des, x_meas, y_meas, dx_meas, dy_meas, dt);
        
        % Attitude control
        output = attitude(obj, phi_des, theta_des, psi_des, ...
                             phi_meas, theta_meas, psi_meas, ...
                             p_meas, q_meas, r_meas, dt);
    end
end