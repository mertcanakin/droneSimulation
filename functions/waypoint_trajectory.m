function [x_des, y_des, z_des, psi_des] = waypoint_trajectory(current_time, waypoints, waypoint_times)
    
    waypoint_idx = size(waypoints, 1); 
    for j = 1:length(waypoint_times)-1
        if current_time < waypoint_times(j+1)
            waypoint_idx = j + 1;
            break;
        end
    end
    
    waypoint_idx = min(waypoint_idx, size(waypoints, 1));
    
    x_des   = waypoints(waypoint_idx, 1);
    y_des   = waypoints(waypoint_idx, 2);
    z_des   = waypoints(waypoint_idx, 3);
    psi_des = waypoints(waypoint_idx, 4);

end