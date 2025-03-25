function [public_vars, target] = get_target(read_only_vars, public_vars)
    %% WEEK 3 - TASK 3
    % robot_pose = read_only_vars.mocap_pose;  % [x, y, theta]
    
    % robot_pose = public_vars.mu;  % [x, y, theta]
    robot_pose = public_vars.estimated_pose;
    
    Rx = robot_pose(1);
    Ry = robot_pose(2);
    
    if ~isfield(public_vars, 'wp_index')
        public_vars.wp_index = 1; 
    end

    nWaypoints = size(public_vars.path, 1);

    if public_vars.wp_index > nWaypoints
        public_vars.wp_index = nWaypoints;
    end

    current_wp = public_vars.path(public_vars.wp_index, :);
    distToCurrent = norm([Rx - current_wp(1), Ry - current_wp(2)]);

    threshold = 0.8;
    if distToCurrent < threshold
        if public_vars.wp_index < nWaypoints
            public_vars.wp_index = public_vars.wp_index + 1;
        else
        % no more waypoints 
        end
    end

    target = public_vars.path(public_vars.wp_index, :);
end
