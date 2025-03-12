function [public_vars] = plan_motion(read_only_vars, public_vars)
    %% WEEK 3 - TASK 3
    [public_vars, target] = get_target(read_only_vars, public_vars); % get target

    robot_pose = read_only_vars.mocap_pose;  % [x, y, theta]

    xR = robot_pose(1);
    yR = robot_pose(2);
    thetaR = robot_pose(3);

    epsilon = 0.05;  % virtual point of the robot
    xP = xR + epsilon*cos(thetaR);
    yP = yR + epsilon*sin(thetaR);

    % Dist to target
    dx = target(1) - xP;
    dy = target(2) - yP;

    % P reg
    Kp = 0.5;
    dot_xP = Kp * dx;
    dot_yP = Kp * dy;

    maxLinSpeed = 1; % limit of lin speed
    speedP = sqrt(dot_xP^2 + dot_yP^2);
    if speedP > maxLinSpeed
        scale = maxLinSpeed / speedP;
        dot_xP = dot_xP * scale;
        dot_yP = dot_yP * scale;
    end

    % feedback lineariation
    v =  dot_xP*cos(thetaR) + dot_yP*sin(thetaR);
    w = (1/epsilon)*(-dot_xP*sin(thetaR) + dot_yP*cos(thetaR));

    maxAngSpeed = 2.0; % limit of ang speed
    if abs(w) > maxAngSpeed
        w = sign(w)*maxAngSpeed;
    end

    d = read_only_vars.agent_drive.interwheel_dist;
    vR = v + w*(d/2);
    vL = v - w*(d/2);


    public_vars.motion_vector = [vR, vL];
end
