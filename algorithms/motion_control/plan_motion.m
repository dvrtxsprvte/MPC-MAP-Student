function [public_vars] = plan_motion(read_only_vars, public_vars)

[public_vars, target] = get_target(read_only_vars, public_vars);

robot_pose = public_vars.estimated_pose;
xR = robot_pose(1);
yR = robot_pose(2);
thetaR = robot_pose(3);

epsilon = 0.15;
Kp = 0.9;
maxLinSpeed = 1;
maxAngSpeed = 2.0;
d = read_only_vars.agent_drive.interwheel_dist;

lidar = read_only_vars.lidar_distances;


xP = xR + epsilon * cos(thetaR);
yP = yR + epsilon * sin(thetaR);

dx = target(1) - xP;
dy = target(2) - yP;

dot_xP = Kp * dx;
dot_yP = Kp * dy;

speedP = sqrt(dot_xP^2 + dot_yP^2);
if speedP > maxLinSpeed
    scale = maxLinSpeed / speedP;
    dot_xP = dot_xP * scale;
    dot_yP = dot_yP * scale;
end

v = dot_xP * cos(thetaR) + dot_yP * sin(thetaR);
w = (1 / epsilon) * (-dot_xP * sin(thetaR) + dot_yP * cos(thetaR));

if abs(w) > maxAngSpeed
    w = sign(w) * maxAngSpeed;
end

vR = v + w * (d / 2);
vL = v - w * (d / 2);

wall_distance = 0.2;
if ~isfield(public_vars, 'obstacle_counter')
    public_vars.obstacle_counter = 0;
end

if lidar(1) < wall_distance || lidar(2) < wall_distance || lidar(8) < wall_distance
    % infront
    public_vars.motion_vector = [-0.1, -0.2];

    public_vars.obstacle_counter = public_vars.obstacle_counter + 1;

    if public_vars.obstacle_counter >= 10
        if isfield(public_vars, 'pf_enabled') && public_vars.pf_enabled
            public_vars = init_particle_filter(read_only_vars, public_vars);
        end
        public_vars.path = plan_path(read_only_vars, public_vars);
        public_vars.wp_index = 1;
        public_vars.obstacle_counter = 0; 
    end

elseif lidar(3) < wall_distance || lidar(4) < wall_distance
    % left 
    public_vars.motion_vector = [vR, vL * 0.5];
    public_vars.obstacle_counter = 0;

elseif lidar(6) < wall_distance || lidar(7) < wall_distance
    % right 
    public_vars.motion_vector = [vR * 0.5, vL];
    public_vars.obstacle_counter = 0;

elseif lidar(5) < wall_distance
    % back
    public_vars.motion_vector = [0.2, 0.2];
    public_vars.obstacle_counter = 0;

else
    public_vars.motion_vector = [vR, vL];
    public_vars.obstacle_counter = 0;
end


end
