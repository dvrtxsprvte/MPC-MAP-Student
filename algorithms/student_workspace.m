function [public_vars] = student_workspace(read_only_vars, public_vars)

%% INIT
if read_only_vars.counter <= 1
    public_vars = init_localization_state(read_only_vars, public_vars);
end

if public_vars.pf_enabled && read_only_vars.counter < 25
    public_vars.motion_vector = [-1, 1];  
    return;  
end

if public_vars.kf_enabled && read_only_vars.counter < 25
    public_vars.motion_vector = [0, 0];  
    return;  
end

%% Localization
public_vars = manage_localization_switch(read_only_vars, public_vars);
switch public_vars.localization_mode
    case 'EKF'
        [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
        public_vars.estimated_pose = public_vars.mu';
    case 'PF'
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
        public_vars.estimated_pose = estimate_pose(public_vars);
    case {'SWITCHING_TO_EKF', 'SWITCHING_TO_PF'}
        public_vars.motion_vector = [0, 0];
        public_vars.stabilization_counter = public_vars.stabilization_counter + 1;
        if public_vars.stabilization_counter >= 10
            if strcmp(public_vars.localization_mode, 'SWITCHING_TO_EKF')
                public_vars.localization_mode = 'EKF';
            else
                public_vars.localization_mode = 'PF';
            end
        end
end

%% Estimate pose
if strcmp(public_vars.localization_mode, 'EKF') || strcmp(public_vars.localization_mode, 'PF')
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
end

if public_vars.pf_enabled % particle respawn if estimated_pose is close to the goal pos
    est = public_vars.estimated_pose(1:2); 
    goal = read_only_vars.map.goal;  
    if norm(est - goal) < 0.3
        public_vars = init_particle_filter(read_only_vars, public_vars);
    end
end


%% Path Planning
if isempty(public_vars.path)
    public_vars.path = plan_path(read_only_vars, public_vars);
    public_vars.wp_index = 1;
end

if ~isempty(public_vars.path) && isfield(public_vars, 'wp_index') && public_vars.wp_index <= size(public_vars.path, 1)
    current_wp = public_vars.path(public_vars.wp_index, :);
    robot_pos = public_vars.estimated_pose(1:2);
    if norm(robot_pos - current_wp) > 1.75
        public_vars.path = plan_path(read_only_vars, public_vars);
        public_vars.wp_index = 1;
    end
end

%% Plan Motion
public_vars = plan_motion(read_only_vars, public_vars);

end
