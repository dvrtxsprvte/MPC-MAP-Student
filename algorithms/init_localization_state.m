function public_vars = init_localization_state(read_only_vars, public_vars)

gnss_available = ~any(isnan(read_only_vars.gnss_position));

public_vars.localization_mode = "";
public_vars.gnss_available_prev = gnss_available;
public_vars.stabilization_counter = 0;
public_vars.path_idx = 0;
public_vars.lost = false;


if gnss_available
    public_vars.kf_enabled = 1;
    public_vars.pf_enabled = 0;
    theta_rnd = rand() * 2 * pi - pi;
    public_vars.estimated_pose = [median(read_only_vars.gnss_history(:,1)), median(read_only_vars.gnss_history(:,2)), theta_rnd];
    public_vars.kf.R = diag([0.001, 0.001, 1]);
    public_vars = init_kalman_filter(read_only_vars, public_vars);
    public_vars.localization_mode = 'EKF';
else
    public_vars.kf_enabled = 0;
    public_vars.pf_enabled = 1;
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    public_vars.estimated_pose = estimate_pose(public_vars);
    public_vars.localization_mode = 'PF';
end

end
