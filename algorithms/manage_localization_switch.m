function public_vars = manage_localization_switch(read_only_vars, public_vars)

gnss_available_now = ~any(isnan(read_only_vars.gnss_position));

if gnss_available_now ~= public_vars.gnss_available_prev
    public_vars.stabilization_counter = 0;

    if gnss_available_now
        public_vars.kf_enabled = 1;
        public_vars.pf_enabled = 0;
        public_vars.mu = public_vars.estimated_pose';
        public_vars = init_kalman_filter(read_only_vars, public_vars);
        public_vars.localization_mode = 'SWITCHING_TO_EKF';
    else
        public_vars.kf_enabled = 0;
        public_vars.pf_enabled = 1;
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
        public_vars.localization_mode = 'SWITCHING_TO_PF';
    end
end

public_vars.gnss_available_prev = gnss_available_now;

end
