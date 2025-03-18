function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)

    if read_only_vars.counter < 15
        public_vars.mu = [mean(read_only_vars.gnss_history(:,1)) mean(read_only_vars.gnss_history(:,2)) public_vars.mu(3)];

    else
        public_vars.kf.R = diag([0.0004 0.0003 0.00001]);
    end
     
    mu = public_vars.mu;           % [x, y, theta] (3x1)
    sigma = public_vars.sigma;     % (3x3)

    vR = public_vars.motion_vector(1); 
    vL = public_vars.motion_vector(2);  
    d = read_only_vars.agent_drive.interwheel_dist;  

    v = (vR + vL)/2;  
    w = (vR - vL)/d;  
    dt = read_only_vars.sampling_period;

    [mu, sigma] = ekf_predict(mu, sigma, [v; w], public_vars.kf, dt);

    z = read_only_vars.gnss_position(1:2)'; 
    [mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);

end
