function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

<<<<<<< Updated upstream
mu = public_vars.mu;
sigma = public_vars.sigma;

% I. Prediction
u = [];
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);
=======
    if read_only_vars.counter < 25
        public_vars.mu = [mean(read_only_vars.gnss_history(:,1)) mean(read_only_vars.gnss_history(:,2)) public_vars.mu(3)];

    else
        public_vars.kf.R = diag([0.0007 0.0007 0.00001]);
    end
     
    mu = public_vars.mu;           % [x, y, theta] (3x1)
    sigma = public_vars.sigma;     % (3x3)
>>>>>>> Stashed changes

% II. Measurement
z = [];
[mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);

end

