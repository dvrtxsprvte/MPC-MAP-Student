function [public_vars] = init_kalman_filter(read_only_vars, public_vars)

    public_vars.mu = [0; 0; 0]; 
    public_vars.sigma = diag([1,1,1]);

    public_vars.kf.C = [1 0 0; 
                        0 1 0];
    
    sx = 0.571617494;  
    sy = 0.508137074;  
    public_vars.kf.Q = diag([sx^2, sy^2]);
    

    % Covariance R
    public_vars.kf.R = diag([0.001, 0.001, 0.1]);

end
