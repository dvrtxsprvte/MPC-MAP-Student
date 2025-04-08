function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

<<<<<<< Updated upstream
public_vars.kf.C = [];
public_vars.kf.R = [];
public_vars.kf.Q = [];
=======
    
    if ~isfield(public_vars, 'mu')
        public_vars.mu = [0; 0; 0]; 
    end
    
    public_vars.sigma = diag([1,1,1]);
>>>>>>> Stashed changes

public_vars.mu = [];
public_vars.sigma = [];

end

