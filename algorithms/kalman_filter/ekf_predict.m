function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)

    v = u(1);
    w = u(2);

    x = mu(1);
    y = mu(2);
    theta = mu(3);
    dt = sampling_period;
    
    % Apriori 
    x_pred = x + v*cos(theta)*dt;
    y_pred = y + v*sin(theta)*dt;
    theta_pred = theta + w*dt;
    
    new_mu = [x_pred; y_pred; theta_pred];

    % Jacobian G
    G = [ 1, 0, -v*sin(theta)*dt;
          0, 1,  v*cos(theta)*dt;
          0, 0,  1             ];

    new_sigma = G * sigma * G' + kf.R;
end
