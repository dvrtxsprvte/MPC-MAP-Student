function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf) %kf_correct

    Ct = kf.C;    % [2x3]
    Q = kf.Q;     % [2x2]

    %3. Kt
    S = Ct * sigma * Ct' + Q;
    Kt = sigma * Ct' * S^-1;

    %4. new_mu
    z_pred = Ct * mu;
    x = z - z_pred;
    new_mu = mu + Kt * x;

    %5. new_sigma 
    I = eye(size(sigma));
    new_sigma = (I - Kt * Ct) * sigma;
end
