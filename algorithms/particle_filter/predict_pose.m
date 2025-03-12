function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)

    x = old_pose(1);
    y = old_pose(2);
    theta = old_pose(3);

    vR = motion_vector(1);
    vL = motion_vector(2);
    d = read_only_vars.agent_drive.interwheel_dist;

    v = (vR + vL) / 2;     
    w = (vR - vL) / d;     

    delta_t = read_only_vars.sampling_period; 

    x_new = x + v * cos(theta) * delta_t;
    y_new = y + v * sin(theta) * delta_t;
    theta_new = theta + w * delta_t;

    x_new = x_new + ((randn(1)-0.5)/10);
    y_new = y_new + ((randn(1)-0.5)/10);
    theta_new = theta_new + ((randn(1)-0.5)/10);

    theta_new = mod(theta_new + pi, 2*pi) - pi;

    new_pose = [x_new, y_new, theta_new];
end
