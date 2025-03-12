function [pose_estimate] = estimate_pose(public_vars, read_only_vars)

    particles = public_vars.particles;

    med_x = median(particles(:,1));
    med_y = median(particles(:,2));
    med_theta = median(particles(:,3));

    pose_estimate = [med_x, med_y, med_theta];
end
