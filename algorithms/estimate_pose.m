function [pose_estimate] = estimate_pose(public_vars, read_only_vars)
        
    if public_vars.pf_enabled
        particles = public_vars.particles;
        xMed     = median(particles(:,1));
        yMed     = median(particles(:,2));
        thetaMed = median(particles(:,3));
        pose_estimate = [xMed, yMed, thetaMed];
    end

    if public_vars.kf_enabled
        pose_estimate = public_vars.mu';  % (x,y,theta)
    end
    
end
