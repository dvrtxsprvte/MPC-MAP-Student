function [public_vars] = init_particle_filter(read_only_vars, public_vars)

    num_particles = 1000;

    coords = reshape(read_only_vars.map.gnss_denied, 2, [])';  
    xPoly = coords(:,1);
    yPoly = coords(:,2);

    xmin = min(xPoly);
    xmax = max(xPoly);
    ymin = min(yPoly);
    ymax = max(yPoly);

    theta_min = 0;
    theta_max =  2*pi;

    particles = zeros(num_particles, 3);

    particles(:,1) = xmin + (xmax - xmin) * rand(num_particles,1);
    particles(:,2) = ymin + (ymax - ymin) * rand(num_particles,1);
    particles(:,3) = theta_min + (theta_max - theta_min) * rand(num_particles,1);

    public_vars.particles = particles;
    
end
