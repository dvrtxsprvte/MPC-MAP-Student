function [public_vars] = init_particle_filter(read_only_vars, public_vars)

    num_particles = 1000;

    walls = read_only_vars.map.walls;
    xmin = min(walls(:, [1, 3]), [], 'all');
    xmax = max(walls(:, [1, 3]), [], 'all');
    ymin = min(walls(:, [2, 4]), [], 'all');
    ymax = max(walls(:, [2, 4]), [], 'all');
    theta_min = 0;
    theta_max =  2*pi;

    particles = zeros(num_particles, 3);

    particles(:,1) = xmin + (xmax - xmin) * rand(num_particles,1);
    particles(:,2) = ymin + (ymax - ymin) * rand(num_particles,1);
    particles(:,3) = theta_min + (theta_max - theta_min) * rand(num_particles,1);

    public_vars.particles = particles;
    
end
