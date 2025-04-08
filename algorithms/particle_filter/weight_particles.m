function [weights] = weight_particles(particle_measurements, lidar_distances)
<<<<<<< Updated upstream
%WEIGHT_PARTICLES Summary of this function goes here

N = size(particle_measurements, 1);
weights = ones(N,1) / N;

=======
    valid_mask = ~isinf(lidar_distances);

    same_mask = all(isnan(particle_measurements) == ~valid_mask, 2);
    

    weights = ones(size(particle_measurements,1),1) * 1e-9;


    valid_particles = particle_measurements(same_mask, valid_mask);
    lidar_vector = lidar_distances(valid_mask);


    diffs = valid_particles - lidar_vector;
    dists = sqrt(sum(diffs.^2, 2));

    weights(same_mask) = 1 ./ (dists + 1e-6);

    weights = weights / sum(weights);
>>>>>>> Stashed changes
end

