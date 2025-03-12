function [weights] = weight_particles(particle_measurements, lidar_distances)
    N = size(particle_measurements, 1);
    weights = zeros(N,1);

    distances = sqrt(sum((particle_measurements - lidar_distances).^2, 2));
    weights = 1 ./ distances;

    weights(isnan(weights)) = 0.000001;

    weights = weights / sum(weights);
end
