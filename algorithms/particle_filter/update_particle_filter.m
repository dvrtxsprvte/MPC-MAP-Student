function [particles] = update_particle_filter(read_only_vars, public_vars)

particles = public_vars.particles;

% --- I. Prediction ---
for i=1:size(particles, 1)
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
end

% --- II. Correction ---
measurements = zeros(size(particles,1), length(read_only_vars.lidar_config));
for i=1:size(particles, 1)
    measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
end
weights = weight_particles(measurements, read_only_vars.lidar_distances);

% --- III. Resampling ---
particles = resample_particles(particles, weights);

% --- IV. Kidnapped robot injection --- 
fraction = 0.05;
n_inject = round(fraction * size(particles,1));
idx_inject = randperm(size(particles,1), n_inject);

coords = reshape(read_only_vars.map.gnss_denied, 2, [])';  
xPoly = coords(:,1);
yPoly = coords(:,2);

xmin = min(xPoly);
xmax = max(xPoly);
ymin = min(yPoly);
ymax = max(yPoly);

k = 1;
while k <= n_inject
    x_rand = xmin + (xmax - xmin)*rand;
    y_rand = ymin + (ymax - ymin)*rand;
    if inpolygon(x_rand, y_rand, xPoly, yPoly)
        th_rand = -pi + (2*pi)*rand;
        particles(idx_inject(k), :) = [x_rand, y_rand, th_rand];
        k = k + 1;
    end
end


end
