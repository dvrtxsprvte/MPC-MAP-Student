function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
    x_part = pose(1);
    y_part = pose(2);
    th_part = pose(3);

    ray_origin = [x_part, y_part];
    walls = map.walls;

    measurement = zeros(1, length(lidar_config));

    for i = 1:length(lidar_config)
        ray_angle = th_part + lidar_config(i);
        intersections = ray_cast(ray_origin, walls, ray_angle);

        dx = intersections(:,1) - x_part;
        dy = intersections(:,2) - y_part;
        dists = sqrt(dx.^2 + dy.^2);
        distance = min(dists);

        measurement(i) = distance;
    end
end
