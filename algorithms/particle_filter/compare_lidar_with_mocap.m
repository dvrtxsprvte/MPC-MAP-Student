function compare_lidar_with_mocap(map, read_only_vars)
    mocap_pose = read_only_vars.mocap_pose;  % [x, y, theta]
    predicted = compute_lidar_measurement(map, mocap_pose, read_only_vars.lidar_config);

    real_lidar = read_only_vars.lidar_distances;

    diff = predicted - real_lidar;

    disp('=== Comparison LIDAR vs. MoCap-based measurement ===');
    disp(['Predicted distances: ', num2str(predicted)]);
    disp(['Real distances:      ', num2str(real_lidar)]);
    disp(['Diff:                 ', num2str(diff)]);
    disp(['Mean error:         ', num2str(mean(abs(diff)))]);
end
