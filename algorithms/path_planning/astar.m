function [path] = astar(read_only_vars, public_vars)
<<<<<<< Updated upstream
%ASTAR Summary of this function goes here

path = [];

end

=======
    raw_map = read_only_vars.discrete_map.map;
    [nRows, nCols] = size(raw_map);

    cost_map = expandObstacles(raw_map, 0.5, read_only_vars);
    
    goal = [read_only_vars.discrete_map.goal(2), read_only_vars.discrete_map.goal(1)];
    start = [round(public_vars.estimated_pose(2)/read_only_vars.map.discretization_step), ...
             round(public_vars.estimated_pose(1)/read_only_vars.map.discretization_step)];
    start = max(start, [2, 2]);

    directions = [0 1; 1 0; 0 -1; -1 0];
 

    open_set = [];
    visited = false(nRows, nCols);
    g_score = inf(nRows, nCols);
    f_score = inf(nRows, nCols);
    parent_map = zeros(nRows, nCols, 2);

    g_score(start(1), start(2)) = 0;
    f_score(start(1), start(2)) = estimateHeuristic(start, goal);
    open_set = [start, f_score(start(1), start(2))];

    while ~isempty(open_set)
        [~, i_min] = min(open_set(:,3));
        current = open_set(i_min, 1:2);
        open_set(i_min, :) = [];
        visited(current(1), current(2)) = true;

        if isequal(current, goal)
            path = reconstruct(parent_map, start, goal);
            return;
        end

        for i = 1:size(directions, 1)
            neighbor = current + directions(i, :);

            if any(neighbor < 1) || neighbor(1) > nRows || neighbor(2) > nCols
                continue;
            end

            if cost_map(neighbor(1), neighbor(2)) == 1 || visited(neighbor(1), neighbor(2))
                continue;
            end

            if sum(abs(directions(i, :))) == 2
                step = 1.44;  % diagonální pohyb
            else
                step = 1.0;   % přímý pohyb
            end

            g_candidate = g_score(current(1), current(2)) + step;

            if g_candidate < g_score(neighbor(1), neighbor(2))
                parent_map(neighbor(1), neighbor(2), :) = current;
                g_score(neighbor(1), neighbor(2)) = g_candidate;

                penalty = 100 * cost_map(neighbor(1), neighbor(2));
                f_score(neighbor(1), neighbor(2)) = g_candidate + estimateHeuristic(neighbor, goal) + penalty;

                if ~any(ismember(open_set(:,1:2), neighbor, 'rows'))
                    open_set = [open_set; neighbor, f_score(neighbor(1), neighbor(2))];
                end
            end
        end
    end

    path = [];
    disp("Path not found");
end

function h = estimateHeuristic(p1, p2)
    h = norm(p1 - p2);
end

function path = reconstruct(parents, start, goal)
    path = goal;
    while ~isequal(path(1,:), start)
        prev = parents(path(1,1), path(1,2), :);
        path = [prev(:)'; path];
    end
end

function map_out = expandObstacles(map_in, scale, vars)
    radius = ceil(vars.discrete_map.dims(1) / vars.map.limits(3) * scale);
    [rows, cols] = size(map_in);
    map_out = zeros(rows, cols);
    for r = 1:rows
        for c = 1:cols
            if map_in(r, c) == 1
                map_out(r, c) = 1;
            else
                minDist = inf;
                for dx = -radius:radius
                    for dy = -radius:radius
                        x = r + dx;
                        y = c + dy;
                        if x > 0 && x <= rows && y > 0 && y <= cols && map_in(x, y) == 1
                            minDist = min(minDist, abs(dx) + abs(dy));
                        end
                    end
                end
                map_out(r, c) = min(1, 1 - (minDist / radius));
            end
        end
    end
end

function conv_result = applyConvolution(binary_map, radius)
    kernel_size = 2 * ceil(radius) + 1;
    [X, Y] = meshgrid(-radius:radius, -radius:radius);
    disk_kernel = double(sqrt(X.^2 + Y.^2) <= radius);

    obstacle_mask = double(binary_map == 1);
    conv_result = conv2(obstacle_mask, disk_kernel, 'same');

    if max(conv_result(:)) > 0
        conv_result = conv_result / max(conv_result(:));
    end
end
>>>>>>> Stashed changes
