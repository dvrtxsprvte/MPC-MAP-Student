function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

%% A*
planning_required = 0;

if planning_required
    
    path = astar(read_only_vars, public_vars);
    
    path = smooth_path(path);
    
else
    
    path = public_vars.path;
    
end

% startPth = [2, 8.5]; 
% 
% goalPth = read_only_vars.map.goal(1:2);  % ocekavane [2, 9]
% 
% path = create_line(startPth, goalPth, 50);
% 
% %% WEEK 3 - TASK 2   
%     startPth = [2, 8.5];
%     goalPth  = read_only_vars.map.goal(1:2);  % [x, y] = (9, 9)
% 
%     line1 = create_line([2,8.5], [3,8.5], 20);
% 
%     arc1 = create_arc([3, 5.5], 3, 90, 0, 30);
% 
%     line2_start = arc1(end, :);
%     line2 = create_line(line2_start, [6,3], 40);
% 
%     arc2 = create_arc([7.5,3], 1.5, -180, 0, 30);
% 
%     line3_start = arc2(end, :);
%     line3 = create_line(line3_start, goalPth, 40);
% 
%     path = [
%         startPth; 
%         line1(2:end,:);
%         arc1(2:end,:);
%         line2(2:end,:);
%         arc2(2:end,:);
%         line3(2:end,:);
%     ];

%% MAP INDOOR_3
path = [2,2;5,5;7,5;7,1;9,1;9,9;7,9;4,6;1,5;1,9;3,9];
function points = create_line(p1, p2, nPoints)

    xVals = linspace(p1(1), p2(1), nPoints);
    yVals = linspace(p1(2), p2(2), nPoints);
    points = [xVals(:), yVals(:)];
end

function points = create_arc(center, radius, startDeg, endDeg, nPoints)

    cx = center(1);
    cy = center(2);
    angles = linspace(deg2rad(startDeg), deg2rad(endDeg), nPoints);
    xVals = cx + radius * cos(angles);
    yVals = cy + radius * sin(angles);
    points = [xVals(:), yVals(:)];
end
end


