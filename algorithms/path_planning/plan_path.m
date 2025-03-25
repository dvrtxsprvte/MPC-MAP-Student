function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

%% A*
planning_required = 1;

if planning_required && read_only_vars.counter < 5
   % ( ~isfield(public_vars, 'last_planning_counter') || ...
   %   read_only_vars.counter - public_vars.last_planning_counter >= 200 )
   %   Idea for the project - every xxx iterations recalculate path

    path = astar(read_only_vars, public_vars);
    path = smooth_path(path);

    % public_vars.last_planning_counter = read_only_vars.counter;

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
%path = [2,2;5,5;7,5;7,1;9,1;9,9;7,9;4,6;1,5;1,9;3,9];
%% MAP OUTDOOR_1
% startPth = [2,2];
% goalPth = read_only_vars.map.goal(1:2);
% arc1 = create_arc([8,2],6,180,90,50);
% arc1_end = arc1(end,:);
% line1 = create_line(arc1_end,[12,8],50);
% arc2 = create_arc([12,4],4,90,0,50);
% arc2_end = arc2(end,:);
% line2 = create_line(arc2_end, goalPth,50);
%     path = [
%         startPth; 
%         arc1(2:end,:);
%         line1(2:end,:);
%         arc2(2:end,:);
%         line2(2:end,:);
%     ];
% 
% 
% startPth = [2,2];
% goalPth = read_only_vars.map.goal(1:2);
% 
% arc1 = create_arc([8,2],6,180,90,50);
% arc1_end = arc1(end,:);
% 
% arc2 = create_arc(arc1_end + [0, 1], 1, -90, 0, 50);
% arc2_end = arc2(end, :);
% 
% line1 = create_line(arc2_end, arc2_end + [0, 3], 50);
% 
% arc3 = create_arc([10,12], 1, 180, 0, 50);
% arc3_end = arc3(end, :);
% 
% % Přímka dolů
% line2 = create_line(arc3_end, arc3_end - [0, 3], 50);
% 
% % Malý oblouk dolů
% arc4 = create_arc([12,9], 1, 180, 270, 50);
% 
% % Napojení na druhý oblouk
% arc5 = create_arc([12,4],4,90,0,50);
% arc5_end = arc5(end,:);
% 
% arc6 = create_arc([16.5,4], 0.5, 180, 270, 50);
% arc6_end = arc6(end,:);
% 
% line3 = create_line(arc6_end, arc6_end+[2,0], 50);
% 
% arc7 = create_arc([19,3.25], 0.25, 90, -90, 50); 
% arc7_end = arc7(end,:);
% 
% line4 = create_line(arc7_end, arc7_end-[2,0], 50);
% 
% arc8 = create_arc([17,2], 1, 90, 180, 50); 
% 
% path = [
%     startPth; 
%     arc1(2:end,:);
%     arc2(2:end,:);
%     line1(2:end,:);
%     arc3(2:end,:);
%     line2(2:end,:);
%     arc4(2:end,:);
%     arc5(2:end,:);
%     arc6(2:end,:);
%     line3(2:end,:);
%     arc7(2:end,:);
%     line4(2:end,:);
%     arc8(2:end,:);
% ];
% 
% function points = create_line(p1, p2, nPoints)
% 
%     xVals = linspace(p1(1), p2(1), nPoints);
%     yVals = linspace(p1(2), p2(2), nPoints);
%     points = [xVals(:), yVals(:)];
% end
% 
% function points = create_arc(center, radius, startDeg, endDeg, nPoints)
% 
%     cx = center(1);
%     cy = center(2);
%     angles = linspace(deg2rad(startDeg), deg2rad(endDeg), nPoints);
%     xVals = cx + radius * cos(angles);
%     yVals = cy + radius * sin(angles);
%     points = [xVals(:), yVals(:)];
% end
end


