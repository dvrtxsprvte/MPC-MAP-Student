function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector
%
public_vars.motion_vector = [0, 0];


%% WEEK 2 - TASK 5
%         if ~isfield(public_vars, 'i') 
%             public_vars.i = 0;
%         end
% 
%         if ~isfield(public_vars, 'state')  
%             public_vars.state = 1; 
%         end
% 
% switch public_vars.state
%     case 1  
%         public_vars.motion_vector = [0.5, 0.5];
%         public_vars.i = public_vars.i + 1;
%         if public_vars.i >= 135
%             public_vars.motion_vector = [0, 0];  
%             public_vars.state = 2;  
%             public_vars.i = 0;  
%         end
% 
%     case 2 
%         public_vars.motion_vector = [-0.1, 0.1];
%         public_vars.i = public_vars.i + 1;
%         if public_vars.i >= 18  
%             public_vars.motion_vector = [0, 0];  
%             public_vars.state = 3;
%             public_vars.i = 0;  
%         end
%     case 3
%         public_vars.motion_vector = [0.5, 0.5];
%         public_vars.i = public_vars.i + 1;
%         if public_vars.i >= 65
%             public_vars.motion_vector = [0, 0];  
%             public_vars.state = 4;  
%             public_vars.i = 0;  
%         end
%     case 4
%         public_vars.motion_vector = [-0.1, 0.1];
%         public_vars.i = public_vars.i + 1;
%         if public_vars.i >= 16  
%             public_vars.motion_vector = [0, 0];  
%             public_vars.state = 5; 
%             public_vars.i = 0;  
%         end 
%     case 5
%         public_vars.motion_vector = [0.5, 0.5];
%         public_vars.i = public_vars.i + 1;
%         if public_vars.i >= 120
%             public_vars.motion_vector = [0, 0];  
%             public_vars.state = 6;  
%             public_vars.i = 0;  
%         end
%     case 6
%         public_vars.motion_vector = [0.1, -0.1];
%         public_vars.i = public_vars.i + 1;
%         if public_vars.i >= 16  
%             public_vars.motion_vector = [0, 0];  
%             public_vars.state = 7; 
%             public_vars.i = 0;  
%         end 
%     case 7
%         public_vars.motion_vector = [0.5, 0.5];
%         public_vars.i = public_vars.i + 1;
%         if public_vars.i >= 70
%             public_vars.motion_vector = [0, 0];  
%             public_vars.state = 8;  
%             public_vars.i = 0;  
%         end
%     case 8
%         public_vars.motion_vector = [0.1, -0.1];
%         public_vars.i = public_vars.i + 1;
%         if public_vars.i >= 15  
%             public_vars.motion_vector = [0, 0];  
%             public_vars.state = 9; 
%             public_vars.i = 0;  
%         end
%     case 9
%         public_vars.motion_vector = [0.7, 0.7];
% end
end