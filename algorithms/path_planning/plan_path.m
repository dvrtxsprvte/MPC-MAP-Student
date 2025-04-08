function [path] = plan_path(read_only_vars, public_vars)
<<<<<<< Updated upstream
%PLAN_PATH Summary of this function goes here

planning_required = 1;

if planning_required
    
    path = astar(read_only_vars, public_vars);
    
    path = smooth_path(path);
    
else
    
    path = public_vars.path;
    
end

end

=======
  
    path = astar(read_only_vars, public_vars);
    if isempty(path)
        return
    end
    
    path = [path(:,2) .* read_only_vars.map.discretization_step-0.2 path(:,1) .* read_only_vars.map.discretization_step-0.3];

    path = smooth_path(path);
end
>>>>>>> Stashed changes
