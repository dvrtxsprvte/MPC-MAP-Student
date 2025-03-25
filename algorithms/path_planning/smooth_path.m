function [new_path] = smooth_path(old_path)

    alpha     = 0.3;    
    beta      = 0.7;    
    tolerance = 1e-4;   
    maxIter   = 123;    

    new_path = old_path;

    new_path(1,:)   = old_path(1,:);
    new_path(end,:) = old_path(end,:);

    N = size(old_path, 1);

    for iter = 1:maxIter
        deltaSum = 0;

        for i = 2:(N-1)
            y_i   = new_path(i,:);
            x_i   = old_path(i,:);
            y_im1 = new_path(i-1,:);
            y_ip1 = new_path(i+1,:);

            y_i_new = y_i ...
                      + alpha*(x_i - y_i) ...
                      + beta*(y_im1 + y_ip1 - 2*y_i)*0.5; 
            
            moveVec = y_i_new - y_i;
            delta   = moveVec(1)^2 + moveVec(2)^2;
            deltaSum = deltaSum + delta;

            new_path(i,:) = y_i_new;
        end

        if deltaSum < tolerance
            break;
        end
    end

end
