function [path] = astar(read_only_vars, public_vars)

    map = read_only_vars.discrete_map.map;   
    dims = read_only_vars.discrete_map.dims;  
    limits = read_only_vars.discrete_map.limits;
    goal = read_only_vars.discrete_map.goal;  

    nCols = dims(1);
    nRows = dims(2);
    xmin = limits(1);  
    ymin = limits(2);
    xmax = limits(3);  
    ymax = limits(4);

    resX = (xmax - xmin) / (nCols - 1);

    %% TASK 2
    clearance = 0.3;  % distance from walls, etc.
    range = ceil(clearance / resX);  
    map_expanded = map; 
    for r = 1:nRows
        for c = 1:nCols
            if map(r, c) == 1
                for dr = -range:range
                    for dc = -range:range
                        r_new = r + dr;
                        c_new = c + dc;
                        if (r_new >= 1) && (r_new <= nRows) && (c_new >= 1) && (c_new <= nCols)
                            map_expanded(r_new, c_new) = 1;
                        end
                    end
                end
            end
        end
    end
    map = map_expanded;

    %%
    xStart = public_vars.estimated_pose(1);
    yStart = public_vars.estimated_pose(2);

    rowStart = 1 + round( (yStart - ymin) / ((ymax - ymin)/(nRows - 1)) );
    colStart = 1 + round( (xStart - xmin) / ((xmax - xmin)/(nCols - 1)) );

    colGoal = goal(1);
    rowGoal = goal(2);

    %% call A*
    PathAstart = a_star_indexed(map, [rowStart, colStart], [rowGoal, colGoal]);

    %% Path Reconstruction
    if isempty(PathAstart)
        disp('Path not found.');
        path = [];
    else
        path = zeros(size(PathAstart,1), 2);
        resY = (ymax - ymin) / (nRows - 1);
        for i = 1:size(PathAstart,1)
            rr = PathAstart(i,1);
            cc = PathAstart(i,2);
            xVal = xmin + (cc - 1)*resX;
            yVal = ymin + (rr - 1)*resY;
            path(i,:) = [xVal, yVal];
        end
    end
end

%% A* algo
function route = a_star_indexed(occGrid, start, goal)
    [nRows, nCols] = size(occGrid);

    neighbors = [
        -1,  0;
         1,  0;
         0, -1;
         0,  1;
        -1, -1;
        -1,  1;
         1, -1;
         1,  1
    ];

    gCost = inf(nRows, nCols);
    fCost = inf(nRows, nCols);
    cameFrom = zeros(nRows, nCols, 2, 'int32');

    rs = start(1); cs = start(2);
    rg = goal(1);  cg = goal(2);

    gCost(rs,cs) = 0;
    fCost(rs,cs) = euklidDist(rs, cs, rg, cg);

    openSet = [rs, cs, fCost(rs,cs)];
    closedSet = false(nRows, nCols);

    while ~isempty(openSet)
        [~, idxMin] = min(openSet(:,3));
        currentRC   = openSet(idxMin, 1:2);
        openSet(idxMin,:) = [];

        rCur = currentRC(1);
        cCur = currentRC(2);

        if rCur == rg && cCur == cg
            route = reconstructPath(cameFrom, [rs, cs], [rg, cg]);
            return;
        end

        closedSet(rCur, cCur) = true;

        for iN = 1:size(neighbors,1)
            rN = rCur + neighbors(iN,1);
            cN = cCur + neighbors(iN,2);

            if rN<1 || rN>nRows || cN<1 || cN>nCols
                continue;
            end
            if occGrid(rN,cN) == 1
                continue;
            end
            if closedSet(rN,cN)
                continue;
            end

            costStep = euklidDist(rCur, cCur, rN, cN);
            tentative_g = gCost(rCur, cCur) + costStep;

            if tentative_g < gCost(rN, cN)
                gCost(rN, cN) = tentative_g;
                hVal = euklidDist(rN, cN, rg, cg);
                fCost(rN, cN) = tentative_g + hVal;
                cameFrom(rN, cN, :) = [rCur, cCur];

                idxExist = find(openSet(:,1)==rN & openSet(:,2)==cN);
                if isempty(idxExist)
                    openSet = [openSet; rN, cN, fCost(rN,cN)];
                else
                    openSet(idxExist, 3) = fCost(rN, cN);
                end
            end
        end
    end

    route = [];
end

function d = euklidDist(a, b, c, d)
    dr = c - a;
    dc = d - b;
    d  = sqrt(double(dr*dr + dc*dc));
end

function route = reconstructPath(cameFrom, startRC, goalRC)
    route = goalRC;
    current = goalRC;
    while any(current ~= startRC)
        pr = cameFrom(current(1), current(2), 1);
        pc = cameFrom(current(1), current(2), 2);
        current = [pr, pc];
        route = [current; route];
    end
end
