function [path] = aStar(map, start, goal)

    % DEFINITIONS    
    
    openList = [start];                                 % List with possible nodes to be visited
    closedSet = false(size(map));                       % List with nodes already visited
    cameFrom = zeros(size(map, 1), size(map, 2), 2);    % 3D Array with nodes visiteds and where they came from;
    path = [];
        
    fScore = inf(size(map));     % Evaluation function        
    gScore = inf(size(map));     % Cost of the path from starting node to current node
    
    % Initialize functions with the costs of starting node
    fScore(start(1), start(2)) = getManhattanDistance(start, goal); 
    gScore(start(1), start(2)) = 0;                                 
    
    % MAIN LOOP
    while ~isempty(openList)
        
        % Get node with smaller fScore to be evaluated
        [current, openList] = getMinimumFScore(openList, fScore);
        
        if closedSet(current(1), current(2))
            continue;
        else
            closedSet(current(1), current(2)) = true;
        end

        % If the goal was reached, ends the program and reconstructs the path
        if isequal(current, goal)
            path = reconstructPath(start, current, cameFrom);
            break;
        end
        
        % Get neighbors of the current node
        neighbors = getNeighbors(current, map);
        n = size(neighbors, 1);
    
        for i=1:n
            neighbor = neighbors(i, :);

            if closedSet(neighbor(1), neighbor(2))
                continue;
            end
    
            % Calculates cost from starting node to current one
            gAttempt = gScore(current(1), current(2)) + getNeighborDistance(neighbor, current);
    
            % Verify if it is the smaller cost (i.e, the shortest path) so far
            if gAttempt < gScore(neighbor(1), neighbor(2))
    
                % Update variables
                cameFrom(neighbor(1), neighbor(2), :) = current;    
                gScore(neighbor(1), neighbor(2)) = gAttempt;        
                fScore(neighbor(1), neighbor(2)) = gAttempt + getManhattanDistance(neighbor, goal);
    
                % Add node to open list
                if ~ismember(neighbor, openList, "rows")
                    openList = [openList; neighbor];
                end   
    
            end
        end   
    end
    if isempty(path)
        error("A*: No path found between start and goal.");
    end
end


% AUXILIARY FUNCTIONS

function [manhattanDistance] = getManhattanDistance(pose, goal)
    manhattanDistance = abs(goal(1) - pose(1)) + abs(goal(2) - pose(2));
end

function [node, openList] = getMinimumFScore(openList, fScore)
    linearIdx = sub2ind(size(fScore), openList(:,1), openList(:,2));
    [~, idx] = min(fScore(linearIdx));
    [row, column] = ind2sub(size(fScore), linearIdx(idx));
    node = [row, column];
    openList_idx = find(ismember(openList, node, 'rows') == 1);
    openList(openList_idx, :) = [];
end

function [neighbors] = getNeighbors(node, map)

    dirs = [-1 -1;
             0 -1;
             1 -1;
            -1  0;
             1  0;
            -1  1;
             0  1;
             1  1];

    neighbors = [];

    for i = 1:length(dirs)
        n = node + dirs(i,:);

        if n(1) < 1 || n(2) < 1
            continue
        end

        if n(1) > size(map,1) || n(2) > size(map,2)
            continue
        end

        if map(n(1),n(2)) == 1
            continue
        end

        neighbors = [neighbors; n];

    end
end

function [distance] = getNeighborDistance(neighbor, current)

    if ((neighbor(1) == current(1)) || (neighbor(2) == current(2)))
        distance = 1;
    else
        distance = sqrt(2);
    end

end

function path = reconstructPath(start, current, cameFrom)

    path = [current];

    while ~isequal(current, start)

        x = cameFrom(current(1), current(2), 1);
        y = cameFrom(current(1), current(2), 2);

        current = [x, y];

        path = [current; path];

    end
    
end
