function [map, robotGrid] = mapUpdate(map, data, idx_occ, idx_free, pose, params)
    % mapUpdate- Pure map update logic

    % Extract occupied lidar points and filter invalid scans
    occ_scans = data.Cartesian(idx_occ, :);
    occ_scans = scanFilter(occ_scans);
    
    % Generate free-space points using truncated lidar rays
    free_scans = truncateFreeRays(data, idx_free, params.lidarMaxRange);

    % Convert scan points from robot frame to world frame
    occWorldPts  = scanToWorld(occ_scans, pose);
    freeWorldPts  = scanToWorld(free_scans, pose);
    
    % Convert world coordinates to grid coordinates
    occGridPts   = worldToGrid(occWorldPts, params);
    freeGridPts   = worldToGrid(freeWorldPts, params);
    
    % Robot position in grid coordinates
    robotGrid = worldToGrid([pose.x, pose.y], params);

    % Map update strategy
    switch params.update
        case "bayesian"
            % Bayesian occupancy update using log-odds
            map.logOdds   = logOddsUpdate(map.logOdds, robotGrid, occGridPts, freeGridPts, params);
            map.prob      = 1 ./ (1 + exp(-map.logOdds));
        otherwise  % "binary"
           % Mark occupied cells directly in the occupancy grid
           x1 = occGridPts(:, 1);
           y1 = occGridPts(:, 2);

           validOcc = x1 > 0 & x1 <= params.size & y1 > 0 & y1 <= params.size;
           idx      = sub2ind([params.size params.size], x1(validOcc), y1(validOcc));
           map.prob(idx) = 1;
    end
end

function scan = scanFilter(scan)
    % scanFilter - Removes invalid or out-of-range lidar measurements.
    ranges = sqrt(scan(:,1).^2 + scan(:,2).^2);

    maxRange = 3.5;
    minRange = 0.1;

    valid = ~isnan(ranges) & ~isinf(ranges) & ...
            ranges < maxRange & ranges > minRange;

    scan = scan(valid, :);

end

function freePts = truncateFreeRays(data, idx, maxRange)
    % truncateFreeRays - Generates free-space endpoints for lidar rays.
    angles = data.Angles(idx);
    freePts = [maxRange * cos(angles), maxRange * sin(angles)];
end