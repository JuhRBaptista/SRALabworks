function [map, robotGrid] = mapUpdate(map, data, idx_occ, idx_free, pose, params)
    % mapUpdate- Pure map update logic
    %
    % Inputs:
    %   map      : struct with fields .prob (and .logOdds for "logodds")
    %   scan     : Nx2 filtered lidar scan in robot frame
    %   pose     : struct with fields .x .y .theta
    %   params   : struct with mapSize, scale, origin
    %   strategy : "binary" (W2) or "logodds" (W4)
    %
    % Output:
    %   map      : updated map struct

    occ_scans = data.Cartesian(idx_occ, :);
    occ_scans = scanFilter(occ_scans);

    free_scans = truncateFreeRays(data, idx_free, params.lidarMaxRange);

    % 2. UPDATE MAP
    occWorldPts  = scanToWorld(occ_scans, pose);
    freeWorldPts  = scanToWorld(free_scans, pose);

    occGridPts   = worldToGrid(occWorldPts, params);
    freeGridPts   = worldToGrid(freeWorldPts, params);

    robotGrid = worldToGrid([pose.x, pose.y], params);

    % STRATEGY
    switch params.update
        case "bayesian"
            map.logOdds   = logOddsUpdate(map.logOdds, robotGrid, occGridPts, freeGridPts, params);
            map.prob      = 1 ./ (1 + exp(-map.logOdds));
        otherwise  % "binary"
           x1 = occGridPts(:, 1);
           y1 = occGridPts(:, 2);

           validOcc = x1 > 0 & x1 <= params.size & y1 > 0 & y1 <= params.size;
           idx      = sub2ind([params.size params.size], x1(validOcc), y1(validOcc));
           map.prob(idx) = 1;
    end
end

function scan = scanFilter(scan)

    ranges = sqrt(scan(:,1).^2 + scan(:,2).^2);

    maxRange = 3.5;
    minRange = 0.1;

    valid = ~isnan(ranges) & ~isinf(ranges) & ...
            ranges < maxRange & ranges > minRange;

    scan = scan(valid, :);

end

function freePts = truncateFreeRays(data, idx, maxRange)
    angles = data.Angles(idx);
    freePts = [maxRange * cos(angles), maxRange * sin(angles)];
end