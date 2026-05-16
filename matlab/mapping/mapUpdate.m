function [map, robotGrid] = mapUpdate(map, scan, pose, params)
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

    scan = scanFilter(scan);

    % GEOMETRY
    worldPts  = scanToWorld(scan, pose);
    occGrid   = unique(worldToGrid(worldPts, params), 'rows');
    robotGrid = worldToGrid([pose.x, pose.y], params);

    % STRATEGY
    switch params.update
        case "logodds"
            freeGrid      = worldToGrid(truncateFreeRays(scan, params), params);
            map.logOdds   = logOddsUpdate(map.logOdds, robotGrid, occGrid, freeGrid, params);
            map.prob      = 1 ./ (1 + exp(-map.logOdds));
        otherwise  % "binary"
           x1 = occGrid(:, 1);
           y1 = occGrid(:, 2);

           validOcc = x1 > 0 & x1 <= params.size & y1 > 0 & y1 <= params.size;
           idx      = sub2ind([params.size params.size], x1(validOcc), y1(validOcc));
           map.prob(idx) = 1;
    end
end

function scan = scanFilter(scan)

    ranges = sqrt(scan(:,1).^2 + scan(:,2).^2);

    maxRange = 2.0;
    minRange = 0.1;

    valid = ~isnan(ranges) & ~isinf(ranges) & ...
            ranges < maxRange & ranges > minRange;

    scan = scan(valid, :);

end

function freePts = truncateFreeRays(data, idx, maxRange)
    angles = data.Angles(idx);
    freePts = [maxRange * cos(angles), maxRange * sin(angles)];
end