function [map, robotGrid] = mapUpdate(map, scan, pose, params, strategy)
    % mapUpdate- Pure map update logic
    %
    % Takes the current map, a lidar scan, and the robot pose,
    % and returns an updated map using the selected strategy.
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

    % GEOMETRY
    scan = scanFilter(scan);

    worldPts  = scanToWorld(scan, pose);
    occGrid   = unique(worldToGrid(worldPts, params), 'rows');
    robotGrid = worldToGrid([pose.x, pose.y], params);

    % STRATEGY
    switch strategy
        case "logodds"
            freeGrid      = worldToGrid(truncateFreeRays(scan, params), params);
            map.logOdds   = logOddsUpdate(map.logOdds, robotGrid, occGrid, freeGrid, params);
            map.prob      = 1 ./ (1 + exp(-map.logOdds));
        otherwise  % "binary"
           x1 = occGrid(:, 1);
           y1 = occGrid(:, 2);

           validOcc = x1 > 0 & x1 <= params.mapSize & y1 > 0 & y1 <= params.mapSize;
           idx      = sub2ind([params.mapSize params.mapSize], x1(validOcc), y1(validOcc));
           map.prob(idx) = 1;
    end
end