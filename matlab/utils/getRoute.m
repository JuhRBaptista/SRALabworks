function [p, path] = getRoute(tbot, targets, mapParams, p0)
    if nargin >= 4 && ~isempty(p0)
        p = p0(:);          % já localizado em main.m, não repete
    else
        [~, lddata, ~] = tbot.readLidar();
        p = globalLocalize(lddata, mapParams.map, mapParams.scale, ...
                           mapParams.origin, mapParams.maxRange, [-0.0305; 0]);
        p(3) = normalizeAngle(p(3));
    end
    % ... resto inalterado

    % Convert estimated pose to grid coordinates
    x = round((p(1) * mapParams.scale) + mapParams.origin);
    y = round((p(2) * mapParams.scale) + mapParams.origin);
    gridPose = [x, y];
    
    gridPath = [];

    % Plan A* path through each target sequentially
    for i=1:size(targets, 1)
        target = targets(i, :);

        % Convert target to grid coordinates
        x = round((target(1) * mapParams.scale) + mapParams.origin);
        y = round((target(2) * mapParams.scale) + mapParams.origin);
        gridTarget = [x, y];
        
        % Compute optimal path from current grid pose to target
        subpath = aStar(mapParams.map, gridPose, gridTarget);
        gridPath = [gridPath; subpath];

        % Advance starting point to current target
        gridPose = gridTarget;
    end

    % Convert full grid path back to world coordinates
    xWorld   = (gridPath(:,1) - mapParams.origin) / mapParams.scale;
    yWorld   = (gridPath(:,2) - mapParams.origin) / mapParams.scale;
    
    path = [xWorld, yWorld];
end