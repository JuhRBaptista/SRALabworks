function [fAttractive, fRepulsive] = VFF(robotPose, targetPose, mapParams, avoidParams)
    % VFF - Computes attractive and repulsive forces for obstacle avoidance.
    
    % mapParams
    %   map           : occupancy grid map
    %   scale         : map scale to real world
    %   origin        : map origin

    % avoidParams
    %   kAtt          : attractive force gain
    %   kRep          : repulsive force gain
    %   windowSize    : local obstacle search window

    % Robot position in occupancy grid
    robotGridX = round(robotPose(1) * mapParams.scale + mapParams.origin);
    robotGridY = round(robotPose(2) * mapParams.scale + mapParams.origin);

    % Attractive force toward target
    delta = targetPose - robotPose;
    dist  = norm(delta);

    if dist > 0
        fAttractive = avoidParams.kAtt * (delta / dist);
    else
        fAttractive = [0, 0];
    end

    % Initialize repulsive force
    fRepulsive = [0, 0];

    % Local search window
    halfWindow = floor(avoidParams.windowSize / 2);

    [mapH, mapW] = size(mapParams.map);

    xMin = max(1, robotGridX - halfWindow);
    xMax = min(mapH, robotGridX + halfWindow);
    yMin = max(1, robotGridY - halfWindow);
    yMax = min(mapW, robotGridY + halfWindow);

    % Evaluate occupied cells inside the local window
    for x = xMin:xMax
        for y = yMin:yMax

            occValue = mapParams.map(x, y);
            if occValue == 0
                continue; % skip free cells
            end

            dx = x - robotGridX;
            dy = y - robotGridY;
            d  = norm([dx, dy]);

            % Avoid singularity at robot position
            if d == 0
                continue; 
            end

             % Repulsive force contribution
            magnitude = avoidParams.kRep * occValue / (d^2);
            direction = [dx, dy] / d;

            fRepulsive = fRepulsive - magnitude * direction;
        end
    end
end