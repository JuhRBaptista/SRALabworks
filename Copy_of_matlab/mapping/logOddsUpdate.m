function logOdds = logOddsUpdate(logOdds, robotGrid, occGrid, freeGrid, params)
    % logOddsUpdate - Updates occupancy probabilities using log-odds.

    % Log-odds update parameters
    l_occ  =  0.65;
    l_free = -0.15;
    l_min  = -5.0;
    l_max  =  5.0;
    
    % Robot grid position
    x0 = robotGrid(1);
    y0 = robotGrid(2);
    width  = params.size(1);
    height = params.size(2);

    % Process occupied and free-space rays
    pts = [occGrid; freeGrid];
    for i = 1:size(pts, 1)

        % Compute cells crossed by the ray
        freeCells = bresenham(x0, y0, pts(i,1), pts(i,2));

        % Keep only valid map cells
        valid = freeCells(:,1) > 0 & freeCells(:,1) <= width & ...
                freeCells(:,2) > 0 & freeCells(:,2) <= height;

        freeCells = freeCells(valid, :);

        % Update free-space probabilities
        if ~isempty(freeCells)
            idx = sub2ind([width height], freeCells(:,1), freeCells(:,2));
            logOdds(idx) = logOdds(idx) + l_free;
        end
    end
    
    % Occupied cell coordinates
    x1 = occGrid(:,1);
    y1 = occGrid(:,2);
    
    % Keep only valid occupied cells
    validOcc = x1 > 0 & x1 <= width & y1 > 0 & y1 <= height;

    % Update occupied probabilities
    occIdx = sub2ind([width height], x1(validOcc), y1(validOcc));
    logOdds(occIdx) = logOdds(occIdx) + l_occ;

    logOdds = clamp(logOdds, l_min, l_max);
end

function x = clamp(x, mn, mx)
    x = max(mn, min(mx, x));
end