function logOdds = logOddsUpdate(logOdds, robotGrid, occGrid, freeGrid, params)
    
    l_occ  =  0.65;
    l_free = -0.15;
    l_min  = -5.0;
    l_max  =  5.0;

    x0 = robotGrid(1);
    y0 = robotGrid(2);
    M  = params.mapSize;

    % Free cells via Bresenham (usa freeGrid, que inclui raios truncados)
    pts = [occGrid; freeGrid];
    for i = 1:size(pts, 1)
        freeCells = bresenham(x0, y0, pts(i,1), pts(i,2));
        valid = freeCells(:,1) > 0 & freeCells(:,1) <= M & ...
                freeCells(:,2) > 0 & freeCells(:,2) <= M;
        freeCells = freeCells(valid, :);
        if ~isempty(freeCells)
            idx = sub2ind([M M], freeCells(:,1), freeCells(:,2));
            logOdds(idx) = logOdds(idx) + l_free;
        end
    end

    x1 = occGrid(:,1);
    y1 = occGrid(:,2);

    validOcc = x1 > 0 & x1 <= M & y1 > 0 & y1 <= M;
    occIdx = sub2ind([M M], x1(validOcc), y1(validOcc));
    logOdds(occIdx) = logOdds(occIdx) + l_occ;

    logOdds = clamp(logOdds, l_min, l_max);
end
function x = clamp(x, mn, mx)
    x = max(mn, min(mx, x));
end