function prob = binaryUpdate(prob, robotGrid, occGrid, params)
    % binaryUpdate - Simple binary occupancy grid update (W2).
    %
    % Marks obstacle cells as 1. Traces free cells along each
    % Bresenham ray, setting them to 0 only if not yet occupied.
    %
    % Inputs:
    %   prob      : current occupancy grid (mapSize x mapSize)
    %   robotGrid : [x, y] robot position in grid coordinates
    %   occGrid   : Nx2 matrix of occupied cell coordinates
    %   params    : struct with field mapSize
    %
    % Output:
    %   prob      : updated occupancy grid

    M  = params.mapSize;
    x0 = robotGrid(1);
    y0 = robotGrid(2);

    % Mark occupied cells
    x1 = occGrid(:, 1);
    y1 = occGrid(:, 2);

    validOcc = x1 > 0 & x1 <= M & y1 > 0 & y1 <= M;
    idx      = sub2ind([M M], x1(validOcc), y1(validOcc));
    prob(idx) = 1;
end