function cells = bresenham(xOrigin, yOrigin, xTarget, yTarget)
    % bresenham - Computes grid cells crossed by a line segment.

    cells = [];

    % Initial point
    x0 = xOrigin;
    y0 = yOrigin;
    
    % Target point
    x1 = xTarget;
    y1 = yTarget;
    
    % Check if line slope is steep
    steep = abs(y1 - y0) > abs(x1 - x0);
    
    % Reflect line across y = x for steep slopes
    if steep
        [x0, y0] = deal(y0, x0);
        [x1, y1] = deal(y1, x1);
    end
    
    % Ensure iteration from left to right
    if x0 > x1
        [x0, x1] = deal(x1, x0);
        [y0, y1] = deal(y1, y0);
    end
    
    % Line increments
    dx = x1 - x0;
    dir = sign(y1 - y0);
    dy = dir*(y1 - y0);
    
    % Bresenham decision parameter
    p = 2*dy - dx;

    y = y0;
    
    for x = x0:x1

        if steep
            % Skip target cell
            if (x == yTarget && y == xTarget)
                continue;
            end
            % Undo reflection before storing cells
            cells = [cells; [y, x]];  
        else
            % Skip target cell
            if (x == xTarget && y == yTarget)
                continue;
            end
            cells = [cells; [x, y]];
        end
        
        % Update decision parameter
        if p >= 0
            y = y + dir;
            p = p - 2*dx;
        end
        p = p + 2*dy;
    end
end