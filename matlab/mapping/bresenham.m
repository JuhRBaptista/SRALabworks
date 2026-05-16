function cells = bresenham(xOrigin, yOrigin, xTarget, yTarget)
    
    cells = [];
    x0 = xOrigin;
    y0 = yOrigin;

    x1 = xTarget;
    y1 = yTarget;

    steep = abs(y1 - y0) > abs(x1 - x0);
    
    % If steep, reflect across y=x so we always iterate over the longer axis
    if steep
        [x0, y0] = deal(y0, x0);
        [x1, y1] = deal(y1, x1);
    end
    
    % Always go left to right
    if x0 > x1
        [x0, x1] = deal(x1, x0);
        [y0, y1] = deal(y1, y0);
    end
    
    dx = x1 - x0;
    dir = sign(y1 - y0);
    dy = dir*(y1 - y0);
   
    p = 2*dy - dx;
    y = y0;
    
    for x = x0:x1

        if steep
            if (x == yTarget && y == xTarget)
                continue;
            end
            cells = [cells; [y, x]];  % undo the reflection
        else
            if (x == xTarget && y == yTarget)
                continue;
            end
            cells = [cells; [x, y]];
        end
        
        if p >= 0
            y = y + dir;
            p = p - 2*dx;
        end
        p = p + 2*dy;
    end
end