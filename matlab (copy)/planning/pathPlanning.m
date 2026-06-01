function path = pathPlanning(mapName, savePath, flip,  robot_radius, scale)

    arguments
        mapName
        savePath     = []
        flip         = false;
        robot_radius = 0.105
        scale        = 20  
    end

    % Prepare map
    map          = loadMap(mapName, flip);
    robot_pixels = round(robot_radius * scale);
    se           = strel('disk', robot_pixels);
    map          = imdilate(map, se);

    pts = selectPoints(map);

    path = [];
    for i=1:size(pts, 1) - 1
        start = pts(i, :);
        goal = pts(i + 1, :);
        subPath          = aStar(map, start, goal);
        path = [path; subPath]; 
    end    

    showPath(map, path, pts(1, :), pts(end, :));

    if ~isempty(savePath)
        save(savePath, 'path');
    end
end

% Auxiliary functions
function [points] = selectPoints(map)
    figure;
    showMap(map, 'Click points (ENTER to finish) - GREEN=first, BLUE=last');
    disp('Click points on the map. Press ENTER when done.');
    
    points = [];
    colors = {'go', 'bo'};  % first = green, others = blue
    
    while true
        [x, y, button] = ginput(1);
        
        % ENTER key or figure closed
        if isempty(x) || button == 13
            break;
        end
        
        pt = [round(y), round(x)];
        points = [points; pt];
        
        n = size(points, 1);
        
        % First point green, rest blue
        if n == 1
            plot(pt(2), pt(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
        else
            % Repaint previous last point as blue (intermediate)
            if n > 2
                prev = points(n-1, :);
                plot(prev(2), prev(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
            end
            plot(pt(2), pt(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
        end
        
        % Draw line between consecutive points
        if n > 1
            prev = points(n-1, :);
            plot([prev(2), pt(2)], [prev(1), pt(1)], 'y--', 'LineWidth', 1.5);
        end
        
        fprintf('Point %d: [%d, %d]\n', n, pt(1), pt(2));
        drawnow;
    end
    
    % Repaint last point in blue
    if size(points, 1) >= 2
        last = points(end, :);
        plot(last(2), last(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    end
    
    fprintf('Total points selected: %d\n', size(points, 1));
    pause(1);
    close;
end

function showPath(map, path, start, goal)
    figure;
    showMap(map, 'Planned Path');
    hold on;

    if ~isempty(path)
        plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);
    end

    plot(start(2), start(1), 'go', 'MarkerFaceColor', 'g');
    plot(goal(2),  goal(1),  'bo', 'MarkerFaceColor', 'b');
end