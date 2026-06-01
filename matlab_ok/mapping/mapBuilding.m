function mapBuilding(tbot, mapParams, params, savePath)
    % mapBuilding  Teleoperated robot mapping using LiDAR and occupancy grid.
    %
    %   Inputs:
    %     tbot      - Robot interface object
    %     mapParams - Map configuration (initial probabilities, log-odds, etc.)
    %     params    - Runtime parameters (e.g., control loop rate)
    %     savePath  - File path to save the resulting map matrix

    % Initialization
    r   = rateControl(params.rate);
    map = initMap(mapParams);

    % Teleoperation State (global keyboard control)
    global v w exitFlag;
    v        = 0;
    w        = 0;
    exitFlag = false;

    % Graphics Setup
    hFig = figure('KeyPressFcn', @keyboardHandle);
    [hImg, hRobot, hArrow] = initGraphics(hFig, map.prob);

    % Main Control Loop
    while ~exitFlag

        % 1) Read robot state
        [pose.x, pose.y, pose.theta] = tbot.readPose();
        pose.theta = normalizeAngle(pose.theta);

        % 2) Read and classify LiDAR data
        [~, data]  = tbot.readLidar();
        idx_occ    = tbot.getInRangeLidarDataIdx(data);
        idx_free   = tbot.getOutRangeLidarDataIdx(data);

        % 3) Update occupancy map
        [map, robotGrid] = mapUpdate(map, data, idx_occ, idx_free, pose, mapParams);

        % 4) Render and send velocity command
        updateGraphics(hImg, hRobot, hArrow, map.prob', robotGrid(1), robotGrid(2), pose.theta);
        tbot.setVelocity(v, w);

        waitfor(r);
    end

    % Save Results and Clean Up
    saveResults(map.prob, savePath);
    statistics(r);
    close(hFig);
    tbot.stop();
end

function map = initMap(mapParams)
    % Builds the initial map structure from mapParams.
    map.prob    = mapParams.initialProb;
    map.logOdds = mapParams.initialLogOdds;
end

function [hImg, hRobot, hArrow] = initGraphics(hFig, prob)
    % Sets up the occupancy grid display and robot marker.
    figure(hFig);
    hImg = imagesc(prob);
    set(gca, 'YDir', 'normal');
    colormap(flipud(gray));
    axis equal;
    hold on;
    hRobot = plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    hArrow = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2);
end

function updateGraphics(hImg, hRobot, hArrow, map, xR, yR, theta)
    % Refreshes the map image and robot pose indicator.
    set(hImg,   'CData', map);
    set(hRobot, 'XData', xR, 'YData', yR);

    arrowLength = 10;
    dx = arrowLength * cos(theta);
    dy = arrowLength * sin(theta);
    set(hArrow, 'XData', xR, 'YData', yR, 'UData', dx, 'VData', dy);

    drawnow limitrate;
end

function saveResults(mapProb, savePath)
    % Exports the occupancy grid as both a PNG image and .mat file.
    imgData = uint8(flipud(1 - mapProb) * 255);
    imwrite(imgData, [savePath, '.png']);
    save([savePath, '.mat'], 'mapProb');
end

function keyboardHandle(~, eventData)
    % Adjusts robot velocity via arrow keys; ESC exits the loop.

    global v w exitFlag;
    dv = 0.025;   % linear velocity increment  (m/s)
    dw = 0.05;    % angular velocity increment (rad/s)

    switch eventData.Key
        case 'uparrow',    v = v + dv;
        case 'downarrow',  v = v - dv;
        case 'rightarrow', w = w - dw;
        case 'leftarrow',  w = w + dw;
        case 'space',      v = 0;  w = 0;
        case 'escape',     v = 0;  w = 0;  exitFlag = true;
    end
end