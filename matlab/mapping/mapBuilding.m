function mapBuilding(tbot, params, savePath)

    % INIT
    map.logOdds = zeros(params.mapSize, params.mapSize);
    map.prob    = 0.5 * ones(params.mapSize, params.mapSize);

    state.exit = false;

    ui = plotManager(map.prob, params);

    r = rateControl(params.rate);

    set(gcf, 'KeyPressFcn', @keyboardControl);

    global control;
    control.v = 0;
    control.w = 0;
    control.exit = false;

    frame = 0;

    while ~control.exit

        frame = frame + 1;

        % SENSORS 
        [pose.x, pose.y, theta] = tbot.readPose();
        pose.theta = normalizeAngle(theta);

        [~, data] = tbot.readLidar();
        idx = tbot.getInRangeLidarDataIdx(data);
        scan = data.Cartesian(idx, :);

        scan = scanFilter(scan);

        % GEOMETRY 
        worldPts = scanToWorld(scan, pose);
        gridPts = unique(worldToGrid(worldPts, params), 'rows');
        robotGrid = worldToGrid([pose.x, pose.y], params);
        
        % MAPPING 
        map.logOdds = logOddsUpdate(map.logOdds, robotGrid, gridPts, params);

        map.prob = 1 ./ (1 + exp(-map.logOdds));

        % VISUALIZATION 
        if mod(frame, params.plotSkip) == 0
            plotManagerUpdate(ui, map.prob, robotGrid, pose.theta);
        end

        % CONTROL
        tbot.setVelocity(control.v, control.w);

        waitfor(r);
    end

    % SAVE
    binaryMap = map.prob > 0.5;

    imwrite(uint8(flipud(1 - map.prob) * 255), savePath + ".png");
    save(savePath + ".mat", 'map', 'binaryMap');

    close(ui.fig);
    tbot.stop();
end