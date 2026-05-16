function mapBuilding(tbot, mapParams, params, savePath)

    % INIT
    map_size = mapParams.size;

    map.logOdds = zeros(map_size, map_size);
    map.prob    = zeros(map_size, map_size);

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

        [map, robotGrid] = mapUpdate(map, scan, pose, mapParams);

        % VISUALIZATION 
        plotManagerUpdate(ui, map.prob, robotGrid, pose.theta);

        % CONTROL
        tbot.setVelocity(control.v, control.w);

        waitfor(r);
    end

    % SAVE
    binaryMap = (map.prob > 0.5);

    imwrite(uint8(flipud(1 - map.prob) * 255), savePath + ".png");
    save(savePath + ".mat", 'map.prob');

    close(ui.fig);
    tbot.stop();
end