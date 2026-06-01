rosshutdown;

% Parameters
mapParams.scale   = 20;
mapParams.origin  = 0;
mapParams.size = 80;
mapParams.update = "binary";
mapParams.lidarMaxRange = 3.5;
mapParams.initialProb = zeros(mapParams.size, mapParams.size);
mapParams.initialLogOdds = zeros(mapParams.size, mapParams.size);

params.rate = 5;

% Initialize tbot
tbot = connectRobot("sim");
tbot.setPose(0, 0, 0);

mapBuilding(tbot, mapParams, params, '../data/binary_ymap_test');
