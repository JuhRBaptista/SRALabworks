rosshutdown;

% Parameters
mapParams.scale   = 20;
mapParams.origin  = 0;
mapParams.size = 80;
mapParams.update = "binary";
params.rate    = 5;

% Initialize tbot
tbot = connectRobot("sim");
tbot.setPose(0, 0, 0);

mapBuilding(tbot, mapParams, params, "../data/house_map");
