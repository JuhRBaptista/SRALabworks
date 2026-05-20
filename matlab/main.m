rosshutdown; clear; close all;

% path = pathPlanning("../data/house.png", "../data/house_path2", true)
% 
% Definitions 
params.kv = 2.0;
params.ki = 0.1;
params.ks = 3.0;
params.distance = 0.1;
params.vMax = 0.14;
params.wMax = 2.8;
params.rate = 50;
params.T = 6000;
params.dt = 0.05;
params.toleranceError = 0.2;
params.ekf = true;
params.estimatePose = true;

mapParams.map = loadMap("../data/house.png", true);
mapParams.scale = 20;
mapParams.origin = 175;
mapParams.size = 350;
mapParams.maxRange = 3;

avoidance = "vfh";
avoidParams.windowSize    = 10; 
avoidParams.sectorWidth    = pi/36;                 % angular resolution (~5 deg)
avoidParams.numSectors     = round(2*pi / avoidParams.sectorWidth);
avoidParams.valleyMinWidth = 18;
avoidParams.threshold      = 0.6;
avoidParams.smoothSigma    = 1.5;

% avoidance = "vff";
% avoidParams.kAtt = 1.0;
% avoidParams.kRep = 2.0;
% avoidParams.windowSize = 10;

% Path
path     = [156, 192;
            155, 259;
            90, 260;
            90, 192];

xWorld   = (path(:,1) - mapParams.origin) / mapParams.scale;
yWorld   = (path(:,2) - mapParams.origin) / mapParams.scale;

pathWorld = [xWorld, yWorld];

start = gridToWorld([90, 192], mapParams);

% Initialize turtlebot
tbot = connectRobot("sim");
tbot.setPose(start(1), start(2), 0);

% Initialize Plot
opts.showTarget = true;
opts.showVFH = true;
opts.showCovariance = true;
opts.showGroundTruth = true; 
handles = setupPlot(mapParams.map, [], mapParams.origin, mapParams.scale, opts);

% Control
PathTrackingControl(tbot, params, pathWorld, handles, avoidance, mapParams, avoidParams);
