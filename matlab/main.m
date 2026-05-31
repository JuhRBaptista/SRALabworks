rosshutdown; clear; close all;

% path = pathPlanning("rmap", "../data/rmap")

% Definitions 
params.kv = 2.0;
params.ki = 0.1;
params.ks = 3.0;
params.distance = 0.1;
params.vMax = 0.05;
params.wMax = 2.8;
params.rate = 2000;
params.T = 6000;
params.dt = 0.05;
params.toleranceError = 0.15;
params.finalToleranceError = 0.1;
params.ekf = true;
params.estimatePose = false;

map = loadMap("rmap");
log_odds_map = log(map ./ (1 - map));

mapParams.map = map;
mapParams.update = "bayesian";
mapParams.initialProb = map;
mapParams.initialLogOdds = log_odds_map;
mapParams.scale = 40;
mapParams.origin = 0;
mapParams.size = [120, 80];
mapParams.maxRange = 3;
mapParams.lidarMaxRange = 3.5;

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
data     = load("../data/rmap");
path     = data.path;
xWorld   = (path(:,1) - mapParams.origin) / mapParams.scale;
yWorld   = (path(:,2) - mapParams.origin) / mapParams.scale;

pathWorld = [xWorld, yWorld];
slam = false;

% Initialize turtlebot
tbot = connectRobot("sim");
tbot.setPose(xWorld(1), yWorld(1), pi);

% Initialize Plot
opts.showTarget = true;
opts.showVFH = true;
opts.showCovariance = true;
opts.showGroundTruth = true; 
handles = setupPlot(mapParams.initialLogOdds', [], mapParams.origin, mapParams.scale, opts);

% Control
PathTrackingControl(tbot, params, pathWorld, handles, avoidance, mapParams, avoidParams, slam, "../data/rmap_updated");
