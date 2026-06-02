rosshutdown; clear; close all;

% path = pathPlanning("rmap", "../data/rmap_coords")

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
params.ekf = false;
params.estimatePose = false;

map = loadMap("rmap");
map = padarray(map, 20, 0, 'both');
log_odds_map = log(map ./ (1 - map));
log_odds_map= max(-5, min(5, log_odds_map));
mapParams.map = map;
mapParams.update = "bayesian";
mapParams.initialProb = map;
mapParams.initialLogOdds = log_odds_map;
mapParams.scale = 40;
mapParams.origin = 0;
mapParams.size = [140, 100];
mapParams.maxRange = 3;
mapParams.lidarMaxRange = 2;

avoidance = "vfh";
avoidParams.windowSize     = 10; 
avoidParams.sectorWidth    = pi/36;                 % angular resolution (~5 deg)
avoidParams.numSectors     = round(2*pi / avoidParams.sectorWidth);
avoidParams.valleyMinWidth = 18;
avoidParams.threshold      = 0.2;
avoidParams.smoothSigma    = 0.75;

% avoidance = "vff";
% avoidParams.kAtt = 1.0;
% avoidParams.kRep = 2.0;
% avoidParams.windowSize = 10;

% Path
data     = load("../data/rmap_coords");
path = data.path;
xWorld   = (path(:, 1) - mapParams.origin)/mapParams.scale;
yWorld   = (path(:, 2) - mapParams.origin)/mapParams.scale;
pathWorld = [xWorld, yWorld];

slam = true;

% Initialize turtlebot
tbot = connectRobot("sim");
tbot.setPose(xWorld(1), yWorld(2), 0);

% Initialize Plot
opts.showTarget = true;
opts.showVFH = true;
opts.showCovariance = true;
opts.showGroundTruth = true; 
handles = setupPlot(mapParams.initialLogOdds', [], mapParams.origin, mapParams.scale, opts);

% Control
savePath = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'rmap_updated_real');
PathTrackingControl(tbot, params, pathWorld, handles, avoidance, mapParams, avoidParams, slam, savePath);
