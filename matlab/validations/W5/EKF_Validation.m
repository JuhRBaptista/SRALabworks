rosshutdown; clear; close all;

% path = pathPlanning("ymap", "../data/ymap_path");

% Definitions 
params.kv = 2.0;
params.ki = 0.1;
params.ks = 3.0;
params.distance = 0.1;
params.vMax = 0.18;
params.wMax = 2.8;
params.rate = 50;
params.T = 6000;
params.dt = 0.05;
params.toleranceError = 0.2;

mapParams.map = loadMap("ymap");
mapParams.scale = 20;
mapParams.origin = 0;
mapParams.size = 80;
mapParams.maxRange = 3.5;

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
data     = load("../data/ymap_path");
path     = data.path;
xWorld   = (path(:,1) - mapParams.origin) / mapParams.scale;
yWorld   = (path(:,2) - mapParams.origin) / mapParams.scale;
pathWorld = [xWorld, yWorld];


% Initialize turtlebot
tbot = connectRobot("sim");
tbot.setPose(pathWorld(1, 1), pathWorld(1, 2), 0);

% Initialize Plot
handles = setupPathTrackingPlot(mapParams.map, pathWorld, mapParams.origin, mapParams.scale);

% Control
PathTrackingControl(tbot, params, pathWorld, handles, avoidance, mapParams, avoidParams);
