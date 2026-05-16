rosshutdown; clear; close all;

% Definitions
params.kv = 2.0;
params.ki = 0.1;
params.ks = 3.0;
params.distance = 0.1;
params.vMax = 0.18;
params.wMax = 2.8;
params.rate = 5;
params.T = 60;
params.dt = 0.05;
params.toleranceError = 0.2;

% Map
map    = load("../data/binary_ymap_test.mat").mapProb;
mapParams.origin = 0;
mapParams.scale  = 20;
mapParams.size = 80;

% Path
data     = load("../data/binary_ymap_path.mat");
path     = data.path;
xWorld   = (path(:,1) - mapParams.origin) / mapParams.scale;
yWorld   = (path(:,2) - mapParams.origin) / mapParams.scale;
pathWorld = [xWorld, yWorld];

% Inicializa robô
tbot = connectRobot("sim");
tbot.setPose(pathWorld(1,1), pathWorld(1,2), 0);

% Initialize Plot
handles = setupPathTrackingPlot(map, pathWorld, mapParams.origin, mapParams.scale);

% Control
PathTrackingControl(tbot, params, pathWorld, handles);
