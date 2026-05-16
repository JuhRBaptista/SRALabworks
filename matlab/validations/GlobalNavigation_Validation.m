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
map    = load("../data/mapa.mat").map';
params.origin = 175;
params.scale  = 20;

% Path
data     = load("path.mat");
path     = data.path;
xWorld   = (path(:,2) - params.origin) / params.scale;
yWorld   = (path(:,1) - params.origin) / params.scale;
pathWorld = [xWorld, yWorld];

% Inicializa robô
tbot = connectRobot("sim");
tbot.setPose(pathWorld(1,1), pathWorld(1,2), 0);

% Initialize Plot
handles = setupPathTrackingPlot(map, pathWorld, params.origin, params.scale);
 
% Control
PathTrackingControl(tbot, params, pathWorld, handles);
