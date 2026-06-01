rosshutdown; close all; clear all; 

% Definitions
params.target = [1, 1];
params.kV = 2;
params.kW = 1;
params.vMax = 0.18;
params.wMax = 2.8;
params.rate = 5;
params.maxIterations = 150;
params.toleranceError = 0.2;


% Initialize turtlebot
tbot = connectRobot("sim");
tbot.setPose(0, 0, 0.00);

opts.showMap  = false;
opts.showPath = false;   
handles = setupPlot([], [], 0, 20, opts);

% Control
PointToPointControl(tbot, params, handles);