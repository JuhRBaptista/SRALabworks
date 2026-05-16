rosshutdown; close all; clear all; 

% Definitions  
params.target = [1, 1, pi];
params.kpRho = 1.5;
params.kpAlpha = 2;
params.kpBeta = -0.4;
params.vMax = 0.18;
params.wMax = 2.8;
params.rate = 5;  
params.maxIterations = 500;  
params.toleranceErrorAngle = 0.4;
params.toleranceErrorDist = 0.05;

% Initialize turtlebot
tbot = connectRobot("sim");
tbot.setPose(0, 0, 0);

% Wait for odometry to reset 
resetOdometry(tbot, [0, 0, 0]);

% Control
PoseToPoseControl(tbot, params);