rosshutdown; clear; close all;


% Build path
x_center = 0;
y_center = 0;
R = 1;
N = 100;

theta = linspace(0,2*pi,N);

x_path = x_center + R*cos(theta);
y_path = y_center + R*sin(theta);

path = [x_path' y_path'];

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

% Initialize turtlebot
tbot = connectRobot("sim");
tbot.setPose(0, 0, 0);

% Control
PathTrackingControl(tbot, params, path);
