% main.m  -  TurtleBot3 SLAM + VFH navigation
%
% KEY FIXES vs. the previous version
%   1. Border walls stamped into the map so the EKF sees wall returns
%      near the arena edges (without this, those beams hit maxRange,
%      Jg = 0, and are skipped -> EKF drifts near walls).
%   2. globalLocalize now returns a residual score; up to loc_retries
%      attempts are made and the pose with the lowest residual is kept.
%   3. Initial covariance Sigma0 is set proportionally to the localisation
%      quality (high residual -> larger uncertainty -> filter stays open).
%   4. ekfIter is passed through params so EKF.m can apply the warm gate
%      for the first warmIters iterations.
%   5. sensor_offset passed to globalLocalize (LiDAR ~3 cm behind wheel axis).

rosshutdown; clear; close all;

% ===== Map ================================================================
map = loadMap("../data/rmap_updated.png", true);

% Stamp 2 cm border walls so the EKF is corrected by the arena boundary.
% Without this, beams that hit the physical walls find maxRange in the
% grid, produce Jg = 0, and are silently discarded -> EKF drifts at edges.
wall_th_m  = 0.02;
scale_val  = 40;            % px / m  (set equal to mapParams.scale below)
tpx        = max(1, round(wall_th_m * scale_val));
map(1:tpx,        :) = 1;
map(end-tpx+1:end,:) = 1;
map(:, 1:tpx)        = 1;
map(:, end-tpx+1:end)= 1;

log_odds_map = log(map ./ (1 - map));
log_odds_map = max(-5, min(5, log_odds_map));

% ===== Map parameters =====================================================
mapParams.map            = map;
mapParams.update         = "bayesian";
mapParams.initialProb    = map;
mapParams.initialLogOdds = log_odds_map;
mapParams.scale          = 40;
mapParams.origin         = 0;
mapParams.size           = [120, 80];
mapParams.maxRange       = 3;
mapParams.lidarMaxRange  = 2;

% ===== Controller parameters ==============================================
params.kv                = 2.0;
params.ki                = 0.1;
params.ks                = 3.0;
params.distance          = 0.1;
params.vMax              = 0.05;
params.wMax              = 2.8;
params.rate              = 2000;
params.T                 = 6000;
params.dt                = 0.05;
params.toleranceError    = 0.15;
params.finalToleranceError = 0.10;
params.ekf               = true;
params.estimatePose      = true;

% ===== Avoidance ==========================================================
avoidance = "vfh";
avoidParams.windowSize    = 10;
avoidParams.sectorWidth   = pi/36;
avoidParams.numSectors    = round(2*pi / avoidParams.sectorWidth);
avoidParams.valleyMinWidth = 18;
avoidParams.threshold     = 0.2;
avoidParams.smoothSigma   = 0.75;

% ===== Target (world coords) ==============================================
pathWorld   = ([21, 60] - mapParams.origin) / mapParams.scale;
initialPose = ([101, 19] - mapParams.origin) / mapParams.scale;

% ===== Connect robot ======================================================
tbot = connectRobot("sim");
tbot.setPose(initialPose(1), initialPose(2), pi);

% ===== Global localisation with retry & quality check ====================
sensor_offset = [-0.0305; 0];   % LiDAR ~3 cm behind wheel axis
loc_retries   = 4;
loc_score_max = 0.12;           % acceptable mean range error [m]

fprintf('\n=== Global localization ===\n');
bestResidual = inf;  pBest = [];
for attempt = 1:loc_retries
    [~, lddata0, ~] = tbot.readLidar();
    [p_try, s_try]  = globalLocalize(lddata0, map, mapParams.scale, ...
                                     mapParams.origin, mapParams.maxRange, ...
                                     sensor_offset);
    if s_try < bestResidual
        bestResidual = s_try;
        pBest        = p_try;
    end
    if bestResidual <= loc_score_max, break; end
    fprintf('  attempt %d: residual=%.3f m (need <= %.3f) - retrying\n', ...
            attempt, s_try, loc_score_max);
end

if isempty(pBest)
    error('Global localization failed after %d attempts.', loc_retries);
end

fprintf('=> pose est: [%.2f m  %.2f m  %.1f deg]  residual=%.3f m\n\n', ...
        pBest(1), pBest(2), rad2deg(pBest(3)), bestResidual);

if bestResidual > loc_score_max
    warning('High localization residual (%.3f m). Starting covariance inflated.', ...
            bestResidual);
end

% Scale initial covariance by localization quality:
% perfect fix (residual=0) -> 1x; at threshold -> 5x; beyond -> more
conf   = min(1, bestResidual / loc_score_max);
Sigma0 = diag([0.20, 0.20, deg2rad(12)].^2) .* (1 + 4*conf);

% Store the EKF state in params so PathTrackingControl can use it.
% The ekfIter field lets EKF.m know when to apply the warm gate.
params.p0     = pBest;
params.Sigma0 = Sigma0;

% ===== Plot setup =========================================================
opts.showTarget      = true;
opts.showVFH         = true;
opts.showCovariance  = true;
opts.showGroundTruth = true;
handles = setupPlot(mapParams.initialLogOdds', [], mapParams.origin, ...
                    mapParams.scale, opts);

% ===== Run navigation =====================================================
slam    = false;
savePath = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'rmap_updated');

PathTrackingControl(tbot, params, pathWorld, handles, avoidance, ...
                    mapParams, avoidParams, slam, savePath);
