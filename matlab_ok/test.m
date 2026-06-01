% demoNavigateWallVFH - Final demo variant: VFH sees the KNOWN walls too.
%   (Copy of demoNavigate.m. Difference: VFH steers on the FULL live grid -
%   prior walls + Bayesian map + current LiDAR - i.e. classic VFH that treats
%   the known walls as obstacles. demoNavigate.m instead hides the walls from
%   VFH. Everything else - SLAM/EKF/A*/diagnostics - is identical.)
% =========================================================================
% ONE integrated, single run that wires EVERY part built in the course:
%
%   SLAM        : globalLocalize (recover the unknown start pose from LiDAR)
%                 + EKF  ekfPredict (odometry) / ekfUpdate (LiDAR vs known map)
%                 + MapUpdate (live Bayesian occupancy map)
%   PLANNING    : A_star on a coarse, robot-inflated grid
%   REACTIVITY  : VFH on the LIVE map (prior walls + Bayesian updates + the
%                 current LiDAR), then PI pure-pursuit -> setVelocity(v,w)
%   ADAPT       : skip A* waypoints blocked by a new obstacle; VFH then steers
%                 to the next reachable waypoint. Live Bayesian map shown inset.
%
% The robot is placed at a random pose; this script localizes, plans and
% drives to the target ONCE, then stops. Re-run the script for each new try
% (the professor records the metrics manually).
%
% Requirements:
%   * Real robot up:  roslaunch turtlebot3_bringup turtlebot3_robot.launch
%     (or Gazebo: ./start-gazebo-rmap.sh).
%   * Arena matches maps/rmap_grid1.png at 0.5 cm/px (3.0 m x 2.0 m).
%   * Set the two IPs and the TARGET below.
% =========================================================================

clearvars -except tbot

% ===== 1) TurtleBot3 connection (edit the IPs) ==========================
if (~exist("tbot", "var"))
    %IP_TURTLEBOT     = "192.168.1.200";     % robot IP (.200/.201 = real robot)
    IP_TURTLEBOT     = "192.168.21.140";
    %IP_HOST_COMPUTER = "192.168.1.115";     % this computer's IP
    IP_HOST_COMPUTER = "192.168.21.1";
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);
    if (tbot.getVersion() < 0.9), error('TurtleBot v09 required'); end
end

normalizeAngle = @(a) atan2(sin(a), cos(a));

% ===== 2) Target + map ==================================================
TARGET   = [0.4, 0.5];          % target point in world coords [m]
%tbot.setPose(4, 0.5, 0); pause(0.5);

mapFile  = 'maps/rmap_grid1.png';   % Demo map: 3.0 x 2.0 m arena
%mapFile  = 'maps/rmap_grid5.png';  % Gazebo 6x4 m map
mscale   = 200;                  % px/m (0.5 cm/px -> 3.0 x 2.0 m arena)
%mscale   = 20;                   % Gazebo scale for rmap_grid5 (6x4 m)
offset_m = 0;                     % world origin at the map corner

img      = imread(mapFile);
mapProb0 = 1 - double(img(:,:,1)) ./ 255;   % obstacle prob in [0,1]
occGrid  = mapProb0 > 0.5;                  % KNOWN prior map (logical, fine)

% The rmap PNG has NO bounding walls, but the physical/Gazebo arena is fully
% enclosed. Add the 4 boundary walls into the map so the EKF expects (and is
% corrected by) the wall LiDAR returns. Without this, every wall beam fails
% the Mahalanobis gate and the EKF drifts ("loses itself") near the edges.
addBorderWalls = true;          % set false only if your map already has walls
wall_th_m      = 0.02;          % wall thickness to stamp into the map [m]
if addBorderWalls
    tpx = max(1, round(wall_th_m * mscale));
    occGrid(1:tpx,        :) = true;        % x = 0 wall  (first rows)
    occGrid(end-tpx+1:end,:) = true;        % x = H/mscale wall (last rows)
    occGrid(:, 1:tpx)        = true;        % y = 0 wall  (first cols)
    occGrid(:, end-tpx+1:end)= true;        % y = W/mscale wall (last cols)
end

[H, W]   = size(occGrid);

r_robot_m = 0.105;              % robot safety radius [m]

% world (x,y) -> fine grid (row,col)  [corner convention, offset_m = 0]
world2grid = @(xy) [round((xy(1)+offset_m)*mscale)+1, ...
                    round((xy(2)+offset_m)*mscale)+1];

% prior obstacle cells for plotting
[ioP, joP] = find(occGrid);
xObsPrior  = (ioP-1)/mscale - offset_m;
yObsPrior  = (joP-1)/mscale - offset_m;

% validate the target
g_rc = world2grid(TARGET);
if any(g_rc < 1) || g_rc(1) > H || g_rc(2) > W || occGrid(g_rc(1), g_rc(2))
    error('TARGET %s is outside the map or inside an obstacle.', mat2str(TARGET));
end

fprintf('\n=== TurtleBot3 navigation demo ===\n');
fprintf('map %s : %dx%d px @ %d px/m -> %.1f x %.1f m\n', ...
        mapFile, H, W, mscale, H/mscale, W/mscale);
fprintf('target = [%.2f %.2f] m\n\n', TARGET(1), TARGET(2));

% ===== 3) Parameters (validated in E11) =================================
% A*/VFH run on a grid down-sampled to ~planMscale px/m. For the fine grid1
% map (200 px/m) this gives a 10x speed-up; for grid5 (20 px/m) the factor is
% 1, i.e. A*/VFH run on the map at native resolution (no down-sampling).
planMscale = 20;                % coarse A*/VFH resolution [px/m]
wpSpacing  = 0.60;              % A* waypoint spacing [m] (larger = fewer waypoints)

% localization
sensor_offset = [-0.0305; 0];   % LiDAR ~3 cm behind the wheel axis
nBeams        = 120;             % beams used by the EKF correction
max_range     = 3.50;
loc_score_max = 0.12;           % acceptable globalLocalize residual [m]
loc_retries   = 4;

% EKF
kr = 5e-3;  kl = 5e-3;
Q  = diag([1e-6, 1e-6, 1e-7]);
b  = tbot.getWheelBaseline();
sigma_R_rel  = 0.035;
sigma_R_base = 0.03;
e_mahal      = 2.5;
e_mahal_warm = 4.0;     % looser gate while the EKF pulls in after globalLocalize
warmIters    = 15;
encoder_noise_std = 0;          % real odometry already noisy (0 = no extra)
Sigma0 = diag([0.20, 0.20, deg2rad(12)].^2);

% Bayesian map update (MapUpdate.m, log-odds)
l0 = 0;  l_occ = 0.9;  l_free = -0.4;
priorOcc = 0.9;  priorFree = 0.1;
mapThresh      = 0.65;          % prob -> obstacle threshold for VFH/replan
mapUpdateEvery = 1;             % run MapUpdate every N loops
mapUpdateStride= 2;            % subsample LiDAR beams fed to MapUpdate
liveGridEvery  = 3;            % rebuild the coarse VFH/A* grid every N loops

% control (PI pure-pursuit on the VFH direction)
cStar = 0.6;  dStar = 0.30;
kv = 0.2;     ki = 0.05;   ks = 0.6;
searchWindow = 0.4;             % VFH active window radius [m] (0.6->0.4: only
                                % react to obstacles within 0.4 m, so the bottom
                                % wall ~0.5 m away no longer repels the robot
                                % off the direct line - "less aggressive")
v_max = 0.15; w_max = 1.8;      % conservative caps for the small arena
slow_radius = 0.25;
distEps     = 0.12;             % waypoint capture radius [m]
distEpsGoal = 0.10;             % final-goal capture radius [m]

% This variant uses CLASSIC VFH: it steers on the full live grid (known walls
% + Bayesian map + current LiDAR), so the prior map walls ARE treated as
% obstacles and produce reactive repulsion. Trade-off: a waypoint/target close
% to a wall can be hard to reach, because VFH (which normalises its histogram)
% reads the wall right behind the goal as "blocked" and deflects away from it.
% (demoNavigate.m removes the walls from VFH to avoid exactly that.)

% adaptive waypoint skipping (handles obstacles that appear AFTER the global
% A* path was computed): if a new obstacle now covers an intermediate
% waypoint, skip it and let VFH steer to the next reachable one. The test is
% purely "is the waypoint cell occupied in the live map", so no extra params.

% loop / safety
rateHz       = 5;
hitTol       = 0.15;            % "reached target" tolerance [m]
collisionDist= 0.14;           % LiDAR proximity reported as a collision [m]
collisionCool= 1.5;            % min seconds between reported collisions
timeLimit    = 300;            % safety cap [s] (re-run the script per attempt)
plotEvery    = 4;

% diagnostics & recovery. In Gazebo the odometry (green) is ground truth, so
% dEKF = ||EKF - odom|| is the EKF's TRUE error; on the real robot it is just
% the EKF-vs-odom disagreement (still a useful divergence indicator). The
% watchdog detects the robot orbiting / the EKF having diverged (no progress
% toward the current waypoint for stuckTimeout s) and recovers by re-running
% globalLocalize + A* from the fresh fix - the cure for "circling short of
% the goal" on a diverged estimate. Set recoverOnStuck=false to disable.
diagEvery      = rateHz;        % print EKF-vs-truth diagnostics ~1x per second
recoverOnStuck = false;         % re-localize + re-plan when progress stalls
                                % (OFF by default: re-localizing on a self-
                                % similar map can jump to another wrong pose)
stuckTimeout   = 8.0;           % [s] with no progress toward the waypoint
stuckImprove   = 0.03;          % [m] closing distance that counts as progress

% EKF divergence guard, keyed on the LiDAR beam-acceptance crash. When the
% pose slips toward a self-similar wrong solution, the predicted ranges stop
% matching the map and the gate rejects most beams (accepted count/fraction
% collapse). Two responses:
%  (1) Do NOT apply an under-determined correction (< minApplyBeams accepted):
%      a handful of beams give an easily-biased update - the seed of the
%      runaway - so coast on the (short-term reliable) odometry instead.
%  (2) If acceptance stays crashed for divHoldSecs, declare divergence and
%      re-localize + re-plan to try to regain lock (rate-limited by cooldown).
% Good runs (beams stay healthy) never trigger this, so they are unchanged.
ekfGuard      = true;           % enable the beam-crash divergence guard
minApplyBeams = 8;              % skip the EKF update below this many accepted beams
accGoodCount  = 12;             % "healthy" accepted-beam count
accGoodFrac   = 0.35;           % "healthy" accepted / valid fraction
divHoldSecs   = 3.0;            % sustained crash before declaring divergence [s]
relocCooldown = 8.0;            % min seconds between recovery re-localizations

% static coarse grids (raw walls + robot-inflated), and the coarse SE
[planInfl, mscalePlan, ~, coarseOcc] = buildPlanGrid(occGrid, mscale, planMscale, r_robot_m);
se_coarse = strel('disk', max(1, ceil(r_robot_m*mscalePlan)), 0);
[Hc, Wc]  = size(coarseOcc);

% ===== 4) SLAM step A: global localization (unknown start) =============
p = [];  loc_score = inf;
for attempt = 1:loc_retries
    [~, lddata0, ~] = tbot.readLidar();
    [p_try, s_try]  = globalLocalize(lddata0, occGrid, mscale, offset_m, ...
                                     max_range, sensor_offset);
    if s_try < loc_score, p = p_try; loc_score = s_try; end
    if loc_score <= loc_score_max, break; end
end
if isempty(p), error('global localization failed.'); end
fprintf('global loc: est=[% .2f % .2f % .1fdeg]  residual=%.3f m\n', ...
        p(1), p(2), rad2deg(p(3)), loc_score);
if loc_score > loc_score_max
    warning('high localization residual (%.3f m) - start may be wrong.', loc_score);
end
conf = min(1, loc_score / loc_score_max);          % 0 = great fix, 1 = at threshold
Sigma = Sigma0 .* (1 + 4*conf);                    % inflate when residual is high

% ===== 5) PLANNING: A* (EKF pose -> target) ============================
[waypts, planOK] = planPath(p(1:2).', TARGET, planInfl, mscalePlan, offset_m, ...
                            wpSpacing);
if ~planOK, error('A*: no path from the start to the target.'); end
nWp = size(waypts, 1);  wp_i = 1;
fprintf('A*: %d waypoints to the target.\n\n', nWp);

% ===== 6) Closed loop: SLAM + reactive control =========================
% live Bayesian probability map, initialized from the prior known map
probMap = priorFree + (priorOcc - priorFree) * double(occGrid);

rate  = rateControl(rateHz);
dt    = rate.DesiredPeriod;
maxIt = ceil(timeLimit*rateHz) + 50;
estTraj  = nan(maxIt, 3);       % EKF pose (drives control)
odomTraj = nan(maxIt, 3);       % raw odometry (reference; drifts)

previousError = 0; IntegralError = 0;
wasNear = false; lastCollT = -inf; collisions = 0;
liveInflD = double(planInfl);   % coarse inflated grid VFH steers on (rebuilt below)
goalReached = false;
bestDWp = inf; lastProgressT = 0; nRecover = 0;     % progress watchdog state
lowAccSince = inf; lastRelocT = -inf;               % beam-crash divergence guard
trueErr = 0; nAcc = 0; nValid = 0;                  % diagnostics (for the title)

tbot.initEncoders();
tbot.setVelocity(0, 0);
figure(1); clf;

it = 0; tic;
while it < maxIt && toc <= timeLimit
    it = it + 1;
    t_now = toc;

    % --- Sensing ---
    [dsr, dsl, pose2D, ~] = tbot.readEncodersWithNoise(encoder_noise_std);
    [~, lddata, ~]        = tbot.readLidar();
    ranges = lddata.Ranges;  angles = lddata.Angles;

    % --- SLAM: EKF predict (odometry) ---
    [p, Sigma] = ekfPredict(p, Sigma, dsr, dsl, b, kr, kl, Q);

    % --- SLAM: EKF correct (subsampled LiDAR vs the KNOWN prior map) ---
    beam_idx = round(linspace(1, numel(ranges), nBeams));
    v_vec = []; Hg = []; R_diag = [];
    nValid = 0; nAcc = 0; sumInnov = 0;   % diag: raycast hits / accepted / |v|
    for k = 1:numel(beam_idx)
        z_i = ranges(beam_idx(k));  theta_i = angles(beam_idx(k));
        if ~isfinite(z_i) || z_i < 0.12 || z_i > max_range, continue; end
        [g_i, hit_pt, valid] = predictRange(p, theta_i, occGrid, mscale, ...
                                            offset_m, max_range, sensor_offset);
        if ~valid, continue; end
        nValid = nValid + 1;
        R_i = (sigma_R_rel*z_i + sigma_R_base)^2;
        H_i = obsJacobian(p, hit_pt, sensor_offset);
        v_i = z_i - g_i;
        s_i = H_i*Sigma*H_i.' + R_i;
        % Mahalanobis gate -> unmapped obstacles don't corrupt localization.
        % The looser warm gate applies during warmup so the EKF can pull in
        % after globalLocalize and tolerate map-vs-reality mismatch early on.
        gate = e_mahal; if it <= warmIters, gate = e_mahal_warm; end
        if (v_i*v_i)/s_i <= gate^2
            v_vec  = [v_vec ; v_i];
            Hg     = [Hg    ; H_i];
            R_diag = [R_diag; R_i];
            nAcc = nAcc + 1; sumInnov = sumInnov + abs(v_i);
        end
    end
    % Apply the correction only if enough beams passed the gate. A handful of
    % accepted beams give an under-determined, easily-biased update (the seed
    % of the divergence runaway) - skip it and coast on odometry instead.
    if ~isempty(v_vec) && (~ekfGuard || nAcc >= minApplyBeams)
        [p, Sigma, ~] = ekfUpdate(p, Sigma, v_vec, Hg, diag(R_diag));
    end

    xr = p(1); yr = p(2); theta = p(3);
    estTraj(it,:)  = p(:).';
    odomTraj(it,:) = pose2D(:).';

    % --- Diagnostics: EKF vs odometry (ground truth in Gazebo) ----------
    trueErr     = norm(p(1:2).' - pose2D(1:2));
    trueHeadErr = abs(normalizeAngle(p(3) - pose2D(3)));
    if mod(it, diagEvery) == 0
        meanInnov = sumInnov / max(1, nAcc);
        fprintf(['t=%5.1fs  dEKF-odom=%.2fm hdg=%4.1fdeg  beams=%3d/%3d acc  ' ...
                 'meanInnov=%.3fm  d(goal)=%.2fm\n'], ...
                t_now, trueErr, rad2deg(trueHeadErr), nAcc, nValid, ...
                meanInnov, norm([xr yr]-TARGET));
    end

    % --- EKF divergence guard: watch the beam-acceptance health ---------
    % Healthy = plenty of beams pass the gate. Once acceptance crashes and
    % stays crashed (the pose has slipped), lowAccSince ages past divHoldSecs
    % and the recovery block below re-localizes. (warmup is exempt - the EKF
    % is still pulling in then and naturally accepts fewer beams.)
    accHealthy = (it <= warmIters) || (nValid == 0) || ...
                 (nAcc >= accGoodCount && nAcc >= accGoodFrac*nValid);
    if accHealthy
        lowAccSince = inf;
    elseif isinf(lowAccSince)
        lowAccSince = t_now;
    end
    beamDiverged = ekfGuard && (t_now - lowAccSince) > divHoldSecs && ...
                   (t_now - lastRelocT) > relocCooldown;

    % --- SLAM: Bayesian map update (live occupancy from the EKF pose) ---
    if mod(it, mapUpdateEvery) == 0
        ldsub.Ranges = ranges(1:mapUpdateStride:end);
        ldsub.Angles = angles(1:mapUpdateStride:end);
        probMap = MapUpdate(probMap, [xr+offset_m, yr+offset_m, theta], ...
                            ldsub, mscale, l_occ, l_free, l0);
        probMap(occGrid) = max(probMap(occGrid), 0.70);   % prior walls can't be cancelled
    end

    % --- Collision reporting (LiDAR proximity, debounced) ---
    valid_r = ranges(isfinite(ranges) & ranges >= 0.12 & ranges <= max_range);
    near    = ~isempty(valid_r) && min(valid_r) < collisionDist;
    if near && ~wasNear && (t_now - lastCollT) > collisionCool
        collisions = collisions + 1;  lastCollT = t_now;
        fprintf('   [!] collision proximity #%d at t=%.1fs (d=%.2f m)\n', ...
                collisions, t_now, min(valid_r));
    end
    wasNear = near;

    % --- Waypoint capture (on the EKF estimate) ---
    target     = waypts(wp_i, :);
    dWp        = norm(target - [xr, yr]);
    is_last_wp = (wp_i == nWp);
    captureEps = distEpsGoal*is_last_wp + distEps*(~is_last_wp);
    if dWp <= captureEps
        if is_last_wp, goalReached = true; break; end
        wp_i = wp_i + 1;  IntegralError = 0;
        bestDWp = inf;  lastProgressT = t_now;   % reset watchdog for new wp
        continue;
    end

    % --- Recovery: re-localize + re-plan on divergence ------------------
    % Triggered by EITHER a sustained beam-acceptance crash (the EKF has lost
    % lock; primary trigger) OR, if enabled, the progress watchdog (the robot
    % stopped getting closer to its waypoint). Getting closer resets the
    % progress timer.
    if dWp < bestDWp - stuckImprove
        bestDWp = dWp;  lastProgressT = t_now;
    end
    stuckDiverged = recoverOnStuck && (t_now - lastProgressT) > stuckTimeout;
    if beamDiverged || stuckDiverged
        nRecover = nRecover + 1;
        if beamDiverged
            warning(['EKF divergence: beams crashed (%d/%d) for >%.0fs ' ...
                     '-> re-localizing & re-planning [recovery #%d].'], ...
                     nAcc, nValid, divHoldSecs, nRecover);
        else
            warning(['no progress for %.0fs (dEKF-odom=%.2fm) -> re-localizing ' ...
                     '& re-planning [recovery #%d].'], stuckTimeout, trueErr, nRecover);
        end
        tbot.setVelocity(0, 0);

        % fresh global fix (same retry logic as the initial localization)
        bestS = inf;  pNew = p;
        for a = 1:loc_retries
            [~, ld0, ~] = tbot.readLidar();
            [pt, st] = globalLocalize(ld0, occGrid, mscale, offset_m, ...
                                      max_range, sensor_offset);
            if st < bestS, pNew = pt; bestS = st; end
            if bestS <= loc_score_max, break; end
        end
        p = pNew(:);  Sigma = Sigma0;
        fprintf('   re-localized: est=[% .2f % .2f % .1fdeg] residual=%.3f m\n', ...
                p(1), p(2), rad2deg(p(3)), bestS);

        % re-plan A* from the recovered pose to the target
        [waypts, planOK] = planPath(p(1:2).', TARGET, planInfl, mscalePlan, ...
                                    offset_m, wpSpacing);
        if planOK
            nWp = size(waypts, 1);
        else
            warning('re-plan failed; steering VFH straight at the target.');
            waypts = TARGET;  nWp = 1;
        end
        wp_i = 1;  IntegralError = 0;  previousError = 0;
        bestDWp = inf;  lastProgressT = t_now;
        lowAccSince = inf;  lastRelocT = t_now;   % reset the beam-crash guard
        waitfor(rate);  continue;
    end

    % --- Build the VFH grid (coarse + robot-inflated) -------------------
    %   liveInflD : known walls OR Bayesian map OR current LiDAR, inflated.
    %               Classic VFH steers on this, so the walls repel too.
    if mod(it, liveGridEvery) == 0 || it == 1
        liveOcc = coarseOcc | downsampleMax(probMap > mapThresh, Hc, Wc);
        liveOcc = liveOcc | currentLidarCoarse(p, ranges, angles, ...
                            sensor_offset, mscalePlan, offset_m, Hc, Wc, max_range);
        liveInflD = double(imdilate(liveOcc, se_coarse));
    end

    % --- Adaptive waypoint skipping (blocked-only) ----------------------
    % If an obstacle now covers this (intermediate) waypoint in the live grid,
    % skip it; VFH then steers toward the next waypoint. The final goal
    % waypoint is never skipped.
    if ~is_last_wp && waypointBlocked(target, liveInflD, mscalePlan, offset_m, Hc, Wc)
        fprintf('   wp %d/%d blocked by a new obstacle -> skipping.\n', wp_i, nWp);
        wp_i = wp_i + 1;  IntegralError = 0;
        bestDWp = inf;  lastProgressT = t_now;   % reset watchdog for new wp
        continue;
    end

    % --- REACTIVITY: classic VFH on the full live grid ------------------
    % VFH steers on liveInflD (known walls + Bayesian map + current LiDAR), so
    % the walls repel like any obstacle. Reacts to unexpected objects, but can
    % struggle to dock at a waypoint/target that sits close to a wall.
    [steerDir, h_smooth, bin_edges] = VFH([xr, yr], target, liveInflD, ...
                                          mscalePlan, searchWindow);
    if isnan(steerDir)
        tbot.setVelocity(0, 0);     % no free valley: hold, re-sense next loop
        waitfor(rate);  continue;
    end

    % --- CONTROL: PI pure-pursuit along the VFH direction ---
    L = min(cStar, dWp);
    xStar = xr + L*cos(steerDir);
    yStar = yr + L*sin(steerDir);
    if dWp > dStar
        Error = sqrt((xStar-xr)^2 + (yStar-yr)^2) - dStar;
    else
        Error = dWp;
    end
    Error = max(0, Error);
    if Error > 0
        IntegralError = IntegralError + (Error + previousError)*dt/2;
    end
    previousError = Error;

    v = kv*Error + ki*IntegralError;
    v = max(-v_max, min(v, v_max));
    if is_last_wp, v = v * min(1, max(0, dWp/slow_radius)); end

    thetaStar = atan2(yStar-yr, xStar-xr);
    w = ks * normalizeAngle(thetaStar - theta);
    w = max(-w_max, min(w, w_max));

    tbot.setVelocity(v, w);

    % --- Live plot (throttled) ---
    if mod(it, plotEvery) == 0 || it == 1
        figure(1); clf; hold on; axis equal; grid on;
        plot(xObsPrior, yObsPrior, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 3);
        newOcc = (probMap > mapThresh) & ~occGrid;     % obstacles MapUpdate found
        [in_, jn_] = find(newOcc);
        if ~isempty(in_)
            plot((in_-1)/mscale - offset_m, (jn_-1)/mscale - offset_m, ...
                 '.', 'Color', [0.85 0.2 0.2], 'MarkerSize', 4);
        end
        plot(odomTraj(1:it,1), odomTraj(1:it,2), 'Color', [0.4 0.7 0.4], 'LineWidth', 1.0);
        plot(estTraj(1:it,1),  estTraj(1:it,2),  'b-', 'LineWidth', 1.3);
        plot(waypts(:,1), waypts(:,2), 'm.', 'MarkerSize', 8);
        plot(target(1), target(2), 'mx', 'MarkerSize', 10, 'LineWidth', 2);
        plot(TARGET(1), TARGET(2), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 9);
        drawTurtleBot(xr, yr, theta);
        drawErrorElipse(Sigma(1:2,1:2), p(1:2), 2, 'm', 1.0);
        quiver(xr, yr, 0.3*cos(steerDir), 0.3*sin(steerDir), 0, 'r', ...
               'LineWidth', 2, 'MaxHeadSize', 0.8);
        drawPolarHistogram(xr, yr, 0, h_smooth, bin_edges, searchWindow*0.6, 'r');
        axis([-0.2, H/mscale + 0.2, -0.2, W/mscale + 0.2]);
        xlabel('x [m]'); ylabel('y [m]');
        title(sprintf(['SLAM+A*+VFH | t=%.1fs  d(goal)=%.2fm  dEKF-odom=%.2fm  ' ...
                       'beams=%d/%d  coll=%d  rec=%d\n' ...
                       '(grey=map, red=new, green=odom, blue=EKF)'], ...
                      t_now, norm([xr yr]-TARGET), trueErr, nAcc, nValid, ...
                      collisions, nRecover));

        % --- Inset mini-map: live Bayesian occupancy (MapUpdate) ---
        axInset = axes('Position', [0.655 0.655 0.28 0.28]);
        imagesc(axInset, [0 H/mscale], [0 W/mscale], probMap');
        set(axInset, 'YDir', 'normal', 'CLim', [0 1], ...
                     'XTick', [], 'YTick', []);
        colormap(axInset, flipud(gray));     % occupied = dark
        axis(axInset, 'image');
        title(axInset, 'Bayesian map');

        drawnow limitrate;
    end

    waitfor(rate);
end

tbot.stop();

% ===== 7) Final status ==================================================
finalErr = norm(p(1:2).' - TARGET);
fprintf('\n----------------------------------------------\n');
if goalReached && finalErr <= hitTol
    fprintf('RESULT: TARGET REACHED\n');
elseif goalReached
    fprintf('RESULT: stopped at goal waypoint but %.3f m off (> %.2f m tol)\n', ...
            finalErr, hitTol);
else
    fprintf('RESULT: did NOT reach the target (time/safety cap)\n');
end
fprintf('time = %.1f s | collisions = %d | EKF dist to target = %.3f m\n', ...
        toc, collisions, finalErr);
fprintf('----------------------------------------------\n');
fprintf('Re-run this script (>> demoNavigateWallVFH) for the next attempt.\n\n');


% =========================================================================
% Local helper functions
% =========================================================================

function [waypts, ok] = planPath(start_world, goal_world, planInfl, mscalePlan, ...
                                 offset_m, wpSpacing)
% A* on the coarse inflated grid -> waypoints spaced ~wpSpacing metres apart.
    ok = false;  waypts = [];
    [Hp, Wp] = size(planInfl);
    w2gP = @(xy) [ max(1,min(Hp, round((xy(1)+offset_m)*mscalePlan)+1)), ...
                   max(1,min(Wp, round((xy(2)+offset_m)*mscalePlan)+1)) ];
    s_rc = w2gP(start_world);  g_rc = w2gP(goal_world);

    pg = planInfl;
    pg(s_rc(1), s_rc(2)) = false;       % free start/goal if inflation closed them
    if pg(g_rc(1), g_rc(2))
        warning('goal cell inside an inflated obstacle - clearing it for A*.');
        pg(g_rc(1), g_rc(2)) = false;
    end

    pathGrid = A_star(double(pg), s_rc, g_rc);
    if isempty(pathGrid), return; end

    pathWorld = [(pathGrid(:,1)-1)/mscalePlan - offset_m, ...
                 (pathGrid(:,2)-1)/mscalePlan - offset_m];
    stride = max(1, round(wpSpacing*mscalePlan));
    idx    = unique([1:stride:size(pathWorld,1), size(pathWorld,1)]);
    waypts = pathWorld(idx, :);
    if size(waypts,1) > 1 && norm(waypts(1,:) - start_world) < 0.10
        waypts(1,:) = [];               % drop first waypoint if it's the start
    end
    waypts(end,:) = goal_world(:).';    % snap last waypoint exactly to the goal
    ok = true;
end


function coarse = downsampleMax(fineLogical, Hc, Wc)
% Max-pool a fine logical grid down to [Hc x Wc] (any obstacle -> obstacle).
    [H, W] = size(fineLogical);
    fr = ceil(H/Hc);  fc = ceil(W/Wc);
    pad = false(Hc*fr, Wc*fc);
    pad(1:H, 1:W) = fineLogical;
    blk = reshape(pad, fr, Hc, fc, Wc);
    coarse = squeeze(any(any(blk,1),3));
end


function g = currentLidarCoarse(p, ranges, angles, sensor_offset, mscP, ...
                                offset_m, Hc, Wc, max_range)
% Rasterize the CURRENT LiDAR returns into a coarse logical grid (for
% immediate reactivity, complementing the slower Bayesian map).
    g = false(Hc, Wc);
    phi = p(3);
    xso = sensor_offset(1); yso = sensor_offset(2);
    xs = p(1) + xso*cos(phi) - yso*sin(phi);
    ys = p(2) + xso*sin(phi) + yso*cos(phi);
    valid = isfinite(ranges) & ranges >= 0.12 & ranges <= max_range;
    zi = ranges(valid);  ai = angles(valid);
    xw = xs + zi.*cos(phi+ai);
    yw = ys + zi.*sin(phi+ai);
    % VFH forward map: round(x*mscale), no +1 (requires offset_m == 0)
    rr = round((xw+offset_m)*mscP);
    cc = round((yw+offset_m)*mscP);
    in = rr>=1 & rr<=Hc & cc>=1 & cc<=Wc;
    if any(in), g(sub2ind([Hc Wc], rr(in), cc(in))) = true; end
end


function tf = waypointBlocked(wp, obsGrid, mscP, offset_m, Hc, Wc)
% True if the waypoint's coarse cell is occupied in the given inflated grid
% (same forward map VFH uses: round(x*mscale), no +1; needs offset_m == 0).
    r = max(1, min(Hc, round((wp(1)+offset_m)*mscP)));
    c = max(1, min(Wc, round((wp(2)+offset_m)*mscP)));
    tf = obsGrid(r, c) > 0.5;
end