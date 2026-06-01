function PathTrackingControl1(tbot, params, path, handles, avoidance, mapParams, avoidParams, slam, savePath)
% PathTrackingControl - Tracks a path using waypoint navigation.
%
% KEY FIXES vs. the previous version
%   1. When EKF is active and estimatePose=true, the initial pose and
%      covariance are read from params.p0 / params.Sigma0 (set by main.m
%      after globalLocalize) instead of repeating globalLocalize here.
%   2. params.ekfIter is incremented each loop iteration and forwarded
%      to mapParams so EKF.m can apply the warm gate during early steps.

    if nargin < 5
        avoidance = "none";
        mapParams  = [];  mapParams.map = [];
        avoidParams = [];
    end
    if nargin < 8, slam = false; end
    
    originalPts = path;
    useEKF = isfield(params, 'ekf') && params.ekf;

    if slam
        map = initMap(mapParams);
        mapParams.map = mapParams.initialProb;
    end

    target_index = 1;
    e_int        = 0;
    N            = size(path, 1);
    traj         = [];
    gtTraj       = [];
    r            = rateControl(params.rate);
    Cp           = diag(zeros(1, 3));
    ekfIter      = 0;    % iteration counter forwarded to EKF warm gate

    if useEKF
        tbot.initEncoders();

        if params.estimatePose
            % Use the pose already computed by globalLocalize in main.m
            if isfield(params, 'p0') && ~isempty(params.p0)
                p  = params.p0(:);
                Cp = params.Sigma0;
                % Rebuild A* path from the localised start
                if isfield(params, 'p0') && ~isempty(params.p0)
                    p = params.p0(:);
                    Cp = params.Sigma0;
                    [p, path] = getRoute(tbot, originalPts, mapParams, p);  % passa p0 para skip da localização
                else
                    [p, path] = getRoute(tbot, originalPts, mapParams);
                end

                N  = size(path, 1);
            else
                % Fallback (old behaviour): run getRoute which calls
                % globalLocalize internally
                [p, path] = getRoute(tbot, path, mapParams);
                N  = size(path, 1);
                Cp = diag([0.1, 0.1, 0.1]);
            end
        else
            [px, py, pth, ~] = tbot.readPose();
            p  = [px; py; normalizeAngle(pth)];
            Cp = diag([0.01, 0.01, 0.001]);
        end

        gzSub     = rossubscriber('/gazebo/model_states');
        robotName = 'turtlebot3';
    end

        % --- fora do loop, na inicialização ---
    relocCooldown    = 0;   % contador de iterações com sigma alto
    relocMinIter     = 10;  % só relocaliza se sigma alto durante 50 iterações consecutivas (~2.5s a 20Hz)
    relocBlocked     = 0;   % cooldown após relocalização para não disparar logo de novo
    relocBlockFrames = 25; % bloqueia relocalizações durante 100 iter após cada uma
    
    for t = 0 : params.dt : params.T
        [~, data] = tbot.readLidar();

        if useEKF 
            ekfIter = ekfIter + 1;
            mapParams.ekfIter = ekfIter;   % passed into EKF -> warm gate

            noise_std    = 0.002;
            [dsr, dsl, ~, ~] = tbot.readEncodersWithNoise(noise_std);
            
            [p, Cp]      = EKF(dsr, dsl, p, Cp, data, mapParams);

            pose.x     = p(1);
            pose.y     = p(2);
            pose.theta = p(3);

            [gt_x, gt_y, gt_theta] = readGroundTruth(gzSub, robotName);
            gtTraj = [gtTraj; gt_x, gt_y, gt_theta];
        else
            [pose.x, pose.y, pose.theta, ~] = tbot.readPose();
            pose.theta = normalizeAngle(pose.theta);
        end

        % --- Relocalização automática se a covariância se mantiver grande ---
        sigma_pos = sqrt(Cp(1,1) + Cp(2,2));
        sigma_th  = sqrt(Cp(3,3));
        
        if relocBlocked > 0
            relocBlocked = relocBlocked - 1;   % ainda em cooldown, não faz nada
        elseif sigma_pos > 0.01 || sigma_th > deg2rad(30)
            % ===== Global localisation with retry & quality check ====================
            sensor_offset = [-0.0305; 0];   % LiDAR ~3 cm behind wheel axis
            loc_retries   = 4;
            loc_score_max = 0.12;           % acceptable mean range error [m]

            relocCooldown = relocCooldown + 1;
        
            if relocCooldown >= relocMinIter
                fprintf('[RELOC] Sigma alto há %d iterações (sigma_pos=%.2fm, sigma_th=%.1fdeg) - a relocalizar...\n', ...
                        relocCooldown, sigma_pos, rad2deg(sigma_th));
        
                tbot.setVelocity(0, 0);
        
                bestResidual = inf;  pBest = [];
                for attempt = 1:loc_retries
                    [~, lddata0, ~] = tbot.readLidar();
                    [p_try, s_try]  = globalLocalize(lddata0, mapParams.map, mapParams.scale, ...
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

        
                p  = pBest;
                pose.x     = p(1);
                pose.y     = p(2);
                pose.theta = p(3);
                conf   = min(1, bestResidual / loc_score_max);
                Cp = diag([0.20, 0.20, deg2rad(12)].^2) .* (1 + 4*conf);
                
                % Store the EKF state in params so PathTrackingControl can use it.
                % The ekfIter field lets EKF.m know when to apply the warm gate.
                params.Sigma0 = Cp;
        
                % waypointsLeft = originalPts(target_index:end, :);
                [p, path]     = getRoute(tbot, originalPts, mapParams, p);
                N             = size(path, 1);
                target_index  = 1;
                e_int         = 0;
                ekfIter       = 0;
        
                fprintf('[RELOC] Rota retraçada com %d waypoints.\n', N);
        
                relocCooldown = 0;              % reinicia contador
                relocBlocked  = relocBlockFrames;  % bloqueia durante X iterações
                pause(2);
            end
        else
            relocCooldown = 0;   % sigma voltou a ser aceitável, reinicia contador
        end

        traj = [traj; pose.x, pose.y];

        waypoint.x = path(target_index, 1);
        waypoint.y = path(target_index, 2);

        if slam
            idx_occ  = tbot.getInRangeLidarDataIdx(data);
            idx_free = tbot.getOutRangeLidarDataIdx(data);
            [map, ~] = mapUpdate(map, data, idx_occ, idx_free, pose, mapParams);
            mapParams.map = map.prob;
        end

        [target, h, alpha] = computeTarget(pose, waypoint, avoidance, mapParams, avoidParams);

        distance = getEuclidianDistance(pose, waypoint);
        e        = getEuclidianDistance(pose, target) - params.distance;
        phi      = getAngularError(pose, target);

        e_int           = e_int + e * params.dt;
        linearVelocity  = params.kv * e + params.ki * e_int;
        angularVelocity = params.ks * phi;

        linearVelocity  = max(min(linearVelocity,  params.vMax), 0);
        angularVelocity = max(min(angularVelocity,  params.wMax), -params.wMax);

        tbot.setVelocity(linearVelocity, angularVelocity);

        if distance < params.toleranceError && target_index < N
            target_index = target_index + 1;
            e_int = 0;
        end

        if target_index >= N && distance < params.finalToleranceError
            break;
        end

        updatePlot(handles, traj, gtTraj, pose, target, path, h, alpha, Cp, mapParams.map);
        waitfor(r);
    end

    tbot.setVelocity(0, 0);
    save("../data/noCorrected_trajs", "traj", "gtTraj");

    if slam
        saveResults(map.prob, savePath);
    end
end

% -------------------------------------------------------------------------

function [target, h, alpha] = computeTarget(pose, waypoint, avoidance, mapParams, avoidParams)
    h = 0; alpha = 0;
    switch avoidance
        case "vff"
            target = computeTargetVFF(pose, waypoint, mapParams, avoidParams);
        case "vfh"
            alpha  = avoidParams.sectorWidth;
            [target, h] = computeTargetVFH(pose, waypoint, mapParams, avoidParams);
        otherwise
            target = waypoint;
    end
end

function target = computeTargetVFF(pose, waypoint, mapParams, avoidParams)
    [Fa, Fr] = VFF([pose.x, pose.y], [waypoint.x, waypoint.y], mapParams, avoidParams);
    F  = Fa + Fr;  Fn = norm(F);
    if Fn < 1e-6
        target = pose;
    else
        dir_hat  = F / Fn;
        dist     = getEuclidianDistance(pose, waypoint);
        L        = min(1, dist);
        target.x = pose.x + L * dir_hat(1);
        target.y = pose.y + L * dir_hat(2);
    end
end

function [target, h] = computeTargetVFH(pose, waypoint, mapParams, avoidParams)
    [steerAngle, h] = VFH([pose.x, pose.y, pose.theta], [waypoint.x, waypoint.y], ...
                          mapParams, avoidParams);
    if isnan(steerAngle)
        target = pose; return;
    end
    dist     = getEuclidianDistance(pose, waypoint);
    L        = min(1, dist);
    target.x = pose.x + L * cos(steerAngle);
    target.y = pose.y + L * sin(steerAngle);
end

function map = initMap(mapParams)
    map.prob    = mapParams.initialProb;
    map.logOdds = mapParams.initialLogOdds;
end

function saveResults(mapProb, savePath)
    imgData = uint8(flipud(1 - mapProb) * 255);
    imwrite(imgData, [savePath, '.png']);
    save([savePath, '.mat'], 'mapProb');
end