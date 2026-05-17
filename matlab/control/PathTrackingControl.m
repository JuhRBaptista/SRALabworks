function PathTrackingControl(tbot, params, path, handles, avoidance, mapParams, avoidParams, slam, savePath)
    % PathTrackingControl - Tracks a path using waypoint navigation.
    
    % params:
    %   kv, ki        : distance control gains
    %   ks            : heading control gain
    %   distance      : desired tracking distance
    %   vMax          : maximum linear velocity
    %   wMax          : maximum angular velocity
    %   rate          : frequency of iteration
    %   dt            : timeskip for integrating
    %   T             : maximun time
    %   toleranceError: minimum distance necessary for stopping

    % mapParams
    %   map           : environment map
    %   scale         : map scale to real world
    %   origin        : map origin

    % avoidParameters (if avoidance is VFF or VFH)
    %  check VFF/VFH header 
   
    % Default obstacle avoidance mode
    if nargin < 5
        avoidance="none";
        mapParams = [];
        mapParams.map = [];
        avoidParams = [];
    end
    
    if nargin < 8
        slam = false;
    end

    if slam 
        map = initMap(mapParams);
        mapParams.map = mapParams.initialProb;  % Add this before the for loop
    end

    target_index = 1;
    e_int = 0;

    % Number of waypoints
    N = size(path, 1); 
    
    traj = [];
    
    r = rateControl(params.rate);  

    % Matriz de covariancia em relação a posição
    Cp = diag([0, 0, 0]);

    tbot.initEncoders();
    [p(1), p(2), p(3), ~] = tbot.readPose();
    p(3) = normalizeAngle(p(3));

        
    for t= 0:params.dt:params.T
        [~, data]  = tbot.readLidar();
        
        % Get current robot pose
        % [pose.x, pose.y, pose.theta, ~] = tbot.readPose();
        % pose.theta = normalizeAngle(pose.theta);
        
        % Update the robot's pose estimate using the EKF
        [dsr, dsl, ~, ~] = tbot.readEncoders();
        

        [p, Cp] = EKF(dsr, dsl, p, Cp, data, mapParams);

        pose.x = p(1);
        pose.y = p(2);
        pose.theta = p(3);

        % Store trajectory
        traj = [traj; [pose.x, pose.y]];
            
        % Current waypoint
        waypoint.x = path(target_index, 1);
        waypoint.y = path(target_index, 2);
        
        if slam
            idx_occ    = tbot.getInRangeLidarDataIdx(data);
            idx_free   = tbot.getOutRangeLidarDataIdx(data);
            [map, ~]   = mapUpdate(map, data, idx_occ, idx_free, pose, mapParams);
            mapParams.map = map.prob;
        end

        % Compute intermediate target
        [target, h, alpha] = computeTarget(pose, waypoint, avoidance, mapParams, avoidParams);

        % Tracking errors
        distance = getEuclidianDistance(pose, waypoint);
        e        = getEuclidianDistance(pose, target) - params.distance;
        phi      = getAngularError(pose, target);

        % PI distance controller + heading controller
        e_int            = e_int + e * params.dt;
        linearVelocity  = params.kv * e + params.ki * e_int;
        angularVelocity = params.ks * phi;

        % Velocity saturation
        linearVelocity = max(min(linearVelocity, params.vMax), 0);
        angularVelocity = max(min(angularVelocity, params.wMax), -params.wMax);
        
        % Send velocity command
        tbot.setVelocity(linearVelocity, angularVelocity);

        % Advance to next waypoint
        if distance < params.toleranceError && target_index < N
            target_index = target_index + 1;
            e_int = 0;
        end
        
         % Stop when final waypoint is reached
        if target_index >= N && distance < params.toleranceError
            break;
        end
        
        updatePlot(handles, traj, pose, target, h, alpha, Cp,  mapParams.map);


        waitfor(r);
    end
    
    % Stop robot
    tbot.setVelocity(0, 0);

    if slam
        saveResults(map.prob, savePath);
    end
end

function [target, h, alpha] = computeTarget(pose, waypoint, avoidance, mapParams, avoidParams)
    % computeTarget - Computes the navigation target.
    %
    % avoidance:
    %   "none" : direct waypoint following
    %   "vff"  : Vector Field Force
    %   "vfh"  : Vector Field Histogram

    
    h = 0;
    alpha= 0;

    switch avoidance
        case "vff"
            target = computeTargetVFF(pose, waypoint, mapParams, avoidParams);
        case "vfh"
             alpha = avoidParams.sectorWidth;
            [target, h] = computeTargetVFH(pose, waypoint, mapParams, avoidParams);
        otherwise
            target = waypoint;
    end
end

function target = computeTargetVFF(pose, waypoint, mapParams, avoidParams)
    % computeTargetVFF - Computes a target using VFF obstacle avoidance.
    
    % Attractive and repulsive forces
    [Fa, Fr] = VFF([pose.x, pose.y], [waypoint.x, waypoint.y], mapParams, avoidParams);

    % Resultant force
    F  = Fa + Fr;
    Fn = norm(F);

    % Keep current position if force is negligible
    if Fn < 1e-6
        target = pose;
    else
        % Normalized motion direction
        dir_hat  = F / Fn;

        % Distance to waypoint
        dist    = getEuclidianDistance(pose, waypoint);

        % Step size
        L        = min(1, dist);

        % Intermediate target
        target.x = pose.x + L * dir_hat(1);
        target.y = pose.y + L * dir_hat(2);
    end
end

function [target, h] = computeTargetVFH(pose, waypoint, mapParams, avoidParams)
    % computeTargetVFH - Computes a target using VFH obstacle avoidance.

    % Compute steering direction
    [steerAngle, h] = VFH([pose.x, pose.y, pose.theta], [waypoint.x, waypoint.y], mapParams, avoidParams);
    
    % Stop if no valid direction exists
    if isnan(steerAngle)
        target = pose;
        return;
    end
    
    % Distance to waypoint
    dist     = getEuclidianDistance(pose, waypoint);

    % Step size
    L        = min(1, dist);

    % Intermediate target
    target.x = pose.x + L * cos(steerAngle);
    target.y = pose.y + L * sin(steerAngle);
end

function map = initMap(mapParams)
    % Builds the initial map structure from mapParams.
    map.prob    = mapParams.initialProb;
    map.logOdds = mapParams.initialLogOdds;
end

function saveResults(mapProb, savePath)
    % Exports the occupancy grid as both a PNG image and .mat file.
    imgData = uint8(flipud(1 - mapProb) * 255);
    imwrite(imgData, [savePath, '.png']);
    save([savePath, '.mat'], 'mapProb');
end