function PathTrackingControl(tbot, params, path, handles, avoidance)
    % PathTrackingControl - Tracks a path using waypoint navigation.
    %
    % Required params:
    %   path          : Nx2 matrix of waypoints [x y]
    %   kv, ki        : distance control gains
    %   ks            : heading control gain
    %   distance      : desired tracking distance
    %   avoidance     : "none", "vff", or "vfh"
    %   vMax          : maximum linear velocity
    %   wMax          : maximum angular velocity
    %   rate          : frequency of iteration
    %   dt            : timeskip for integrating
    %   T             : maximun time
    %   map           : environment map
    %   scale         : map scale to real world
    %   origin        : map origin
    %   toleranceError: minimum distance necessary for stopping
   
    % Default obstacle avoidance mode
    if nargin < 5
        avoidance="none";
    end

    target_index = 1;
    e_int = 0;

    % Number of waypoints
    N = size(path, 1); 
    
    traj = [];
    
    r = rateControl(params.rate);  
    
    for t= 0:params.dt:params.T
        
        % Get current robot pose
        [pose.x, pose.y, pose.theta, ~] = tbot.readPose();
        pose.theta = normalizeAngle(pose.theta);
        
        % Store trajectory
        traj = [traj; [pose.x, pose.y]];
            
        % Current waypoint
        waypoint.x = path(target_index, 1);
        waypoint.y = path(target_index, 2);

        % Compute intermediate target
        [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance);

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

        waitfor(r);
    end
    
    % Stop robot
    tbot.setVelocity(0, 0);
end

function [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance)
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
            target = computeTargetVFF(pose, waypoint, params);
        case "vfh"
            [target, h, alpha] = computeTargetVFH(pose, waypoint, params);
        otherwise
            target = waypoint;
    end
end

function target = computeTargetVFF(pose, waypoint, params)
    % computeTargetVFF - Computes a target using VFF obstacle avoidance.
    %
    % Required params:
    %   map, scale, origin : occupancy map parameters
    
    % Attractive and repulsive forces
    [Fa, Fr] = VFF([pose.x, pose.y], [waypoint.x, waypoint.y], ...
                   params.map, 10, params.scale, params.origin);

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


function [target, h, alpha] = computeTargetVFH(pose, waypoint, params)
    % computeTargetVFH - Computes a target using VFH obstacle avoidance.
    %
    % Outputs:
    %   target : intermediate target position
    %   h      : polar histogram
    %   alpha  : steering sectors

    % Compute steering direction
    [steerAngle, h, alpha] = VFH([pose.x, pose.y, pose.theta], [waypoint.x, waypoint.y], ...
                     params.map, 10);
    
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