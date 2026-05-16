function PointToPointControl(tbot, params)
    % PointToPointControl - Drives the robot to a target position.
    %
    % Required params:
    %   target        : [x, y] target position
    %   kV            : linear velocity gain
    %   kW            : angular velocity gain
    %   vMax          : maximum linear velocity
    %   wMax          : maximum angular velocity
    %   rate          : frequency of iteration
    %   maxIterations : maximun number of iterations
    %   toleranceError: minimum distance necessary for stopping

    r = rateControl(params.rate);
    trajectory = zeros(params.maxIterations, 3);

    % Target position
    target.x = params.target(1);
    target.y = params.target(2);

    for it=1:params.maxIterations
        
        % Get current robot pose
        [pose.x, pose.y, pose.theta, pose.timestamp] = tbot.readPose();
        pose.theta = normalizeAngle(pose.theta);

        % Store trajectory
        trajectory(it,:) = [pose.x, pose.y, pose.theta];

        % Heading error relative to the target
        deltaTheta = getAngularError(pose, target);
        
        % Distance to target position
        distance = getEuclidianDistance(pose, target); 
    
        % Stop if target position is reached
        if distance < params.toleranceError
           tbot.stop(); 
           break; 
        end
    
        % Proportional controller
        linearVelocity = params.kV*distance;
        angularVelocity = params.kW*deltaTheta;

        % Velocity saturation
        linearVelocity = max(min(linearVelocity, params.vMax), 0);
        angularVelocity = max(min(angularVelocity, params.wMax), -params.wMax);
        
        % Send velocity command
        tbot.setVelocity(linearVelocity, angularVelocity);

        waitfor(r);    
    
    end
end