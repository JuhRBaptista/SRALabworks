function PoseToPoseControl(tbot, params, handles)
    % PoseToPoseControl - Drives the robot to a target pose.
    %
    % Required params:
    %   target             : [x, y, theta] target pose
    %   kpRho              : linear gain
    %   kpAlpha            : heading gain
    %   kpBeta             : final orientation gain
    %   vMax               : maximum linear velocity
    %   wMax               : maximum angular velocity
    %   rate               : frequency of iteration
    %   maxIterations      : maximun number of iterations
    %   toleranceErrorDist : minimum distance necessary for stopping
    %   toleranceErrorAngle: minimum angle necessary for stopping

    r = rateControl(params.rate);
    traj = [];
    
    % Target pose
    target.x = params.target(1);
    target.y = params.target(2);
    target.theta = params.target(3);

    for it=1:params.maxIterations

        % Get current robot pose
        [pose.x, pose.y, pose.theta, pose.timestamp] = tbot.readPose();
        pose.theta = normalizeAngle(pose.theta);
        
        % Final orientation error
        deltaTheta = normalizeAngle(target.theta - pose.theta);
        
        % Store trajectory
        trajectory(it,:) = [pose.x, pose.y, pose.theta];

        % Heading error relative to the target
        alpha = getAngularError(pose, target);
        
        % Final orientation alignment error
        beta = target.theta - pose.theta - alpha;         
        beta = normalizeAngle(beta);

        % Distance to target position
        rho = getEuclidianDistance(pose, target);       

        % Stop if target pose is reached
        if ((rho < params.toleranceErrorDist) && (abs(deltaTheta) < params.toleranceErrorAngle)) 
            tbot.stop(); 
            break; 
        end
        
        % Pose stabilization control law
        linearVelocity = params.kpRho*rho;
        angularVelocity = params.kpAlpha*alpha + params.kpBeta*beta;

        % Velocity saturation
        linearVelocity = max(min(linearVelocity, params.vMax), 0);
        angularVelocity = max(min(angularVelocity, params.wMax), -params.wMax);
        
        % Send velocity command
        tbot.setVelocity(linearVelocity, angularVelocity);

        % Store trajectory
        traj = [traj; [pose.x, pose.y, pose.theta]];
        updatePlot(handles, traj, pose, target, [], 0, [], []);

        waitfor(r);    
    
    end
end