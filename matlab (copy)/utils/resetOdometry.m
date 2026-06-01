function resetOdometry(tbot, start)

    tbot.setPose(start(1), start(2), start(3));
    
    % Wait until odometry is properly reset
    fprintf("Waiting for odometry reset...");
    
    while true
        [x, y, theta, timestamp] = tbot.readPose();
    
        if abs(start(1) - x) < 0.02 && abs(start(2) - y) < 0.02
            break;
        end
    
        pause(0.1);
    end

    fprintf("Odometry initialized.");
    
end