function [deltaTheta] = getAngularError(currentPose, target)

    deltaX = target.x - currentPose.x;
    deltaY = target.y - currentPose.y;

    deltaTheta = atan2(deltaY,deltaX) - currentPose.theta;
    deltaTheta = normalizeAngle(deltaTheta);

end