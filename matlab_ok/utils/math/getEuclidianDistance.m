function [distance] = getEuclidianDistance(currentPose, target)
    
    deltaX = target.x - currentPose.x;
    deltaY = target.y - currentPose.y;
    
    distance = sqrt(deltaX^2 + deltaY^2);
end