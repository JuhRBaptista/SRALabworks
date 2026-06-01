function worldPts = scanToWorld(scan, pose)
    
    x = scan(:,1);
    y = scan(:,2);

    c = cos(pose.theta);
    s = sin(pose.theta);

    wx = pose.x + x*c - y*s;
    wy = pose.y + x*s + y*c;

    worldPts = [wx, wy];

end