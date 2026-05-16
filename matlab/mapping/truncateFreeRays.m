function freePts = truncateFreeRays(data, idx, maxRange)
    angles = data.Angles(idx);
    freePts = [maxRange * cos(angles), maxRange * sin(angles)];
end