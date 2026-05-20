function [p, path] = getRoute(tbot, target, mapParams)
    
    p  = globalLocalize(tbot, mapParams);
    p(3) = normalizeAngle(p(3));

    x = round((p(1) * mapParams.scale) + mapParams.origin);
    y = round((p(2) * mapParams.scale) + mapParams.origin);
    gridPose = [x, y];

    x = round((target(1) * mapParams.scale) + mapParams.origin);
    y = round((target(2) * mapParams.scale) + mapParams.origin);
    gridTarget = [x, y];

    gridPath = aStar(mapParams.map, gridPose, gridTarget);

    path = gridToWorld(gridPath, mapParams);
end