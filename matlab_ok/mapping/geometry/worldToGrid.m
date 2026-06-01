function gridPts = worldToGrid(worldPts, params)

    x = round((worldPts(:,1) * params.scale) + params.origin);
    y = round((worldPts(:,2) * params.scale) + params.origin);

    gridPts = [x, y];

end