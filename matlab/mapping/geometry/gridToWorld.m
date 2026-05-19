function worldPts = gridToWorld(gridPts, params)

    x = round((gridPts(:,1) - params.origin)/params.scale);
    y = round((gridPts(:,2) - params.origin)/params.scale);

    worldPts = [x, y];

end