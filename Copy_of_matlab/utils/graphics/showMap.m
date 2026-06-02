function showMap(map, titleStr)
    imagesc(map);
    colormap(flipud(gray));
    axis equal tight;
    set(gca, 'YDir', 'reverse');
    title(titleStr);
    hold on;
end