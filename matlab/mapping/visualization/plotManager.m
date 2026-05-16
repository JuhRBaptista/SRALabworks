function ui = plotManager(map, params)

    ui.fig = figure('KeyPressFcn', @keyboardControl);

    ui.img = imagesc(map', [0 1]); 
    colormap(flipud(gray));
    axis equal;
    set(gca, 'YDir', 'normal');
    hold on;

    ui.robot = plot(0,0,'ro','MarkerSize',8,'LineWidth',2);
    ui.arrow = quiver(0,0,0,0,'r','LineWidth',2);

end
