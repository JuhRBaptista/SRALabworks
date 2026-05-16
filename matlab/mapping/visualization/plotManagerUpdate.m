function plotManagerUpdate(ui, map, robotGrid, theta)
    arrowLen = 10;
    set(ui.img, 'CData', map');
    set(ui.robot, 'XData', robotGrid(1), 'YData', robotGrid(2));
    set(ui.arrow, ...
        'XData', robotGrid(1), 'YData', robotGrid(2), ...
        'UData', arrowLen * cos(theta), ...
        'VData', arrowLen * sin(theta));
    drawnow limitrate;
end