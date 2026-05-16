function plotHandles = setupPathTrackingPlot(map, pathWorld, origin, scale)
    
    figure; hold on;

    % Converter mapa para mundo
    [nRows, nCols] = size(map);

    xLimits = [origin/scale, (nCols-1+origin)/scale];
    yLimits = [origin/scale, (nRows-1+origin)/scale];

    % Mostrar mapa (APENAS UMA VEZ)
    plotHandles.mapImage = imagesc(xLimits, yLimits, map');
    colormap(flipud(gray));
    set(gca, 'YDir', 'normal');

    % Elementos do gráfico

    % Caminho desejado
    plotHandles.path = plot(pathWorld(:,1), pathWorld(:,2), ...
        'r--', 'LineWidth', 2);

    % Trajetória do robô
    plotHandles.trajectory = plot(NaN, NaN, ...
        'b', 'LineWidth', 2);

    % Posição atual do robô
    plotHandles.robot = plot(NaN, NaN, ...
        'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

    % Covariance ellipse
    plotHandles.covariance = plot(NaN, NaN, ...
        'c-', 'LineWidth', 2);

    % Target atual
    plotHandles.target = plot(NaN, NaN, ...
        'go', 'MarkerSize', 10, 'LineWidth', 2);

    % Histograma VFH
    plotHandles.histogram = plot(NaN, NaN, ...
        'm-', 'LineWidth', 1.5);

    % Ajustes visuais
    axis equal;
    grid on;

    legend('Path', 'Trajectory', 'Robot', 'Target', 'VFH Histogram');

    xlabel('X (m)');
    ylabel('Y (m)');
    title('Robot Navigation (Real-time)');
end