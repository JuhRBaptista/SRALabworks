function updatePathTrackingPlot(plotHandles, trajectory, robotPose, targetPose, histogram, alpha, Sigma, map)
    
    % Atualizar mapa
    if nargin >= 8 && ~isempty(map)
        set(plotHandles.mapImage, 'CData', map');
    end

    % Atualizar trajetória
    if ~isempty(trajectory)
        set(plotHandles.trajectory, ...
            'XData', trajectory(:,1), ...
            'YData', trajectory(:,2));
    end

    % Atualizar robô
    set(plotHandles.robot, ...
        'XData', robotPose.x, ...
        'YData', robotPose.y);

    % Atualizar target
    set(plotHandles.target, ...
        'XData', targetPose.x, ...
        'YData', targetPose.y);

    % Atualizar histograma VFH
    if nargin >= 5 && ~isempty(histogram)

        nSectors = length(histogram);

        % Normalizar (evita divisão por zero)
        histNorm = histogram / (max(histogram) + 1e-6) * 0.3;

        % Ângulos dos setores
        angles = (0:nSectors-1) * alpha - pi + alpha/2;

        % Coordenadas polares → cartesianas
        xHist = robotPose.x + histNorm .* cos(angles);
        yHist = robotPose.y + histNorm .* sin(angles);

        % Fechar o polígono
        xHist(end+1) = xHist(1);
        yHist(end+1) = yHist(1);

        set(plotHandles.histogram, ...
            'XData', xHist, ...
            'YData', yHist);
    end

    % ==============================
    % Covariance ellipse
    % ==============================
    if nargin >= 7 && ~isempty(Sigma)
    
        % Position covariance only
        SigmaXY = Sigma(1:2,1:2);
    
        % Eigen decomposition
        [V, D] = eig(SigmaXY);
    
        % Ellipse orientation
        angle = atan2(V(2,1), V(1,1));
    
        % Confidence scaling (95%)
        k = 2;
    
        % Ellipse axes lengths
        a = k * sqrt(D(1,1));
        b = k * sqrt(D(2,2));
    
        % Ellipse parametric points
        t = linspace(0, 2*pi, 50);
    
        ellipse = [a*cos(t); b*sin(t)];
    
        % Rotation matrix
        R = [
            cos(angle) -sin(angle);
            sin(angle)  cos(angle)
        ];
    
        ellipseRot = R * ellipse;
    
        % Translate to robot position
        xEllipse = ellipseRot(1,:) + robotPose.x;
        yEllipse = ellipseRot(2,:) + robotPose.y;
    
        % Update plot
        set(plotHandles.covariance, ...
            'XData', xEllipse, ...
            'YData', yEllipse);
    
    end

    % Atualizar gráfico
    drawnow limitrate;
end