function updatePlot(plotHandles, trajectory, robotPose, targetPose, histogram, alpha, Sigma, map)
% updatePathTrackingPlot - Refreshes all active navigation plot elements each iteration.
%
% USAGE:
%   updatePathTrackingPlot(plotHandles, trajectory, robotPose, targetPose, ...
%                          histogram, alpha, Sigma, map)
%
% INPUTS:
%   plotHandles : struct returned by setupPathTrackingPlot.
%                 The field plotHandles.opt controls which elements are updated.
%
%   trajectory  : Nx2 matrix [x, y] of the robot's traveled path so far.
%                 Pass [] to skip.
%
%   robotPose   : struct with fields .x, .y (and optionally .theta).
%                 Represents the robot's current estimated pose.
%
%   targetPose  : struct with fields .x, .y.
%                 Current intermediate navigation target.
%                 Only used when plotHandles.opt.showTarget = true.
%
%   histogram   : 1xN vector of VFH sector magnitudes (already smoothed).
%                 Only used when plotHandles.opt.showVFH = true.
%                 Pass [] to skip.
%
%   alpha       : angular width of each VFH sector [rad].
%                 Required when histogram is non-empty.
%
%   Sigma       : 3x3 pose covariance matrix from the EKF.
%                 Only the top-left 2x2 block (x,y) is used for the ellipse.
%                 Only used when plotHandles.opt.showCovariance = true.
%                 Pass [] to skip.
%
%   map         : updated occupancy grid matrix (same size as at setup).
%                 Only used when plotHandles.opt.showMap = true.
%                 Pass [] to skip.
%
% NOTE:
%   All optional inputs beyond trajectory can be omitted or passed as [].
%   The function guards every update with both the opt flag AND an isfield
%   check, so it is safe to call even when a handle was never created.

    % Retrieve the option flags stored at setup time
    opt = plotHandles.opt;

    % ------------------------------------------------------------------
    % 1. Map background update (optional)
    %    Only runs if the map was shown at setup AND a new map is provided
    % ------------------------------------------------------------------
    if opt.showMap && ~isempty(plotHandles.mapImage) && nargin >= 8 && ~isempty(map)
        % Transpose: imagesc expects (row = y, col = x), our map is (row = x, col = y)
        set(plotHandles.mapImage, 'CData', map');
    end

    % ------------------------------------------------------------------
    % 2. Robot trajectory
    %    Replaces the full XData/YData each frame (simple and robust)
    % ------------------------------------------------------------------
    if opt.showTrajectory && isfield(plotHandles, 'trajectory') && ~isempty(trajectory)
        set(plotHandles.trajectory, ...
            'XData', trajectory(:,1), ...
            'YData', trajectory(:,2));
    end

    % ------------------------------------------------------------------
    % 3. Robot current position marker
    % ------------------------------------------------------------------
    if opt.showRobot && isfield(plotHandles, 'robot')
        set(plotHandles.robot, ...
            'XData', robotPose.x, ...
            'YData', robotPose.y);
    end

    % ------------------------------------------------------------------
    % 4. Current navigation target
    % ------------------------------------------------------------------
    if opt.showTarget && isfield(plotHandles, 'target') && nargin >= 4 && ~isempty(targetPose)
        set(plotHandles.target, ...
            'XData', targetPose.x, ...
            'YData', targetPose.y);
    end

    % ------------------------------------------------------------------
    % 5. VFH polar histogram overlay
    %    Converts sector magnitudes to a closed polygon in world coordinates
    % ------------------------------------------------------------------
    if opt.showVFH && isfield(plotHandles, 'histogram') && nargin >= 5 && ~isempty(histogram)

        nSectors = length(histogram);

        % Normalise to a fixed display radius (0.3 m) so the polygon
        % doesn't grow with absolute obstacle density
        histNorm = histogram / (max(histogram) + 1e-6) * 0.3;

        % Centre angle of each sector
        angles = (0:nSectors-1) * alpha - pi + alpha/2;

        % Polar -> Cartesian, centred on the robot
        xHist = robotPose.x + histNorm .* cos(angles);
        yHist = robotPose.y + histNorm .* sin(angles);

        % Close the polygon by repeating the first point
        xHist(end+1) = xHist(1);
        yHist(end+1) = yHist(1);

        set(plotHandles.histogram, 'XData', xHist, 'YData', yHist);
    end

    % ------------------------------------------------------------------
    % 6. EKF covariance ellipse (95% confidence)
    %    Uses eigen-decomposition of the 2x2 position covariance block
    % ------------------------------------------------------------------
    if opt.showCovariance && isfield(plotHandles, 'covariance') && nargin >= 7 && ~isempty(Sigma)

        % Extract position-only covariance (ignore heading uncertainty here)
        SigmaXY = Sigma(1:2, 1:2);

        % Eigen-decomposition: V = eigenvectors (axes), D = eigenvalues (variances)
        [V, D] = eig(SigmaXY);

        % Ellipse rotation angle from the principal eigenvector
        angle = atan2(V(2,1), V(1,1));

        % Scale factor for 95% confidence region (chi-squared with 2 DOF)
        k = 2;

        % Semi-axis lengths
        a = k * sqrt(abs(D(1,1)));
        b = k * sqrt(abs(D(2,2)));

        % Parametric ellipse points (unrotated, uncentred)
        t = linspace(0, 2*pi, 50);

        % Rotation matrix
        R = [cos(angle), -sin(angle); ...
             sin(angle),  cos(angle)];

        % Rotate and translate to robot position
        ell = R * [a * cos(t); b * sin(t)];

        set(plotHandles.covariance, ...
            'XData', ell(1,:) + robotPose.x, ...
            'YData', ell(2,:) + robotPose.y);
    end

    % ------------------------------------------------------------------
    % 7. Flush the graphics buffer
    %    limitrate prevents the plot from becoming the loop bottleneck
    % ------------------------------------------------------------------
    drawnow limitrate;

end