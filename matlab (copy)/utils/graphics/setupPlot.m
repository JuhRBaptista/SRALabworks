function plotHandles = setupPlot(map, pathWorld, origin, scale, options)
% setupPathTrackingPlot - Creates a real-time navigation plot with configurable elements.
%
% USAGE:
%   handles = setupPathTrackingPlot(map, pathWorld, origin, scale, options)
%
% INPUTS:
%   map        : occupancy grid matrix (used only if options.showMap = true)
%   pathWorld  : Nx2 matrix of [x, y] waypoints in world coordinates
%   origin     : map origin offset (in grid cells)
%   scale      : grid cells per meter (e.g. 20 means 1m = 20 cells)
%   options    : struct with optional flags (all default to false except noted):
%
%       options.showMap        - display the occupancy grid as background  (default: true)
%       options.showPath       - plot the reference path as a dashed line  (default: true)
%       options.showTrajectory - plot the robot's traveled trajectory      (default: true)
%       options.showRobot      - plot the robot's current position         (default: true)
%       options.showTarget     - plot the current navigation target        (default: false)
%       options.showCovariance - plot the EKF covariance ellipse           (default: false)
%       options.showVFH        - plot the VFH polar histogram              (default: false)
%
% OUTPUT:
%   plotHandles : struct with handles to each active plot element,
%                 plus plotHandles.opt storing the resolved options.
%
% EXAMPLES:
%   % Minimal (point-to-point, no map):
%   opts.showMap  = false;
%   opts.showPath = false;
%   handles = setupPathTrackingPlot([], [], 0, 20, opts);
%
%   % Full EKF + VFH:
%   opts.showMap        = true;
%   opts.showTarget     = true;
%   opts.showCovariance = true;
%   opts.showVFH        = true;
%   handles = setupPathTrackingPlot(map, path, origin, scale, opts);

    % ------------------------------------------------------------------
    % 1. Resolve options (fill in any missing fields with defaults)
    % ------------------------------------------------------------------
    if nargin < 5
        options = struct();
    end
    opt = parseOptions(options);

    % ------------------------------------------------------------------
    % 2. Create figure
    % ------------------------------------------------------------------
    figure;
    hold on;
    axis equal;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Robot Navigation');

    % Accumulate legend labels alongside their handles
    legendHandles = [];
    legendLabels  = {};

    % ------------------------------------------------------------------
    % 3. Map background (optional)
    %    Converts grid indices to world coordinates for correct axis scaling
    % ------------------------------------------------------------------
    if opt.showMap && ~isempty(map)
        [nRows, nCols] = size(map);

        % Compute world-coordinate limits from grid size, scale and origin
        xLimits = [(0     - origin) / scale, ...
                   (nCols - 1 + (0 - origin)) / scale];
        yLimits = [(0     - origin) / scale, ...
                   (nRows - 1 + (0 - origin)) / scale];

        % Display map; store handle so it can be updated each iteration
        plotHandles.mapImage = imagesc(xLimits, yLimits, map');
        colormap(flipud(gray));         % white = free, black = occupied
        set(gca, 'YDir', 'normal');     % y-axis pointing up
    else
        % Placeholder so updatePathTrackingPlot can safely check the field
        plotHandles.mapImage = [];
    end

    % ------------------------------------------------------------------
    % 4. Reference path (optional)
    %    Drawn once at setup; not updated during the loop
    % ------------------------------------------------------------------
    if opt.showPath

        % Create empty path handle
        h = plot(NaN, NaN, ...
                 'r--', 'LineWidth', 2);
    
        plotHandles.path = h;
    
        % If an initial path already exists, draw it
        if ~isempty(pathWorld)
    
            set(plotHandles.path, ...
                'XData', pathWorld(:,1), ...
                'YData', pathWorld(:,2));
    
        end
    
        legendHandles(end+1) = h;
        legendLabels{end+1}  = 'Path';
    
    end

    % ------------------------------------------------------------------
    % 5. Robot trajectory (optional)
    %    Starts empty; XData/YData are appended each iteration
    % ------------------------------------------------------------------
    if opt.showTrajectory
        h = plot(NaN, NaN, 'b', 'LineWidth', 2);
        plotHandles.trajectory = h;
        legendHandles(end+1)   = h;
        legendLabels{end+1}    = 'Trajectory';
    end

    % ------------------------------------------------------------------
    % 6. Robot current position marker (optional)
    % ------------------------------------------------------------------
    if opt.showRobot
        h = plot(NaN, NaN, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        plotHandles.robot    = h;
        legendHandles(end+1) = h;
        legendLabels{end+1}  = 'Robot';
    end

    % ------------------------------------------------------------------
    % 7. Current navigation target (optional)
    %    Useful for path-tracking and avoidance scenarios
    % ------------------------------------------------------------------
    if opt.showTarget
        h = plot(NaN, NaN, 'go', 'MarkerSize', 10, 'LineWidth', 2);
        plotHandles.target   = h;
        legendHandles(end+1) = h;
        legendLabels{end+1}  = 'Target';
    end

    % ------------------------------------------------------------------
    % 8. EKF covariance ellipse (optional)
    %    Shows the 95% confidence region of the pose estimate
    % ------------------------------------------------------------------
    if opt.showCovariance
        h = plot(NaN, NaN, 'c-', 'LineWidth', 2);
        plotHandles.covariance = h;
        legendHandles(end+1)   = h;
        legendLabels{end+1}    = 'Covariance (95%)';
    end

    % ------------------------------------------------------------------
    % 9. VFH polar histogram overlay (optional)
    %    Drawn as a closed polar polygon centered on the robot
    % ------------------------------------------------------------------
    if opt.showVFH
        h = plot(NaN, NaN, 'm-', 'LineWidth', 1.5);
        plotHandles.histogram = h;
        legendHandles(end+1)  = h;
        legendLabels{end+1}   = 'VFH Histogram';
    end
     
    % ------------------------------------------------------------------
    % 6. Ground truth trajectory (second line, EKF mode only)
    % ------------------------------------------------------------------
     if opt.showGroundTruth
        h = plot(NaN, NaN, 'g', 'LineWidth', 2);
        plotHandles.groundTruth = h;
        legendHandles(end+1)    = h;
        legendLabels{end+1}     = 'Ground Truth';
     end

    % ------------------------------------------------------------------
    % 11. Build legend from only the active elements
    % ------------------------------------------------------------------
    if ~isempty(legendHandles)
        legend(legendHandles, legendLabels{:});
    end

    % ------------------------------------------------------------------
    % 11. Store resolved options inside the handle struct so
    %     updatePathTrackingPlot knows which elements exist
    % ------------------------------------------------------------------
    plotHandles.opt = opt;

end


% ======================================================================
% HELPER: parseOptions
%   Fills missing fields with their defaults so callers only need to
%   specify the flags they actually want to change.
% ======================================================================
function opt = parseOptions(options)
    opt.showMap        = getOpt(options, 'showMap',        true);
    opt.showPath       = getOpt(options, 'showPath',       true);
    opt.showTrajectory = getOpt(options, 'showTrajectory', true);
    opt.showRobot      = getOpt(options, 'showRobot',      true);
    opt.showTarget     = getOpt(options, 'showTarget',     false);
    opt.showCovariance = getOpt(options, 'showCovariance', false);
    opt.showVFH        = getOpt(options, 'showVFH',        false);
    opt.showGroundTruth = getOpt(options, 'showGroundTruth', false);

end

% ======================================================================
% HELPER: getOpt
%   Returns the value of a struct field if it exists, otherwise the
%   supplied default.
% ======================================================================
function v = getOpt(s, field, default)
    if isfield(s, field)
        v = s.(field);
    else
        v = default;
    end
end