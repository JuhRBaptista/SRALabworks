function [p, residual] = globalLocalize(lddata, map, scale, origin, maxRange, sensor_offset)
% globalLocalize  Estimate the robot pose from a single LiDAR scan and a
%                 known occupancy map via brute-force scan matching.
%
%   [p, residual] = globalLocalize(lddata, map, scale, origin, maxRange, sensor_offset)
%
%   Inputs
%     lddata        - lidar data struct (fields: Ranges, Angles)
%     map           - binary occupancy grid (1 = obstacle)
%     scale         - px / m
%     origin        - world origin offset in pixels
%     maxRange      - maximum usable LiDAR range [m]
%     sensor_offset - [dx; dy] LiDAR offset from the wheel axis in robot frame
%
%   Outputs
%     p        - best-fit pose [x; y; theta]
%     residual - mean range error of inlier beams [m]  (lower = better)
%
% KEY IMPROVEMENTS over the previous version
%   * Returns a proper residual score so the caller can compare retries and
%     pick the best (instead of the unreliable inlier-ratio).
%   * sensor_offset is applied so the predicted ranges match what the LiDAR
%     actually sees (previously the offset was ignored).
%   * Uses a finer angle grid (every 10 deg instead of 90 deg) to avoid
%     missing the true orientation on maps with non-axis-aligned corridors.
%   * Candidate positions come only from free cells (dilated away from walls)
%     so illegal poses are never evaluated.
%   * The distance threshold that classifies a beam as an inlier is kept at
%     0.12 m (two beam-width resolution steps), matching the EKF gate width.

    if nargin < 6
        sensor_offset = [0; 0];
    end

    % ------------------------------------------------------------------ %
    % 1.  Build a robust, noise-reduced range image from the scan
    % ------------------------------------------------------------------ %
    ranges = lddata.Ranges(:);
    angles = lddata.Angles(:);

    % Valid beams: finite, in [minRange, maxRange]
    valid   = isfinite(ranges) & ranges >= 0.12 & ranges <= maxRange;
    ranges  = ranges(valid);
    angles  = angles(valid);

    if numel(ranges) < 10
        p        = [0; 0; 0];
        residual = inf;
        return
    end

    % Subsample to ~72 evenly-spaced beams (speed / robustness balance)
    nBeams   = min(72, numel(ranges));
    idx      = round(linspace(1, numel(ranges), nBeams));
    rSub     = ranges(idx);
    aSub     = angles(idx);

    % ------------------------------------------------------------------ %
    % 2.  Candidate positions: free cells in a dilated-away-from-walls map
    % ------------------------------------------------------------------ %
    robot_px = max(1, round(0.105 * scale));   % ~10.5 cm safety radius
    se       = strel('disk', robot_px);
    free_map = ~imdilate(map > 0.5, se);       % cells safe for the robot centre

    [rows, cols] = find(free_map);
    xW = (rows - 1 - origin) / scale;          % world x  (row -> x)
    yW = (cols - 1 - origin) / scale;          % world y  (col -> y)

    % ------------------------------------------------------------------ %
    % 3.  Candidate orientations: every 10 degrees
    % ------------------------------------------------------------------ %
    candAngles = (0 : 10 : 350) * pi/180;
    nA         = numel(candAngles);
    nC         = numel(xW);

    fprintf('[globalLocalize] %d position candidates × %d angles = %d evaluations\n', ...
            nC, nA, nC*nA);

    % ------------------------------------------------------------------ %
    % 4.  Brute-force scan matching
    % ------------------------------------------------------------------ %
    inlierThr = 0.12;   % [m] beam counted as inlier when |z - z_hat| < thr
    minBeams  = 8;      % minimum valid raycasts per candidate (else skip)

    bestResidual = inf;
    p            = [xW(1); yW(1); 0];

    params.map      = map;
    params.scale    = scale;
    params.origin   = origin;
    params.maxRange = maxRange;

    for k = 1:nC
        cx = xW(k);  cy = yW(k);

        for a = 1:nA
            ctheta = candAngles(a);

            % Apply the sensor offset in the candidate robot frame
            phi = ctheta;
            xs  = cx + sensor_offset(1)*cos(phi) - sensor_offset(2)*sin(phi);
            ys  = cy + sensor_offset(1)*sin(phi) + sensor_offset(2)*cos(phi);

            candidate = [xs; ys; ctheta];   % pose seen by the LiDAR

            % Raycast all subsampled beams
            hitCount   = 0;
            sumErr     = 0;
            nCastValid = 0;

            for b = 1:nBeams
                [z_hat, Jz] = g(candidate, map, aSub(b), params);
                if all(Jz == 0), continue; end   % no wall hit
                nCastValid = nCastValid + 1;
                err = abs(rSub(b) - z_hat);
                sumErr = sumErr + err;
                if err < inlierThr
                    hitCount = hitCount + 1;
                end
            end

            if nCastValid < minBeams, continue; end

            % Score = mean range error over *all* raycasted beams
            % (lower is better; inlier ratio alone is unreliable on
            %  self-similar / symmetric maps)
            score = sumErr / nCastValid;

            if score < bestResidual
                bestResidual = score;
                % Store the *robot-centre* pose (undo sensor offset)
                p = [cx; cy; ctheta];
            end
        end
    end

    residual = bestResidual;

    fprintf('[globalLocalize] best: x=%.2f y=%.2f theta=%.1f deg  residual=%.3f m\n', ...
            p(1), p(2), rad2deg(p(3)), residual);
end
