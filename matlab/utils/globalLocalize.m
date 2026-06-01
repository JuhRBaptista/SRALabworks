function [p, residual] = globalLocalize(lddata, map, scale, origin, maxRange, sensor_offset)
% globalLocalize  Two-stage global pose estimation.
%
%   STAGE 1 — Distance-transform pre-filter (fast)
%     Scores all candidate poses using the DT endpoint trick.
%     Cheap but ambiguous on symmetric maps — used only to shortlist.
%
%   STAGE 2 — Ray-cast re-scoring of top candidates (accurate)
%     The top topN poses from stage 1 are re-scored using proper ray-
%     casting through g.m, which checks BOTH distance and direction to
%     the nearest wall.  This breaks symmetry and selects the true pose.

    if nargin < 6, sensor_offset = [0; 0]; end

    % ------------------------------------------------------------------ %
    % 0.  Build a clean, subsampled range image
    % ------------------------------------------------------------------ %
    ranges = lddata.Ranges(:);
    angles = lddata.Angles(:);

    valid  = isfinite(ranges) & ranges >= 0.12 & ranges <= maxRange;
    ranges = ranges(valid);
    angles = angles(valid);

    if numel(ranges) < 10
        p = [0;0;0];  residual = inf;  return
    end

    nBeams = 48;
    idx    = round(linspace(1, numel(ranges), nBeams));
    rMeas  = ranges(idx);
    aMeas  = angles(idx);   % in robot frame

    % ------------------------------------------------------------------ %
    % 1.  Distance transform  (computed once, used throughout stage 1)
    % ------------------------------------------------------------------ %
    obstMask = map >= 0.5;
    D_m      = bwdist(obstMask) / scale;   % distance to nearest wall [m]

    [H, W] = size(map);
    w2r = @(x) max(1, min(H, round(x * scale + origin + 1)));
    w2c = @(y) max(1, min(W, round(y * scale + origin + 1)));

    % ------------------------------------------------------------------ %
    % 2.  Candidate positions — coarse free-space grid
    % ------------------------------------------------------------------ %
    r_px    = max(1, round(0.105 * scale));
    freeMap = ~imdilate(obstMask, strel('disk', r_px));

    stepPx  = max(2, round(0.20 * scale));   % ~20 cm steps
    [GC, GR] = meshgrid(1:stepPx:W, 1:stepPx:H);
    GR = GR(:);  GC = GC(:);
    keep = freeMap(sub2ind([H W], GR, GC));
    GR = GR(keep);  GC = GC(keep);

    xCand = (GR - 1 - origin) / scale;
    yCand = (GC - 1 - origin) / scale;
    nCand = numel(xCand);

    % ------------------------------------------------------------------ %
    % 3.  Candidate orientations — every 20 deg
    % ------------------------------------------------------------------ %
    candAngles = (0:20:359) * pi/180;
    nA         = numel(candAngles);

    % ------------------------------------------------------------------ %
    % 4.  STAGE 1: DT pre-filter — score every pose, keep top topN
    % ------------------------------------------------------------------ %
    topN   = 20;   % how many to pass to the accurate stage 2
    scores = inf(nCand * nA, 1);
    poses  = zeros(nCand * nA, 3);
    entry  = 0;

    for k = 1:nCand
        cx = xCand(k);  cy = yCand(k);
        for a = 1:nA
            ctheta = candAngles(a);
            % LiDAR origin
            xs = cx + sensor_offset(1)*cos(ctheta) - sensor_offset(2)*sin(ctheta);
            ys = cy + sensor_offset(1)*sin(ctheta) + sensor_offset(2)*cos(ctheta);

            beamAng = ctheta + aMeas;
            sumErr  = 0;
            for b = 1:nBeams
                xw = xs + rMeas(b)*cos(beamAng(b));
                yw = ys + rMeas(b)*sin(beamAng(b));
                sumErr = sumErr + D_m(w2r(xw), w2c(yw));
            end

            entry = entry + 1;
            scores(entry) = sumErr / nBeams;
            poses(entry,:) = [cx, cy, ctheta];
        end
    end

    % Sort and keep top candidates
    [~, order]   = sort(scores(1:entry));
    topIdx       = order(1:min(topN, entry));
    topPoses     = poses(topIdx, :);

    % ------------------------------------------------------------------ %
    % 5.  STAGE 2: ray-cast re-scoring of shortlisted candidates
    %
    %     Uses g.m directly — checks both distance AND direction to walls.
    %     This is what breaks symmetry: a mirrored pose has beams that
    %     travel in the wrong direction relative to the walls, so g.m
    %     returns a large error even if the DT score was low.
    % ------------------------------------------------------------------ %
    params.map      = map;
    params.scale    = scale;
    params.origin   = origin;
    params.maxRange = maxRange;

    % Use more beams for accurate scoring
    nBeamsRC = 72;
    idxRC    = round(linspace(1, numel(ranges), nBeamsRC));
    rRC      = ranges(idxRC);
    aRC      = angles(idxRC);

    bestResidual = inf;
    bestPose     = topPoses(1,:);

    for t = 1:size(topPoses, 1)
        cx     = topPoses(t,1);
        cy     = topPoses(t,2);
        ctheta = topPoses(t,3);

        % Fine orientation sweep ±15 deg around DT-selected angle
        fineAngles = ctheta + (-15:5:15)*pi/180;

        for fa = 1:numel(fineAngles)
            theta_f = fineAngles(fa);
            xs = cx + sensor_offset(1)*cos(theta_f) - sensor_offset(2)*sin(theta_f);
            ys = cy + sensor_offset(1)*sin(theta_f) + sensor_offset(2)*cos(theta_f);

            candidate = [xs; ys; theta_f];
            sumErr    = 0;
            nValid    = 0;

            for b = 1:nBeamsRC
                [z_hat, Jg] = g(candidate, map, aRC(b), params);
                if all(Jg == 0), continue; end   % beam hit maxRange, skip
                sumErr = sumErr + abs(rRC(b) - z_hat);
                nValid = nValid + 1;
            end

            if nValid < 6, continue; end
            score = sumErr / nValid;

            if score < bestResidual
                bestResidual = score;
                bestPose     = [cx, cy, theta_f];
            end
        end
    end

    p        = [bestPose(1); bestPose(2); normalizeAngle(bestPose(3))];
    residual = bestResidual;

    fprintf('[globalLocalize] est: x=%.2f  y=%.2f  theta=%.1f deg  residual=%.3f m\n', ...
            p(1), p(2), rad2deg(p(3)), residual);
end