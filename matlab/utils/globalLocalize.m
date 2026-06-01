function [p, residual] = globalLocalize(lddata, map, scale, origin, maxRange, sensor_offset)

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
    aMeas  = angles(idx);

    % ------------------------------------------------------------------ %
    % 1.  Distance transform
    % ------------------------------------------------------------------ %
    obstMask = map >= 0.5;
    D_m      = bwdist(obstMask) / scale;

    [H, W] = size(map);
    w2r = @(x) max(1, min(H, round(x * scale + origin) + 1));
    w2c = @(y) max(1, min(W, round(y * scale + origin) + 1));

    % ------------------------------------------------------------------ %
    % 2.  Candidate positions — coarse free-space grid
    % ------------------------------------------------------------------ %
    r_robot_m  = 0.105;
    r_robot_px = ceil(r_robot_m * scale);
    freeMap    = ~imdilate(obstMask, strel('disk', r_robot_px));

    stepPx  = max(2, round(0.20 * scale));
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
    % 4.  STAGE 1: DT pre-filter
    % ------------------------------------------------------------------ %
    topN   = 20;
    scores = inf(nCand * nA, 1);
    poses  = zeros(nCand * nA, 3);
    entry  = 0;

    for k = 1:nCand
        cx = xCand(k);  cy = yCand(k);
        for a = 1:nA
            ctheta = candAngles(a);
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
            scores(entry)  = sumErr / nBeams;
            poses(entry,:) = [cx, cy, ctheta];
        end
    end

    [~, order] = sort(scores(1:entry));
    topIdx     = order(1:min(topN, entry));
    topPoses   = poses(topIdx, :);

    % ------------------------------------------------------------------ %
    % 5.  STAGE 2: ray-cast re-scoring
    % ------------------------------------------------------------------ %
    params.map      = map;
    params.scale    = scale;
    params.origin   = origin;
    params.maxRange = maxRange;

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
                if all(Jg == 0), continue; end
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

    % ------------------------------------------------------------------ %
    % 6.  Validate: robot footprint must be free — try ranked candidates
    % ------------------------------------------------------------------ %
    [dx, dy] = meshgrid(-r_robot_px:r_robot_px, -r_robot_px:r_robot_px);
    disc     = (dx.^2 + dy.^2) <= r_robot_px^2;
    [offR, offC] = find(disc);
    offR = offR - r_robot_px - 1;
    offC = offC - r_robot_px - 1;
    
    % Build the full ranked list from Stage 2 scores (already sorted above)
    % topPoses is already in score order — we extend the search to all of them
    allSortedPoses = poses(order, :);   % full ranked list, not just top-N
    
    p        = [];
    residual = inf;
    
    for t = 1:size(allSortedPoses, 1)
        cand = allSortedPoses(t, :);
    
        gr = max(1, min(H, round(cand(1) * scale + origin) + 1));
        gc = max(1, min(W, round(cand(2) * scale + origin) + 1));
    
        rows   = max(1, min(H, gr + offR));
        cols   = max(1, min(W, gc + offC));
        linIdx = sub2ind([H W], rows, cols);
    
        if any(map(linIdx) >= 0.2)
            continue;   % footprint overlaps obstacle — try next
        end
    
        % Found a free pose — now ray-cast score it properly
        ctheta = cand(3);
        xs = cand(1) + sensor_offset(1)*cos(ctheta) - sensor_offset(2)*sin(ctheta);
        ys = cand(2) + sensor_offset(1)*sin(ctheta) + sensor_offset(2)*cos(ctheta);
        candidate = [xs; ys; ctheta];
    
        sumErr = 0;  nValid = 0;
        for b = 1:nBeamsRC
            [z_hat, Jg] = g(candidate, map, aRC(b), params);
            if all(Jg == 0), continue; end
            sumErr = sumErr + abs(rRC(b) - z_hat);
            nValid = nValid + 1;
        end
    
        if nValid < 6, continue; end

        p        = [cand(1); cand(2); normalizeAngle(cand(3))];
        residual = sumErr / nValid;
        break;
    end
        
    if isempty(p)
        % No free pose found — fall back to geometric best and warn
        warning('[globalLocalize] All candidates overlap obstacles. Using best geometric pose.');
        p        = [bestPose(1); bestPose(2); normalizeAngle(bestPose(3))];
        residual = inf;
    end
    
    fprintf('[globalLocalize] est: x=%.2f  y=%.2f  theta=%.1f deg  residual=%.3f m\n', ...
            p(1), p(2), rad2deg(p(3)), residual);
end
