function p = globalLocalize(tbot, params, nScans)

    if nargin < 3
        nScans = 5;
    end

    map    = params.map;
    scale  = params.scale;
    origin = params.origin;
    % [p(1), p(2), p(3), ~] = tbot.readPose();

    fprintf('[GlobalLocalize] Collecting %d scans...\n', nScans);
    allRanges = zeros(nScans, 360);
    for s = 1:nScans
        [~, d] = tbot.readLidar();
        allRanges(s, :) = d.Ranges(:)';
        pause(0.05);
    end
    medianRanges = median(allRanges, 1);   % 1x360 robust range image

    beamIdxs  = 1:5:360;
    validMask = ~isinf(medianRanges) & medianRanges > 0.05 & ...
                medianRanges < params.maxRange;

    nAngles = 5;                               % 10 degree resolution
    angles  = [0];

    robot_radius = 0.105;
    robot_pixels = round(robot_radius * scale);
    se           = strel('disk', robot_pixels);
    dilated_map  = imdilate(map, se);
    [rows, cols] = find(dilated_map < 0.2);

    xW = (rows - origin) / scale;
    yW = (cols - origin) / scale;

    fprintf('[GlobalLocalize] Testing %d candidates x %d angles...\n', ...
        length(xW), nAngles);

    inlierThresh = 0.12;
    minBeams     = 8;

    bestScore  = -1;
    p          = [0; 0; 0];

    for k = 1:5:length(xW)
        for a = 1:length(angles)
            candidate = [xW(k); yW(k); angles(a)];
            inliers   = 0;
            nBeams    = 0;

            for i = beamIdxs
                if ~validMask(i), continue; end
                o = medianRanges(i);
                [o_hat, Jg] = g(candidate, map, deg2rad(i-1), params);
                if all(Jg == 0), continue; end
                nBeams = nBeams + 1;
                if abs(o - o_hat) < inlierThresh
                    inliers = inliers + 1;
                end
            end

            if nBeams < minBeams, continue; end

            score = inliers / nBeams;
            if score > bestScore
                bestScore = score;
                p         = candidate;
            end
        end
    end

    fprintf('[GlobalLocalize] Coarse: x=%.2f y=%.2f th=%.2f score=%.3f\n', ...
        p(1), p(2), p(3), bestScore);
    % 
    % fineXY    = 0.4;
    % fineTheta = deg2rad(12);
    % stepXY    = 0.05;
    % stepTheta = deg2rad(2);
    % 
    % xRange = p(1)-fineXY    : stepXY    : p(1)+fineXY;
    % yRange = p(2)-fineXY    : stepXY    : p(2)+fineXY;
    % aRange = p(3)-fineTheta : stepTheta : p(3)+fineTheta;
    % 
    % bestScoreFine = -inf;
    % pFine         = p;
    % 
    % for xi = xRange
    %     for yi = yRange
    %         for ai = aRange
    %             candidate  = [xi; yi; normalizeAngle(ai)];
    %             totalScore = 0;
    % 
    %             for s = 1:nScans
    %                 scanRanges = allRanges(s, :);
    %                 vm = ~isinf(scanRanges) & scanRanges > 0.05 & ...
    %                      scanRanges < params.maxRange;
    %                 inliers = 0;
    %                 nBeams  = 0;
    %                 for i = beamIdxs
    %                     if ~vm(i), continue; end
    %                     o = scanRanges(i);
    %                     [o_hat, Jg] = g(candidate, map, deg2rad(i-1), params);
    %                     if all(Jg == 0), continue; end
    %                     nBeams = nBeams + 1;
    %                     if abs(o - o_hat) < inlierThresh
    %                         inliers = inliers + 1;
    %                     end
    %                 end
    %                 if nBeams >= minBeams
    %                     totalScore = totalScore + inliers / nBeams;
    %                 end
    %             end
    % 
    %             if totalScore > bestScoreFine
    %                 bestScoreFine = totalScore;
    %                 pFine         = candidate;
    %             end
    %         end
    %     end
    % end
    % 
    % p = pFine;
    % fprintf('[GlobalLocalize] Fine:   x=%.2f y=%.2f th=%.2f score=%.3f\n', ...
    % p(1), p(2), p(3), bestScoreFine);
end