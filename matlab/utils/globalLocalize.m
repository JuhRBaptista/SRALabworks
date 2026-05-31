function p = globalLocalize(tbot, params, nScans)

    if nargin < 3
        nScans = 5;
    end

    map    = params.map;
    scale  = params.scale;
    origin = params.origin;
    
    % Collect multiple scans and compute median to reduce noise
    fprintf('[GlobalLocalize] Collecting %d scans...\n', nScans);
    allRanges = zeros(nScans, 360);
    for s = 1:nScans
        [~, d] = tbot.readLidar();
        allRanges(s, :) = d.Ranges(:)';
        pause(0.05);
    end
    medianRanges = median(allRanges, 1);   % 1x360 robust range image
    
    % Valid beam mask: finite, in range, and below maxRange
    beamIdxs  = 1:5:360;
    validMask = ~isinf(medianRanges) & medianRanges > 0.05 & ...
                medianRanges < params.maxRange;
    
    % Candidate orientations (every 90 degrees)
    nAngles = 4;                               % 10 degree resolution
    angles  = [0, pi/2, pi, 3*pi/2];

    % Build dilated map to exclude cells too close to obstacles
    robot_radius = 0.105;
    robot_pixels = round(robot_radius * scale);
    se           = strel('disk', robot_pixels);
    dilated_map  = imdilate(map, se);
    
    % Extract free-space candidate positions from dilated map
    [rows, cols] = find(dilated_map < 0.2);
    
    xW = (rows - origin) / scale;
    yW = (cols - origin) / scale;

    fprintf('[GlobalLocalize] Testing %d candidates x %d angles...\n', ...
        length(xW), nAngles);

    inlierThresh = 0.12; % max range error to count a beam as inlier
    minBeams     = 8;   % minimum valid beams required per candidate

    bestScore  = -1;
    p          = [0; 0; 0];
    
    % Brute-force search over candidate positions and orientations
    for k = 1:length(xW)
        for a = 1:length(angles)
            candidate = [xW(k); yW(k); angles(a)];
            inliers   = 0;
            nBeams    = 0;
            
            % Compare measured ranges with ray-cast predictions
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
            
            % Keep candidate with highest inlier ratio
            score = inliers / nBeams;
            if score > bestScore
                bestScore = score;
                p         = candidate;
            end
        end
    end

    fprintf('[GlobalLocalize] Coarse: x=%.2f y=%.2f th=%.2f score=%.3f\n', ...
        p(1), p(2), p(3), bestScore);
end