function [steerAngle, binaryHist] = VFH(robotPose, targetPose, mapParams, avoidParams)
    % VFH - Computes a steering direction using Vector Field Histogram.

    % mapParams
    %   map              : occupancy grid map
    %   scale            : map scale to real world
    %   origin           : map origin

    % avoidParams
    %   windowSize       : local obstacle search window
    %   numSectors       : number of histogram sectors
    %   sectorWidth      : angular width of each sector
    %   smoothSigma      : Gaussian smoothing parameter
    %   threshold        : obstacle threshold for binary histogram
    %   valleyMinWidth   : minimum valid valley width

    % Local occupancy window
    halfWindow = floor(avoidParams.windowSize / 2);
    
    % Robot position in occupancy grid
    robotGridX = round(robotPose(1) * mapParams.scale) + mapParams.origin;
    robotGridY = round(robotPose(2) * mapParams.scale) + mapParams.origin;
    
    % Window boundaries
    xMin = max(1, robotGridX - halfWindow);
    xMax = min(size(mapParams.map, 1), robotGridX + halfWindow);
    yMin = max(1, robotGridY - halfWindow);
    yMax = min(size(mapParams.map, 2), robotGridY + halfWindow);
    
    % Extract local occupancy map
    localMap = mapParams.map(xMin:xMax, yMin:yMax);
    
    % Robot position inside local map
    centerX = robotGridX - xMin + 1;
    centerY = robotGridY - yMin + 1;
    
    % Occupied cells
    [obsX, obsY] = find(localMap);

    % Polar obstacle histogram
    histogram = zeros(1, avoidParams.numSectors);
    
    % Distance weighting parameters
    b = 3;
    a = halfWindow * b * sqrt(2);
    
    % Build obstacle histogram
    for i = 1:length(obsX)

        occValue = localMap(obsX(i), obsY(i));

        dx = obsX(i) - centerX;
        dy = obsY(i) - centerY;
        d  = norm([dx, dy]);
        
        % Ignore robot cell
        if d == 0
            continue;
        end
        
        % Obstacle direction
        angle = atan2(dy, dx);

        % Obstacle influence magnitude
        magnitude = max(0, occValue^2 * (a - b*d));
        
        % Histogram sector
        sector = mod(round((angle + pi) / avoidParams.sectorWidth), avoidParams.numSectors) + 1;

        histogram(sector) = histogram(sector) + magnitude;
    end

    % Smooth histogram 
    histogram = smoothHistogram(histogram, avoidParams.smoothSigma);

    % Binary occupancy
    binaryHist = histogram > avoidParams.threshold;

    % Target direction
    targetAngle = atan2(targetPose(2) - robotPose(2), ...
                        targetPose(1) - robotPose(1));

    targetSector = mod(round((targetAngle + pi) / avoidParams.sectorWidth), avoidParams.numSectors) + 1;

    % Move directly toward target if direction is free
    if isSectorFree(histogram, targetSector, floor(avoidParams.valleyMinWidth/2), avoidParams.threshold, avoidParams.numSectors)
        steerAngle = targetAngle;
        return;
    end

    % Detect candidate valleys
    valleyEdges = findValleyEdges(binaryHist, avoidParams.numSectors);

    % No collision-free direction available
    if isempty(valleyEdges)
        steerAngle = NaN; 
        return;
    end

    % Select valley closest to target direction
    diff = min(abs(valleyEdges - targetSector), ...
               avoidParams.numSectors - abs(valleyEdges - targetSector));

    [~, idx] = min(diff);
    startSector = valleyEdges(idx);

    % Count valley width
    [freeCount, direction] = countFreeBins(binaryHist, startSector, avoidParams.numSectors);

    if freeCount >= avoidParams.valleyMinWidth
        % Wide valley → go inside it
        offset      = floor(avoidParams.valleyMinWidth / 2);
        steerSector = mod(startSector + direction*offset - 1, avoidParams.numSectors) + 1;
    else
        % Narrow valley → go to its center
        endSector   = mod(startSector + direction*freeCount - 1, avoidParams.numSectors) + 1;
        steerSector = round(mod((startSector + endSector)/2 - 1, avoidParams.numSectors)) + 1;
    end

    % Convert sector to angle
    steerAngle = (steerSector - 1) * avoidParams.sectorWidth - pi + avoidParams.sectorWidth/2;
end

function hs = smoothHistogram(h, sigma)
    % smoothHistogram - Applies circular Gaussian smoothing.

    halfW  = ceil(3 * sigma);
    kernel = exp(-(-halfW:halfW).^2 / (2 * sigma^2));
    kernel = kernel / sum(kernel);
    n      = length(h);
    hs     = zeros(1, n);
    for i = 1:n
        for j = 1:length(kernel)
            idx    = mod(i + j - halfW - 2, n) + 1;
            hs(i)  = hs(i) + kernel(j) * h(idx);
        end
    end
end

function free = isSectorFree(h, centerSector, halfWidth, threshold, numSectors)
    % isSectorFree - Checks if a histogram sector is obstacle free.
    free = true;
    for offset = -halfWidth:halfWidth
        k = mod(centerSector + offset - 1, numSectors) + 1;
        if h(k) > threshold
            free = false;
            return;
        end
    end
end

function edges = findValleyEdges(binary, numSectors)
    % findValleyEdges - Detects free-space valley boundaries.

    edges = [];
    for k = 1:numSectors
        curr = binary(k);
        prev = binary(mod(k - 2, numSectors) + 1);
        if prev == 1 && curr == 0      % falling edge = início do vale
            edges = [edges, k];
        elseif prev == 0 && curr == 1  % rising edge = fim do vale
            last_free = mod(k - 2, numSectors) + 1;
            edges = [edges, last_free];
        end
    end
end

function [count, dir] = countFreeBins(binary, startSector, numSectors)
    % countFreeBins - Computes valley width and expansion direction.

    count = 0;
    k = startSector;
    next = mod(k, numSectors) + 1;
    if binary(next) == 0
        dir = 1;
    else
        dir = -1;
    end

    while binary(k) == 0
        count = count + 1;
        k = mod(k - 1 + dir, numSectors) + 1;
        if k == startSector; break; end   % full circle
    end
end