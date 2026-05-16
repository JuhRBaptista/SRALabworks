function scan = scanFilter(scan)

    ranges = sqrt(scan(:,1).^2 + scan(:,2).^2);

    maxRange = 2.0;
    minRange = 0.1;

    valid = ~isnan(ranges) & ~isinf(ranges) & ...
            ranges < maxRange & ranges > minRange;

    scan = scan(valid, :);

end