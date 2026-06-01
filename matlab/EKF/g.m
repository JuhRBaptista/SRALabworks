function [o, Jg] = g(p, map, angle, params)

    beam_angle = p(3) + angle;   % beam direction in world frame

    step     = 0.01;
    maxRange = params.maxRange;
    hit      = false;

    % --- Ray casting ---
    for d = 0 : step : maxRange

        xr = p(1) + d * cos(beam_angle);
        yr = p(2) + d * sin(beam_angle);

        gc  = worldToGrid([xr, yr], params);
        row = gc(1);
        col = gc(2);

        if row < 1 || row > size(map,1) || col < 1 || col > size(map,2)
            break;
        end

        if map(row, col) >= 0.6
            hit = true;
            o   = d;
            % CORRECTED: +d*sin(beam_angle)  (was -d*sin(beam_angle))
            Jg  = [-cos(beam_angle), -sin(beam_angle), d*sin(beam_angle)];
            break;
        end
    end

    if ~hit
        o  = maxRange;
        Jg = zeros(1, 3);
    end
end
