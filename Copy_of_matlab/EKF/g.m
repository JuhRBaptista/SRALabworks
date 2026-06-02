function [o, hit_pt] = g(p, map, angle, params)
    sx = p(1) + params.sensor_offset(1)*cos(p(3)) ...
              - params.sensor_offset(2)*sin(p(3));
    sy = p(2) + params.sensor_offset(1)*sin(p(3)) ...
              + params.sensor_offset(2)*cos(p(3));

    beam_angle = p(3) + angle;
    step     = 0.01;
    maxRange = params.maxRange;
    hit_pt   = [];

    for d = 0 : step : maxRange
        xr = sx + d * cos(beam_angle);
        yr = sy + d * sin(beam_angle);

        gc  = worldToGrid([xr, yr], params);
        row = gc(1); col = gc(2);

        if row < 1 || row > size(map,1) || col < 1 || col > size(map,2)
            break;
        end

        if map(row, col) >= 0.6
            o      = d;
            hit_pt = [xr, yr];   % <-- ponto de impacto no mundo
            return;
        end
    end

    o      = maxRange;
    hit_pt = [];   % sem impacto
end