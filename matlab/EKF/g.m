function [z_hat, Jz] = g(p, map, beamIndex, params)

    beam_angle = p(3) + deg2rad(beamIndex - 1);
    step       = 0.05;
    maxRange   = params.maxRange;

    z_hat = maxRange;

    for d = 0:step:maxRange
        xr = p(1) + d * cos(beam_angle);
        yr = p(2) + d * sin(beam_angle);

        gc  = worldToGrid([xr, yr], params);
        row = gc(1);
        col = gc(2);

        if row < 1 || row > size(map,1) || col < 1 || col > size(map,2)
            break;
        end

        if map(row, col) >= 0.6
            z_hat = d;
            break;
        end
    end

    if z_hat >= maxRange
        Jz = zeros(1, 3);   % no valid hit — ignore this beam
    else
        Jz = [-cos(beam_angle), -sin(beam_angle), 0];
    end

end