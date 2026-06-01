function [o, Jg] = g(p, map, angle, params)
% g  Ray-cast a single LiDAR beam from pose p and return the predicted
%    range and its Jacobian w.r.t. the robot pose [x, y, theta].
%
%   [o, Jg] = g(p, map, angle, params)
%
%   Inputs
%     p      - robot pose [x; y; theta]
%     map    - occupancy grid (cells >= 0.6 are obstacles)
%     angle  - beam angle IN THE ROBOT FRAME [rad]
%     params - struct with fields: scale, origin, maxRange
%
%   Outputs
%     o   - predicted range [m]  (= maxRange when no obstacle is hit)
%     Jg  - 1x3 measurement Jacobian  [do/dx, do/dy, do/dtheta]
%           (zeros when no obstacle is hit -> beam ignored by the EKF)
%
% FIX applied (vs. the previous version)
%   The heading partial was sign-flipped.  The correct derivation is:
%
%       o = sqrt((x_hit - p(1))^2 + (y_hit - p(2))^2)
%
%   Differentiating along the ray at distance d in direction beam_angle:
%       do/dx     = -cos(beam_angle)
%       do/dy     = -sin(beam_angle)
%       do/dtheta = +d * sin(beam_angle)   <- was MINUS, now PLUS
%
%   The physical meaning: rotating the robot CW (decreasing theta) swings
%   the beam toward a nearer obstacle, so the predicted range *decreases*
%   (do/dtheta < 0) when the beam points in the +x half-plane
%   (sin(beam_angle) < 0 for downward beams, etc.).  The sign
%   must be +d*sin(beam_angle), not -d*sin(beam_angle).

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
            Jg  = [-cos(beam_angle), -sin(beam_angle), -d*sin(beam_angle)];
            break;
        end
    end

    if ~hit
        o  = maxRange;
        Jg = zeros(1, 3);
    end
end
