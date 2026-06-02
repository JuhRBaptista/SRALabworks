function [angle] = normalizeAngle(angle)
    angle = atan2(sin(angle), cos(angle));
end