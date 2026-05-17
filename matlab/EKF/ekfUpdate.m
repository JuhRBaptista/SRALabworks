function [p, Cp] = ekfUpdate(p, Cp, V, G, R_mat)

    if isempty(V)
        return;
    end

    % Innovation covariance
    S = G * Cp * G' + R_mat;

    % Kalman gain
    K = Cp * G' / S;

    % State update
    p = p + K * V;

    % Normalize heading
    p(3) = normalizeAngle(p(3));

    % Covariance update
    I = eye(size(Cp));

    % Joseph stabilized form
    Cp = (I - K * G) * Cp * (I - K * G)' + K * R_mat * K';

end