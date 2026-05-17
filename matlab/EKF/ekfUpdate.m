function [p, Cp] = ekfUpdate(p, Cp, V, G, R)

    if isempty(V)
        return;
    end
    
    R_mat = diag(R);

    % Innovation covariance
    S = G * Cp * G' + R_mat;

    % Kalman gain
    K = Cp * G' *inv(S);

    % State update
    p = p + K * V;
    p(3) = normalizeAngle(p(3));

    % Covariance update
    I = eye(size(Cp));

    % Joseph stabilized form
    Cp = (I - K*G) * Cp;

end