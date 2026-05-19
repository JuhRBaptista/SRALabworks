function [p, Cp] = ekfUpdate(p, Cp, V, G, R)

    if isempty(V)
        return;
    end

    R_mat = diag(R);

    % Innovation covariance
    S = G * Cp * G' + R_mat;

    % Kalman gain
    K = (S \ (G * Cp'))';
    
    p = p + K * V;
    p(3) = normalizeAngle(p(3));
    

    % Joseph stabilized form
    I  = eye(3);
    IKG = I - K*G;
    

    Cp  = IKG * Cp * IKG' + K * R_mat * K';
    
end