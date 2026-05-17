function [p, Cp] = ekfUpdate(p, Cp, V, G, R)
    if isempty(V)
        return;
    end
    
    n = length(V);
    R_mat = diag(R);              % n×n
    
    S = G * Cp * G' + R_mat;      % n×n innovation covariance
    K = Cp * G' / S;              % 3×n Kalman gain  (use / not inv)
    
    p  = p + K * V;               % 3×1 updated pose
    
    % Numerically stable Joseph form (or simple (I-KG) form):
    I = eye(3);
    Cp = (I - K * G) * Cp;       % more stable than Cp - K*S*K'
end