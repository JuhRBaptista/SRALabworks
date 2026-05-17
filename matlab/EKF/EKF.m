function [p, Cp] = EKF(dsr, dsl, p, Cp, data, params)

    u = [dsr, dsl];

    % Prediction step
    [p, Cp] = ekfPredict(p, Cp, u);

    V     = [];        % Innovation vector
    G     = [];        % Stacked Jacobians
    R_mat = [];        % Block-diagonal measurement covariance

    for i = 1:5:360

        distance = data.Ranges(i);

        if isinf(distance) || distance <= 0
            continue;
        end

        % LiDAR angle relative to robot frame
        angle = deg2rad(i - 1);

        % Measurement vector
        z = [angle; distance];

        % Measurement noise
        sigma_r     = 0.035 * distance;
        sigma_theta = deg2rad(1);

        R_i = diag([sigma_theta^2, sigma_r^2]);

        % Predicted observation and Jacobian
        [z_hat, Jz] = g(p, params.map, angle, params);

        % Ignore invalid observations
        if all(Jz(:) == 0)
            continue;
        end

        % Innovation
        V_i = z - z_hat;

        % Normalize angular innovation
        V_i(1) = normalizeAngle(V_i(1));

        % Innovation covariance
        S_i = Jz * Cp * Jz' + R_i;

        % Mahalanobis validation gate
        e = 2;

        if V_i' / S_i * V_i <= e^2

            V = [V; V_i];

            G = [G; Jz];

            R_mat = blkdiag(R_mat, R_i);

        end
    end

    % Correction step
    [p, Cp] = ekfUpdate(p, Cp, V, G, R_mat);

end