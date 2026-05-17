function [p, Cp] = EKF(dsr, dsl, p, Cp, data, params)

    u = [dsr, dsl];

    % Prediction step
    [p, Cp] = ekfPredict(p, Cp, u);

    V = [];        % Innovation vector
    G = [];        % Stacked Jacobians
    R = [];        % Block-diagonal measurement covariance

    for i = 1:360
        % LiDAR angle relative to robot frame
        angle = deg2rad(i-1);
        o = data.Ranges(i);

        if isinf(o) || o <= 0 || o >= params.maxRange
            continue;
        end

        r_i = (0.05 * o)^2; %0.06

        % Predicted observation and Jacobian
        [o_hat, Jg] = g(p, params.map, angle, params);

        if all(Jg == 0)
            continue;
        end

        % Innovation
        v_i = o - o_hat;
        
        % Innovation covariance
        s_i = Jg * Cp * Jg' + r_i;

        e = 3;   % gate size 
        if (v_i * (1/s_i) * v_i) <= e^2   

            V = [V; v_i];
            G = [G; Jg];
            R = [R; r_i];

        end
    end

    [p, Cp] = ekfUpdate(p, Cp, V, G, R);


end