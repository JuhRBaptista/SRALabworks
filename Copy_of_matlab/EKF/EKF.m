function [p, Cp] = EKF(dsr, dsl, p, Cp, data, params)
    u = [dsr, dsl];

    % --- Prediction ---
    [p, Cp] = ekfPredict(p, Cp, u);

    % --- Measurement model parameters ---
    nBeams       = 120;     % evenly-spaced beams to evaluate
    sigma_R_rel  = 0.035;   % relative range noise coefficient
    sigma_R_base = 0.030;   % absolute noise floor [m]
    minApplyBeams = 8;      % skip update if fewer beams pass the gate

    % Warm gate: looser for the first warmIters steps so the filter can
    % settle after globalLocalize before the tight gate takes effect.
    warmIters = 15;
    e_normal  = 2.5;
    e_warm    = 3.0;
    if isfield(params, 'ekfIter') && params.ekfIter <= warmIters
        e_gate = e_warm;
    else
        e_gate = e_normal;
    end

    % --- Beam selection: evenly spaced over [1 .. 360] ---
    beam_idx = round(linspace(1, numel(data.Ranges), nBeams));

    V = [];   % innovation vector
    G = [];   % stacked Jacobians
    R = [];   % measurement noise variances
    valid_beams = 0;
    for k = 1:numel(beam_idx)
        i   = beam_idx(k);
        o   = data.Ranges(i);
        ang = data.Angles(i);

        % Skip invalid beams
        if ~isfinite(o) || o <= 0.12 || o >= params.maxRange
            continue;
        end
        valid_beams = valid_beams+1;

        % Range-dependent noise variance (with floor)
        r_i = (sigma_R_rel * o + sigma_R_base)^2;

        % Predicted observation and Jacobian
        [o_hat, hit_pt] = g(p, params.map, ang, params);
        if isempty(hit_pt), continue; end   % sem impacto -> salta o beam
        
        Jg = obsJacobian(p, hit_pt, params.sensor_offset);
        % Innovation
        v_i = o - o_hat;

        % Scalar innovation covariance for the gate
        s_i = Jg * Cp * Jg' + r_i;

        % Mahalanobis gate
        if (v_i * (1/s_i) * v_i) <= e_gate^2
            V = [V; v_i];
            G = [G; Jg ];
            R = [R; r_i];
        end
    end
        % --- Guard: skip update if too few beams passed the gate ---
    if numel(V) < minApplyBeams
        return;
    end

    % --- Update ---
    [p, Cp] = ekfUpdate(p, Cp, V, G, R);

    
end
