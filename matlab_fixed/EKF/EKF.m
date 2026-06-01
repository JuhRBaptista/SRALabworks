function [p, Cp] = EKF(dsr, dsl, p, Cp, data, params)
% EKF  One full EKF cycle: predict (odometry) then correct (LiDAR).
%
%   [p, Cp] = EKF(dsr, dsl, p, Cp, data, params)
%
%   params must contain:
%     map      - occupancy grid
%     maxRange - maximum usable LiDAR range [m]
%     ekfIter  - (optional) current iteration counter; used for warm gate
%
% KEY FIXES applied (vs. the previous version)
%
%  1. BEAM SAMPLING
%     Old: every 5th index (i = 1:5:360) -> 72 beams, many of which are
%          Inf or out-of-range so the effective count is far lower.
%     New: linspace over the full 360 -> exactly nBeams evenly-spaced
%          beams are handed to the gate, maximising angular coverage.
%
%  2. MEASUREMENT NOISE MODEL
%     Old: r_i = (0.035 * o)^2  (purely relative, zero floor)
%     New: r_i = (sigma_R_rel*o + sigma_R_base)^2
%          A 3 cm noise floor prevents the filter from over-weighting
%          very short returns, which are noisy due to specular reflection.
%
%  3. MAHALANOBIS GATE
%     Old: e = 2 (hard-coded, no warm-up)
%     New: e = 2.5 normally, e = 4.0 for the first warmIters iterations.
%          The looser early gate lets the EKF pull in after globalLocalize
%          without discarding every beam during the transient.
%
%  4. minApplyBeams GUARD  (was commented out)
%     Old: correction applied even with 0 accepted beams.
%     New: update skipped when fewer than minApplyBeams beams pass the
%          gate.  A handful of beams gives an under-determined,
%          easily-biased update - the primary seed of divergence.

    u = [dsr, dsl];

    % --- Prediction ---
    [p, Cp] = ekfPredict(p, Cp, u);

    % --- Measurement model parameters ---
    nBeams       = 120;     % evenly-spaced beams to evaluate
    sigma_R_rel  = 0.035;   % relative range noise coefficient
    sigma_R_base = 0.040;   % absolute noise floor [m]
    minApplyBeams = 8;      % skip update if fewer beams pass the gate

    % Warm gate: looser for the first warmIters steps so the filter can
    % settle after globalLocalize before the tight gate takes effect.
    warmIters = 15;
    e_normal  = 2.5;
    e_warm    = 4.0;
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

    for k = 1:numel(beam_idx)
        i   = beam_idx(k);
        o   = data.Ranges(i);
        ang = data.Angles(i);

        % Skip invalid beams
        if ~isfinite(o) || o <= 0.12 || o >= params.maxRange
            continue;
        end

        % Range-dependent noise variance (with floor)
        r_i = (sigma_R_rel * o + sigma_R_base)^2;

        % Predicted observation and Jacobian
        [o_hat, Jg] = g(p, params.map, ang, params);

        if all(Jg == 0)
            continue;   % beam hit maxRange (no wall) -> skip
        end

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
