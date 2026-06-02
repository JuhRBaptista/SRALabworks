function [predicted_pose, Cp] = ekfPredict(p, Cp, u)

    L  = 0.16;
    kr = 5e-3;   % was 1e-3 -- too small, covariance collapsed
    kl = 5e-3;

    dr = u(1);
    dl = u(2);

    D         = (dr + dl) / 2;
    delta_phi = (dr - dl) / L;
    theta     = p(3);
    alpha     = theta + delta_phi / 2;   % heading at mid-step

    % --- State prediction ---
    x   = p(1) + D * cos(alpha);
    y   = p(2) + D * sin(alpha);
    phi = normalizeAngle(theta + delta_phi);

    predicted_pose = [x; y; phi];

    % --- Jacobian w.r.t. pose ---
    Fp = [1, 0, -D*sin(alpha);
          0, 1,  D*cos(alpha);
          0, 0,  1           ];

    % --- Wheel noise covariance ---
    Cn = [kr*abs(dr), 0;
          0,          kl*abs(dl)];

    % --- Jacobian w.r.t. noise ---
    Fn = [0.5*cos(alpha) - D*sin(alpha)/(2*L),   0.5*cos(alpha) + D*sin(alpha)/(2*L);
          0.5*sin(alpha) + D*cos(alpha)/(2*L),   0.5*sin(alpha) - D*cos(alpha)/(2*L);
          1/L,                                   -1/L                                ];

    % --- Minimum guaranteed process noise ---
    Q = diag([1e-6, 1e-6, 1e-7]);

    Cp = Fp * Cp * Fp' + Fn * Cn * Fn' + Q;
end
