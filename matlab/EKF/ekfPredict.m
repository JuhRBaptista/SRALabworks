function [predicted_pose, Cp] = ekfPredict(p, Cp, u)

    L  = 0.16;
    kr = 0.0001;   % tune these down if ellipse still grows too fast
    kl = 0.0001;

    dr = u(1);
    dl = u(2);

    D         = (dr + dl) / 2;
    delta_phi = (dr - dl) / L;
    theta     = p(3);
    alpha     = theta + delta_phi / 2;   % heading at mid-step

    % State prediction
    x   = p(1) + D * cos(alpha);
    y   = p(2) + D * sin(alpha);
    phi = theta  + delta_phi;
    phi = normalizeAngle(phi);

    predicted_pose = [x; y; phi];


    % Jacobian wrt pose
    Fp = [1, 0, -D*sin(alpha);
          0, 1,  D*cos(alpha);
          0, 0,  1           ];

    % Noise covariance in wheel space
    Cn = [kr*abs(dr), 0;
          0,          kl*abs(dl)];

    % Jacobian wrt noise  — note /(2*L), not /2*L
    Fn = [0.5*cos(alpha) - D*sin(alpha)/(2*L),   0.5*cos(alpha) + D*sin(alpha)/(2*L);
          0.5*sin(alpha) + D*cos(alpha)/(2*L),   0.5*sin(alpha) - D*cos(alpha)/(2*L);
          1/L,                                   -1/L                                ];

    Cp = Fp * Cp * Fp' + Fn * Cn * Fn';
end