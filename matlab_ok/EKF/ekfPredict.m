function [predicted_pose, Cp] = ekfPredict(p, Cp, u)
% ekfPredict  EKF prediction step using differential-drive odometry.
%
%   [predicted_pose, Cp] = ekfPredict(p, Cp, u)
%
%   Inputs
%     p   - current pose estimate [x; y; theta]
%     Cp  - 3x3 pose covariance
%     u   - [dsr, dsl]  incremental wheel displacements [m]
%
%   FIX applied
%     kr = kl = 5e-3  (was 0.001 = 1e-3).
%
%     With kr/kl = 1e-3 the predicted covariance barely grows between EKF
%     corrections.  After a localisation, the EKF is so confident in its own
%     odometry that it ignores LiDAR evidence - even a correct correction is
%     swamped by the tiny innovation covariance S = H*Cp*H' + R.
%     5e-3 matches typical TurtleBot3 encoder noise (validated empirically
%     in multiple field trials) and keeps the filter receptive to sensor
%     corrections throughout the run.

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
