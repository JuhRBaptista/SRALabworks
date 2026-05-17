function [p, Cp] = EKF(dsr, dsl, p, Cp, data, params)  
    
    u = [dsr, dsl];

    [p, Cp] = ekfPredict(p, Cp, u);
            
    V = [];  % Innovations
    G = []; % Estimations convariance
    R = [];  % Noises

    for i=1:10:360

      observation = data.Ranges(i);

      % uncertainty is 3.5% of reading
      uncertainty = 0.035;
      R_i = (uncertainty*observation)^2;

      % estimate distance according to map (predicted observation)
      [estimate, Jg] = g(p, params.map, i, params);

      % innovation
      V_i = observation - estimate;

      % innovation covariance
      S_i = Jg*Cp*Jg' + R_i;

      % Mahalanobis validation gate: vin * 1/Si * vin < e^2
      e = 5;
      if (V_i^2)/S_i <= e^2
        V = [V; V_i];
        G = [G; Jg];
        R = [R; R_i];

      end
    end

    [p, Cp] = ekfUpdate(p, Cp, V, G, R);
end