function H = obsJacobian(p, hit_pt, sensor_offset)
    % hit_pt = [xw, yw] — coordenadas mundo do ponto de impacto
    theta = p(3);
    xs = p(1) + sensor_offset(1)*cos(theta) - sensor_offset(2)*sin(theta);
    ys = p(2) + sensor_offset(1)*sin(theta) + sensor_offset(2)*cos(theta);
    
    dx = hit_pt(1) - xs;
    dy = hit_pt(2) - ys;
    r  = sqrt(dx^2 + dy^2);
    
    % doffset/dtheta
    dxs_dth = -sensor_offset(1)*sin(theta) - sensor_offset(2)*cos(theta);
    dys_dth =  sensor_offset(1)*cos(theta) - sensor_offset(2)*sin(theta);
    
    H = [-dx/r, -dy/r, (-dx*dxs_dth - dy*dys_dth)/r];
end