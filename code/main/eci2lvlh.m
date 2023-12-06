function Xlvlh = eci2lvlh(cfg, Xeci, t0)
%LVLH2ECI Converts a state written in LVLH frame to ECI frame
% Uses the definition within cfg

mu = cfg.system.mu;
rT = cfg.system.rT;
w = cfg.system.w;

Xlvlh = zeros(size(Xeci));

% compute the ECI state
for i = 1:size(Xeci, 2)
    t = i*cfg.simulation.dt + t0;
    % compute the rotation matrix
    rotMat = [-sin(w*t), 0, -cos(w*t); 
              0,         1,        0; 
              cos(w*t),  0, -sin(w*t)];
    % rotMat = [cos(w*t), 0, -sin(w*t); 
    %           0,         1,        0; 
    %           sin(w*t),  0, cos(w*t)];
    rotMatdot = [-w*cos(w*t), 0, w*sin(w*t); 
              0,         1,        0; 
              -w*sin(w*t),  0, -w*cos(w*t)];
    Xlvlh(1:3, i) = rotMat'*(Xeci(1:3, i) - [rT*cos(w*t); 0; rT*sin(w*t)]);
    Xlvlh(4:6, i) = rotMat'*(Xeci(4:6, i) - [-w*rT*sin(w*t); 0; w*rT*cos(w*t)]) + rotMatdot'*(Xeci(1:3, i)- [rT*cos(w*t); 0; rT*sin(w*t)]);
end

end

