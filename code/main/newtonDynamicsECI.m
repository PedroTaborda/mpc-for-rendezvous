function dx = newtonDynamicsECI(t, x, t0, s, cfg)
    r = x(1:3);
    v = x(4:6);

    mu = cfg.system.mu;
    mc = cfg.system.mc;
    Fgravity = -mu/norm(r)^3*r;
    F = cfg.system.F;
    W = cfg.system.W;
    w = cfg.system.w;
    
    % s containts the time for which each thruster is on
    instantThrust = zeros(3, 1);
    for i = 1:length(cfg.system.F)
        if t - t0 < s(i)
            instantThrust = instantThrust + W(i, :)'*F(i);
        end
    end

    % rotate the thrust vector into the ECI frame
    rotMat = [-sin(w*t), 0, -cos(w*t); 
              0,         1,        0; 
              cos(w*t),  0, -sin(w*t)];
    instantThrustECI = rotMat*instantThrust;


    % compute the acceleration
    a = Fgravity + instantThrustECI/mc;

    % compute the derivative of the state vector
    dx = [v; a];
end
