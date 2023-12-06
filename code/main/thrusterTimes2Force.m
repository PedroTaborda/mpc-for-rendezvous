function F = thrusterTimes2Force(cfg, s, t)
%thrusterTimes2Force Provides force applied at time t by thrusters in cfg
%activated for the times specified in u
    F = zeros(3, 1);
    W = cfg.system.W;
    for i = 1:length(cfg.system.F)
        if t < s(i)
            F = F + W(i, :)'*cfg.system.F(i);
        end
    end
end