function xDot = newtonDynamics(t, x, s, params)
%NEWTONDYNAMICS Computes the state derivative of an object in orbit around
%the earth in the rotating (LVLH) frame

    % Unpack the parameters
    mc = params.system.mc;
    mu = params.system.mu;
    F = params.system.F;
    W = params.system.W;
    w = params.system.w; % omega
    rT = params.system.rT;

    B = [ ...
        zeros(3, 3); ...
        eye(3, 3)/mc;   ...
    ];

    % s containts the time for which each thruster is on
    instantThrust = zeros(3, 1);
    for i = 1:6
        if t < s(i)
            instantThrust = instantThrust + W(i, :)'*F(i);
        end
    end

    % Compute the state derivative, according to
    % x'' - 2*Omega y' - Omega^2 x = - mu (rT + x)/((rT + x)^2 + y^2 + z^2)^3/2 + mu/rT^2 + instantThrust(1)
    % y'' = - mu y/((rT + x)^2 + y^2 + z^2)^3/2 + instantThrust(3)
    % z'' + 2*Omega x' -w^2*z = - mu z/((rT + x)^2 + y^2 + z^2)^3/2 + instantThrust(2)

    xDot = [ ...
        x(4:6); ...
        2*w*x(5) + w^2*x(4) - mu*(rT + x(1))/((rT + x(1))^2 + x(2)^2 + x(3)^2)^3/2 + mu/rT^2 + instantThrust(1); ...
        - mu*x(2)/((rT + x(1))^2 + x(2)^2 + x(3)^2)^3/2 + instantThrust(3); ...
        -2*w*x(4) + w^2*x(6) - mu*x(3)/((rT + x(1))^2 + x(2)^2 + x(3)^2)^3/2 + instantThrust(2); ...
    ];
    
end


