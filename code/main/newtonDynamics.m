function xDot = newtonDynamics(t, x, t0, s, params)
%NEWTONDYNAMICS 

    % Unpack the parameters
    mc = params.system.mc;
    mu = params.system.mu;
    F = params.system.F;
    W = params.system.W;
    w = params.system.w;
    rT = params.system.rT;

    B = [ ...
        zeros(3, 3); ...
        eye(3, 3)/mc;   ...
    ];

    % s containts the time for which each thruster is on
    instantThrust = zeros(3, 1);
    for i = 1:6
        if t - t0 < s(i)
            instantThrust = instantThrust + W(i, :)'*F(i);
        end
    end

    % rotMatLVLHtoECI = [-sin(w*t), 0, -cos(w*t); 
    %           0,         1,        0; 
    %           cos(w*t),  0, -sin(w*t)];
    % instantThrust = rotMatLVLHtoECI'*instantThrust;
          
    omega = [0, -w, 0];
    
    r = [x(1), x(2), x(3)];
    v = [x(4), x(5), x(6)];
    R = [rT*cos(w*t), 0, -rT*sin(w*t)];
    V = [-rT*w*sin(w*t), 0, -rT*w*cos(w*t)];

    % Fcoriolis = -2*cross(omega, v);
    % Fcentrifugal =  -cross(omega, cross(omega, r));

    Fcoriolis = -2 * cross(omega, V) - cross(omega, cross(omega, r));
    Fcentrifugal = -cross(omega, cross(omega, r)) + cross(omega, V);

    rCenterEarth = [0, 0, rT] - r;
    Fgravity = -mu/norm(rCenterEarth)^3*rCenterEarth;

    xDotDot = Fcoriolis + Fcentrifugal + Fgravity;

    xDot = [v';xDotDot'] + B*instantThrust;
end

