function paramsStruct = params()
    %PARAMS Returns parameters list for system, controller and simulation
    
    %% orbit and system parameters
    G = 6.674e-11;                  % N m^2 / kg^2
    M = 5.972e24;                   % kg
    RE = 6371e3;                     % m
    mt = 3000;                      % kg
    mc = 2000;                      % kg
    altTarget = 800e3;              % m
    W = [... % direction of each thruster
        0, 0, 1; 
        0, 0, -1;
        0, 1, 0; 
        0, -1, 0;
        1, 0, 0; 
        -1, 0, 0;
    ];
    F = 1000*[... % max thrust of each thruster
        1;
        1;
        1;
        1;
        1;
        1;
    ];

    %% simulation parameters
    method = 'standard'; % uses linearized dynamics
    % method = 'projected' % uses linearized dynamics
    % method = 'relaxed' % uses linearized dynamics
    alt0 = 700e3;                   % m
    dt = 10;                        % s
    T = 3600;                       % simulation time
    verbosity = 2;                  % 0: no output, >0: waitbar on simulation
                                    % is also passed to yalmip solver

    %% MPC controller parameters 
    N = 10;                         % prediction horizon
    Q = diag([1, 1, 1, 1, 1, 1]);   % state cost
    R = diag([1, 1, 1, 1, 1, 1]);   % input cost
    tmin = 5;                    % min time for each thruster
    tmax = 0;                      % max time for each thruster

    
    %% derived parameters and functions
    w = sqrt(G*M/(RE+altTarget)^3);  % rad/s
    mu = G*M;                       % m^3 / s^2
    % the dynamics of each thruster for a duration s is encoded in the matrix G(s)
    %dynG = @(s) integral(@(T) expm(Ac*T), 0, s, "ArrayValued", true);
%     dynG = @(s) [ ...
%         s, 0         , 3*s^2*w+6*(cos(w*s)-1)/w, -3*s^2/2-4*(cos(w*s)-1)/w^2, 0               , 2*s/w-2*sin(w*s)/w^2 ;...
%         0, sin(w*s)/w, 0                       , 0                          , (1-cos(w*s))/w^2, 0                    ;...
%         0, 0         , 4*s-3*sin(w*s)/w        , 2*sin(w*s)/w^2-2*s/w       , 0               , (1-cos(w*s))/w^2     ;...
%         0, 0         , 6*w*s-6*sin(w*s)        , 4*sin(w*s)/w-3*s           , 0               , 2*(1-cos(w*s))/w     ;...
%         0, cos(w*s)-1, 0                       , 0                          , sin(w*s)/w      , 0                    ;...
%         0, 0         , 3*(1-cos(w*s))          , 2*(cos(w*s)-1)/w           , 0               , sin(w*s)/w           ;...
%     ];
    %dynG = @(s) integral(@(T) expm(-Ac*T), 0, s, "ArrayValued", true);
    dynG = @(s) [ ...
        s, 0         , -3*s^2*w+6*(1-cos(w*s))/w, 3*s^2/2+4*(cos(w*s)-1)/w^2, 0               , 2*s/w-2*sin(w*s)/w^2 ;...
        0, sin(w*s)/w, 0                        , 0                          , (cos(w*s)-1)/w^2, 0                    ;...
        0, 0         , 4*s-3*sin(w*s)/w         , 2*sin(w*s)/w^2-2*s/w       , 0               , (cos(w*s)-1)/w^2     ;...
        0, 0         , 6*w*s-6*sin(w*s)         , 4*sin(w*s)/w-3*s           , 0               , 2*(cos(w*s)-1)/w     ;...
        0, 1-cos(w*s), 0                        , 0                          , sin(w*s)/w      , 0                    ;...
        0, 0         , 3*(cos(w*s)-1)           , 2*(1-cos(w*s))/w           , 0               , sin(w*s)/w           ;...
    ];

    AcCW = [
        zeros(3, 3), eye(3);
        diag([0, -w*w, 3*w*w]), fliplr(diag([2*w, 0, -2*w]));
    ];

    B = [
        zeros(3, 3);
        eye(3)/mc
    ];

    Ad = expm(AcCW*dt);
    dynamicsActuators = @(x, tThrusters) Ad*sumInline(@(idx) dynG(tThrusters(idx))*B*F(idx)*W(idx, :)', 1:length(F));
%     dGdt = @(tlin) [ ...
%         1,              0, 6*tlin*w - 6*sin(tlin*w), (4*sin(tlin*w))/w - 3*tlin,             0, 2/w - (2*cos(tlin*w))/w;
%         0,    cos(tlin*w),                        0,                          0, sin(tlin*w)/w,                       0;
%         0,              0,        4 - 3*cos(tlin*w),    (2*cos(tlin*w))/w - 2/w,             0,           sin(tlin*w)/w;
%         0,              0,    6*w - 6*w*cos(tlin*w),          4*cos(tlin*w) - 3,             0,           2*sin(tlin*w);
%         0, -w*sin(tlin*w),                        0,                          0,   cos(tlin*w),                       0;
%         0,              0,          3*w*sin(tlin*w),             -2*sin(tlin*w),             0,             cos(tlin*w);
%     ];
    dGds = @(s0) [ ...
        1,              0, -6*s0*w + 6*sin(s0*w), -(4*sin(s0*w))/w + 3*s0,              0, 2/w - (2*cos(s0*w))/w;
        0,    cos(s0*w),                         0,                           0, -sin(s0*w)/w,                       0;
        0,              0,         4 - 3*cos(s0*w),     (2*cos(s0*w))/w - 2/w,              0,          -sin(s0*w)/w;
        0,              0,     6*w - 6*w*cos(s0*w),           4*cos(s0*w) - 3,              0,           2*sin(s0*w);
        0,  w*sin(s0*w),                         0,                           0,    cos(s0*w),                       0;
        0,              0,          -3*w*sin(s0*w),               2*sin(s0*w),              0,             cos(s0*w);
    ];
    dynamicsActuatorsLin = @(x, tThrusters) Ad * sumInline(@(idx) tThrusters(idx)*B*F(idx)*W(idx, :)', 1:length(F));

    dynamicsActuatorsLinT = @(x, tThrusters, s0) Ad * sumInline(@(idx) (dynG(s0)+dGds(s0)*(tThrusters(idx)-s0))*B*F(idx)*W(idx, :)', 1:length(F));

%     dynamicsActuatorsLinT = @(x, tThrusters, s0) Ad * sumInline(@(idx) (dynG(s0)+dGds(s0)*(tThrusters(idx)-s0))*(double(tThrusters(idx)>s0))*B*F(idx)*W(idx, :)', 1:length(F));

    fCW = @(x, tThrusters) Ad*x + dynamicsActuators(x, tThrusters);

    fLin = @(x, tThrusters) Ad*x + dynamicsActuatorsLin(x, tThrusters);

    fLinT = @(x, tThrusters, tlin) Ad*x + dynamicsActuatorsLinT(x, tThrusters, tlin);

    paramsStruct.system = struct(...
        'G', G, ...                          % gravitational constant
        'M', M, ...                          % mass of Earth
        'R', RE, ...                         % radius of Earth
        'mt', mt,...                         % mass of target
        'mc', mc,...                         % mass of chaser
        'mu', mu,...                         % gravitational parameter
        'altTarget', altTarget, ...          % altitude of target orbit
        'rT', RE+altTarget, ...              % radius of target orbit
        'w', w, ...                          % angular velocity of target orbit
        'W', W, ...                          % direction of each thruster
        'F', F, ...                          % max thrust of each thruster
        'dynamics', struct(...
            'G', dynG, ...                   % dynamics of each thruster
            'AcCW', AcCW, ...                % CW dynamics - continuous time
            'AdCW', expm(AcCW*dt), ...       % CW dynamics - discrete time
            'B', B, ...                      % input matrix
            'fCW', fCW, ...                  % CW dynamics - discrete time
            'fLin', fLin,...                 % linear dynamics - discrete time
            'fLinT', fLinT...                % linear dynamics at arbitrary point
        )...
    );



    paramsStruct.simulation = struct(...
        'alt0', alt0, ...                   % initial altitude
        'x0', [0; 
               0; 
               altTarget-alt0; 
               0; 
               0; 
               0], ...                      % initial state
        'xTarget', [0; 0; 0; 0; 0; 0], ...  % target state
        'dt', dt, ...                       % time step
        'steps', ceil(T/dt), ...            % number of steps
        'T', T, ...                         % total simulation time
        'method', method, ...               % method for solving the optimization problem
        'verbosity', verbosity, ...         % verbosity level
        'rerun', 0 ...                      % rerun same simulation multiple times
         ...                                % by setting this number to
         ...                                % different values, to ensure
         ,...                               % cache miss
        'obstacles', [] ...                 % cell list of structs with fields
                        ...                 % 'center' and 'radius'
    );

    paramsStruct.controller = struct(...
        'N', N, ...                        % horizon length
        'M', length(F), ...                % number of thrusters
        'K', length(F), ...                % maximum number of simultaneously active thrusters
        'Q', Q, ...                        % state cost matrix
        'R', R, ...                        % input cost matrix
        'XfOnly', true, ...                % whether cost function weighs all states or only the final one
        'tmin', tmin, ...                  % min thrust time
        'tmax', tmax, ...                  % max thrust time
        'tlin', dt/2 ...                      % value at which to linearize dynamics
    );

end

