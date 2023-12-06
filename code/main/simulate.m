function [X, U, iterationTime] = simulate(cfg, usrFcn, overrideCache)
%SIMULATE Simulates the controlled system with the given configuration
%   Outputs the state of the system at each time step
%   If provided, usrFcn is called at each time step with the current state and
%   control input as arguments

global cached_simulations; % is a containers.Map

% unpack configuration (all fixes to cfg such as tmax must be performed
% before trying to access cache)
dt = cfg.simulation.dt;
T = cfg.simulation.T;
x0 = cfg.simulation.x0;
xref = cfg.simulation.xTarget;
method = cfg.simulation.method;
verbosity = cfg.simulation.verbosity;
if cfg.controller.tmax == 0
    cfg.controller.tmax = dt;
end

% check if simulation is cached
[X, U, iterationTime] = getCachedSimulation(cfg);

if nargin < 3 || isempty(overrideCache)
    overrideCache = false;
end

if ~isempty(X)
    if ~overrideCache
%         disp('Simulation is cached. To run a new simulation, delete the cache folder.');
        return;
    else
        disp('Simulation is cached. Overriding cache...');
    end
end

if nargin < 2 || isempty(usrFcn)
    usrFcn = @(X, U) 0;
end


% initialization 
if ceil(T/dt) ~= cfg.simulation.steps
    fprintf("int(T/dt)=%d!=%d", round(T/dt), cfg.simulation.steps)
    error("Bad config");
end
t = 0:dt:T;
X = zeros(length(x0), length(t) + 1);
U = zeros(length(x0), length(t));
iterationTime = zeros(1, length(t));

X(:, 1) = x0;

if verbosity > 0
    % waitbar
    h = waitbar(0, sprintf('Simulating (%s, N=%d)...', method, cfg.controller.N));
else
    h = [];
end


cleanupObj = onCleanup(@() cleanup(verbosity, h));

% simulate
try
    for i = 2:length(t) + 1
        % get control input, while measuring time
        tic;
        if strcmp(method, 'standard')
            Umpc = mpcStandard(cfg, X(:, i-1), xref);
        elseif strcmp(method, 'standard-o')
            Umpc = mpcStandardObstacles(cfg, X(:, i-1), xref);
        elseif strcmp(method, 'projected')
            Umpc = mpcProjected(cfg, X(:, i-1), xref);
        elseif strcmp(method, 'relaxed')
            Umpc = mpcRelaxed(cfg, X(:, i-1), xref);
        else
            % show valid methods
            error('Invalid method. Valid methods are ''standard'', ''projected'', ''relaxed''.');
        end
        iterationTime(i-1) = toc;
%         Umpc = zeros(length(x0), cfg.controller.N);
%         u = zeros(length(x0), 1);
        u = Umpc(:, 1);
        U(:, i-1) = u;
        
        % simulate for one time step
%         [tR, yR] = ode45(@(tO, xO) newtonDynamics(tO, xO, t(i-1), u, cfg), [t(i-1), t(i-1)+dt], X(:, i-1));
%         X(:, i) = yR(end, :)';
        [tR, yR] = ode45(@(tO, xO) newtonDynamicsECI(tO, xO, t(i-1), u, cfg), [t(i-1), t(i-1)+dt], lvlh2eci(cfg, X(:, i-1), t(i-1)));
        X(:, i) = eci2lvlh(cfg, yR(end, :)', t(i-1)+dt);
    
        % testing only - simulate dynamics using CW
%         X(:, i) = cfg.system.dynamics.fCW(X(:, i-1), u);
%         fDynamics = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);
%         X(:, i) = fDynamics(X(:, i-1), u);
    
        % call user function
        usrFcn(X(:, 1:i), Umpc);

        if verbosity > 0
            % update waitbar
            waitbar((i-1) / length(t), h, sprintf('Simulating (%s, N=%d)... %d/%d', method, cfg.controller.N, i-1, length(t)));
        end
        
    end
catch exception % make sure ctrl+c is polled and function is exited
    rethrow(exception);
end

% cache simulation
cacheSimulation(cfg, X, U, iterationTime, overrideCache);

end
function cleanup(verbosity, h)
    if verbosity > 0
        % close waitbar
        close(h);
    end
    clear mpcStandard mpcProjected mpcRelaxed mpcStandardObstacles;
end

function [X, U, iterationTime] = getCachedSimulation(cfgNew)
%GETCACHEDSIMULATION Returns the cached simulation if it exists, otherwise
%   returns an empty matrix

    dirname = 'cache';
    if ~exist(dirname, 'dir')
        mkdir(dirname);
    end

    % master cache file has all the cfgs and their corresponding cache files in
    % a struct, with fields cfg and file
    master_cache_file = sprintf('%s/master_cache.mat', dirname);

    global cached_simulations; % is a containers.Map
    if isempty(cached_simulations)
        % if master cache file doesn't exist, create it
        if ~exist(master_cache_file, 'file')
            renew_mcf();
        end
        load(master_cache_file, 'cached_simulations');
    end

    % check if simulation is cached
    cfgKey = cfgRepr(cfgNew);

    if isKey(cached_simulations, cfgKey)
        load(cached_simulations(cfgKey).file, 'X', 'U', 'iterationTime');
        return;
    end

    X = [];
    U = [];
    iterationTime = [];
end

function cacheSimulation(cfgNew, X, U, iterationTime, overrideCache)
%CACHESIMULATION Caches the simulation

    dir = 'cache';
    if ~exist(dir, 'dir')
        mkdir(dir);
    end
    cfgKey = cfgRepr(cfgNew);
    keyNum = mod(sum(cfgKey) + randi([1 1000000]),1000000);
    name = @(i) sprintf('%s/%d_%d.mat', dir, keyNum, i);
    i = 1;
    while exist(name(i), 'file')
        if overrideCache
            load (name(i), 'cfg');
            if isequal(cfgData(cfgNew), cfgData(cfg))
                break;
            end
        end
        i = i + 1;
    end
    cfg = cfgNew;
    save(name(i), 'X', 'U', 'iterationTime', 'cfg');
    fprintf('Simulation cached to %s\n', name(i) );

    % save to master cache file
    master_cache_file = sprintf('%s/master_cache.mat', dir);
    global cached_simulations; % is a containers.Map
    
    cached_simulations(cfgKey) = struct('cfg', cfg, 'file', name(i));
    save(master_cache_file, 'cached_simulations');
end
