function u = mpcStandard(cfg, xk, xref)
    % MPC controller
    % xk: current state
    % xref: reference trajectory
    
    % M: number of thrusters
    % K: number of simultaneous thrusts
    % N: prediction horizon
    % Q: state cost matrix
    % R: control cost matrix
    % tmin: minimum thrust duration
    % tmax: maximum thrust duration
    % returns: control input (array of size M with thrust duration for each
    % motor)

    persistent optimizerMpc;
    if isempty(optimizerMpc)
        optimizerMpc = setupMpc(cfg);
    end

    % get the optimal control input
    u = optimizerMpc(xk, xref);
end

function opt = setupMpc(cfg)
%% SETUPMPC Creates an optimizer object for repeated use

    % unpack parameters
    N = cfg.controller.N;
    M = cfg.controller.M;
    K = cfg.controller.K;
    Q = cfg.controller.Q;
    R = cfg.controller.R;
    tmin = cfg.controller.tmin;
    tmax = min([cfg.controller.tmax, cfg.simulation.dt]);
    fDynamics = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);

    % initialization
    S = sdpvar(M, N, 'full'); % control input
    X = sdpvar(6, N+1, 'full'); % state
    o = binvar(M, N, 'full'); % binary variables (thruster on/off)
    xRef = sdpvar(6, 1); % final state reference
    x0 = sdpvar(6, 1); % initial state

    % objective function and constraints
    obj = 0;
    con = [];
    for k = 1:N
        con = [con, X(:,k+1) == fDynamics(X(:,k), S(:,k))];
        con = [con, 0 <= S(:,k) <= tmax];
        for i = 1:M
            con = [con, implies(o(i,k) == 1, tmin <= S(i,k) <= tmax)];
            con = [con, implies(o(i,k) == 0, S(i,k) == 0)];
            obj = obj + S(i, k);
        end
        if ~cfg.controller.XfOnly
            obj = obj + (X(:,k+1) - xRef(:))'*Q*(X(:,k+1) - xRef(:));
        end
    end
    if cfg.controller.XfOnly
        obj = obj + N*(X(:,k+1) - xRef(:))'*Q*(X(:,k+1) - xRef(:));
    end
    con = [con, X(:, 1) == x0];
    % con = [con, x0 == xk, xRef == xref];

    % options
    ops = sdpsettings('verbose', 0, 'solver', 'gurobi');

    % create optimizer object
    opt = optimizer(con, obj, ops, {x0, xRef}, S);

end
