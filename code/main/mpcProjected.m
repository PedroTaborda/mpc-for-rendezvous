function u = mpcProjected(cfg, xk, xref)
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
    if numel(cfg.simulation.obstacles) == 0
        % get the optimal control input
        res = mpcProjectedSolve(cfg, xk, xref, optimizerMpc, {}); u = res{1};
        return;
    end
    optimizerMpcObstacles = @(V) mpcProjectedSolve(cfg, xk, xref, optimizerMpc, {V});
    u = mpcCircleToLinearObst(cfg, optimizerMpcObstacles);

end

function res = mpcProjectedSolve(cfg, xk, xref, persOptimizerMpc, argOpt)
    % unpack parameters
    N = cfg.controller.N;
    M = cfg.controller.M;
    tmin = cfg.controller.tmin;

    % solve free
    res = persOptimizerMpc{xk, xref, zeros(M, N), zeros(M, N), argOpt{:}}; u=res{1};x=res{2};

    % add tmin constraints if unfeasible
    [conOn, conOff] = addTminConstraints(tmin, M, N, u);
    isfeasible = isempty(conOn) && isempty(conOff);
    while true && ~isfeasible
        res = persOptimizerMpc{xk, xref, conOn, conOff, argOpt{:}};u=res{1};x=res{2};
        [conOnUpdated, conOffUpdated] = addTminConstraints(tmin, M, N, u);
        isfeasible = isempty(conOnUpdated) && isempty(conOffUpdated);
        if isfeasible || ~nnz(conOnUpdated-conOn)
            break;
        end
        conOn = conOn | conOnUpdated;
        conOff = conOff | conOffUpdated;
    end
    fDynamics = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);
    for k = 1:N
        x(:,k+1) = fDynamics(x(:,k), u(:,k));
    end
    mpcRes{1} = u;
    mpcRes{2} = x;

    res{1} = u;
    res{2} = x;
end

function [conOn, conOff] = addTminConstraints(tmin, M, N, S)
    % add tmin constraints to unfeasible points
    % con: constraints
    % tmin: minimum thrust duration
    % M: number of thrusters
    % N: prediction horizon
    % returns: new constraints with tmin constraints added, or empty if no
    % unfeasible points

    hasUnfeasiblePoint = false;
    conOn = zeros(M, N); % control input constraints
    conOff = zeros(M, N); % control input constraints

    % check if any point is unfeasible
    for k = 1:1
        for m = 1:M
            if S(m, k) < tmin && S(m, k) > 0
                hasUnfeasiblePoint = true;
                if S(m, k) < tmin/2
                    conOff(m, k) = 1;
                else
                    conOn(m, k) = 1;
                end
            end
        end
    end

    if ~hasUnfeasiblePoint
        conOn = [];
        conOff = [];
        return
    end
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
    tmax = cfg.controller.tmax;
    fDynamics = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);

    % initialization
    S = sdpvar(M, N, 'full'); % control input
    conOn = sdpvar(M, N, 'full'); % control input constraints
    conOff = sdpvar(M, N, 'full'); % control input constraints
    X = sdpvar(6, N+1, 'full'); % state
    xRef = sdpvar(6, 1); % final state reference
    x0 = sdpvar(6, 1); % initial state

    if numel(cfg.simulation.obstacles) > 0
        V = sdpvar(numel(cfg.simulation.obstacles), 3, N); % circle linear constraints
    end
    E = [1, 0, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;];

    % objective function and constraints
    obj = 0;
    con = [];
    for k = 1:N
        con = [con, X(:,k+1) == fDynamics(X(:,k), S(:,k))];
        % con = [con, 0 <= S(:,k) <= tmax];
        for obs = 1:numel(cfg.simulation.obstacles)
%            con = [con, dot(V(obs, 1:2, k), E*X(:, k+1) ) + 0.1 >= V(obs, 3, k)];
            con = [con, 0.1 >= V(obs, 3, k)];
        end
        for m = 1:M
            % if conOn(m, k) == 1, then S(m, k) >= tmin
            % if conOn(m, k) == 0, then S(m, k) >= 0
            con = [con, S(m, k) >= 0 + conOn(m, k)*tmin]; 
            % if conOff(m, k) == 1, then S(m, k) <= 0 => S(m, k) == 0
            % if conOff(m, k) == 0, then S(m, k) <= tmax
            con = [con, S(m, k) <= tmax*(1 - conOff(m, k))];
            obj = obj + S(m, k);
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

    ops = sdpsettings('verbose', 0, 'solver', 'gurobi');

    if numel(cfg.simulation.obstacles) > 0
        opt = optimizer(con, obj, ops, {x0, xRef, conOn, conOff, V}, {S, X});
    else
        opt = optimizer(con, obj, ops, {x0, xRef, conOn, conOff}, {S, X});
    end

end
