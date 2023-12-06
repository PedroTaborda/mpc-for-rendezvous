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


    % unpack parameters
    N = cfg.controller.N;
    M = cfg.controller.M;
    tmin = cfg.controller.tmin;

    % solve free
    u = optimizerMpc{xk, xref, zeros(M, N), zeros(M, N)};

    % add tmin constraints if unfeasible
    [conOn, conOff] = addTminConstraints(tmin, M, N, u);
    isfeasible = isempty(conOn) && isempty(conOff);
    while true && ~isfeasible
        u = optimizerMpc{xk, xref, conOn, conOff};
        [conOnUpdated, conOffUpdated] = addTminConstraints(tmin, M, N, u);
        isfeasible = isempty(conOnUpdated) && isempty(conOffUpdated);
        if isfeasible || ~nnz(conOnUpdated-conOn)
            break;
        end
        conOn = conOn | conOnUpdated;
        conOff = conOff | conOffUpdated;
    end
    
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

    % objective function and constraints
    obj = 0;
    con = [];
    for k = 1:N
        con = [con, X(:,k+1) == fDynamics(X(:,k), S(:,k))];
        % con = [con, 0 <= S(:,k) <= tmax];
        for m = 1:M
            % if conOn(m, k) == 1, then S(m, k) >= tmin
            % if conOn(m, k) == 0, then S(m, k) >= 0
            con = [con, S(m, k) >= 0 + conOn(m, k)*tmin]; 
            % if conOff(m, k) == 1, then S(m, k) <= 0 => S(m, k) == 0
            % if conOff(m, k) == 0, then S(m, k) <= tmax
            con = [con, S(m, k) <= tmax*(1 - conOff(m, k))];
        end
        if cfg.controller.XfOnly
            obj = obj + S(:,k)'*R*S(:,k);
        else
            obj = obj + S(:,k)'*R*S(:,k) + (X(:,k+1) - xRef(:))'*Q*(X(:,k+1) - xRef(:));
        end
    end
    if cfg.controller.XfOnly
        obj = obj + N*(X(:,k+1) - xRef(:))'*Q*(X(:,k+1) - xRef(:));
    end
    con = [con, X(:, 1) == x0];
    % con = [con, x0 == xk, xRef == xref];

    ops = sdpsettings('verbose', 0);

    opt = optimizer(con, obj, ops, {x0, xRef, conOn, conOff}, {S});

end
