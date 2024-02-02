function u = mpcRelaxed(cfg, xk, xref)
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
        u = relaxConstraints(cfg, optimizerMpc{xk, xref});
        return;
    end
    optimizerMpcObstacles = @(V) relaxConstraints(cfg, optimizerMpc{xk, xref, V});
    u = mpcCircleToLinearObst(cfg, optimizerMpcObstacles);
end

function mpcRes = relaxConstraints(cfg, mpcRes)
    % unpack parameters
    M = cfg.controller.M;
    N = cfg.controller.N;
    tmin = cfg.controller.tmin;
    tmax = cfg.controller.tmax;
    modeOnlyActuation = true;

    if isa(mpcRes, 'cell')
        u=mpcRes{1};
        x=mpcRes{2};
        modeOnlyActuation = false;
    else
        u=mpcRes;
    end
    
    % check if any u, if nonzero, is less than tmin or greater than tmax
    for i = 1:M
        for k = 1:N
            if abs(u(i,k)) < 1e-10
                continue
            else
                if u(i,k) < tmin
                    % if nearer to tmin, set to tmin, if nearer to 0, set to 0
                    if u(i,k) < tmin/2
%                         fprintf('u(%d,%d) = %e < tmin = %e (set to 0)\n', i, k, u(i,k), tmin);
                        u(i,k) = 0;
                    else
%                         fprintf('u(%d,%d) = %e < tmin = %e (set to tmin)\n', i, k, u(i,k), tmin);
                        u(i,k) = tmin;
                    end
                elseif u(i,k) > tmax
%                     fprintf('u(%d,%d) = %e > tmax = %e\n', i, k, u(i,k), tmax);
                    u(i,k) = tmax;
                end
            end
        end
    end
    
    if modeOnlyActuation
        mpcRes=u;
        return
    end
    fDynamics = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);
    for k = 1:N
        x(:,k+1) = fDynamics(x(:,k), u(:,k));
    end
    mpcRes{1} = u;
    mpcRes{2} = x;
end
function opt = setupMpc(cfg)

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
    X = sdpvar(6, N+1, 'full'); % state
    o = sdpvar(M, N, 'full'); % "binary" variables (thruster on/off)
    sigma = sdpvar(M, N, 'full'); % slack variable
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
            con = [con, dot(V(obs, 1:2, k), E*X(:, k+1) ) >= V(obs, 3, k)];
        end
        for i = 1:M
            con = [con, 0 <= o(i,k) <= 1, o(i,k)*tmin <= sigma(i,k) <= o(i,k)*tmax];
            con = [con, 0 <= S(i,k) <= sigma(i,k)];
        end
        % con = [con, sum(o(:,k)) <= K];
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

    % set up the optimizer
    ops = sdpsettings('verbose', 0, 'solver', 'gurobi');
    % create optimizer object
    if numel(cfg.simulation.obstacles) > 0
        opt = optimizer(con, obj, ops, {x0, xRef, V}, {S, X});
    else
        opt = optimizer(con, obj, ops, {x0, xRef}, S);
    end
end