function u = mpcStandardObstacles(cfg, xk, xref)
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
    
    E = [1, 0, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;];

    debug = 0;

    persistent optimizerMpc;
    persistent linesDrawn;

    if isempty(optimizerMpc)
        optimizerMpc = setupMpc(cfg);
    end
    
    if numel(cfg.simulation.obstacles) == 0
        % get the optimal control input
        u = optimizerMpc(xk, xref);
        return;
    end

    
    if debug
        if isempty(linesDrawn)
            linesDrawn = [];
        else
            for i = 1:numel(linesDrawn)
                delete(linesDrawn(i));
            end
        end
    end

    V = zeros(numel(cfg.simulation.obstacles), 3, cfg.controller.N);

    % get the optimal control input
    res = optimizerMpc(xk, xref, V); u = res{1}; X = res{2};

    infset = infeasibleSet(cfg, X); 
    if ~any(infset)
        return;
    end
    itercount = 0;
    maxiter = 5;
    tol = 1;
    dX = tol+1;
    radFactorPer10= 0.05*0;
    adjustmentRadFactor = @(k) 1 + radFactorPer10*k/10;
    newInfeasibles = [];
    while (dX > tol || any(newInfeasibles, 'all')) && itercount < maxiter
        for k = 1:cfg.controller.N
            for obs = 1:numel(cfg.simulation.obstacles)
                obstacle = cfg.simulation.obstacles{obs};
                obsRadius = adjustmentRadFactor(k)*obstacle.radius;

                p = E*X(:, k+1);
                if infset(obs, k) || (norm((p-obstacle.center)) <= obsRadius)
                    v = (p-obstacle.center)/norm(p-obstacle.center);
                    tangentPoint = obstacle.center + obsRadius*v;
                    V(obs, :, k) = [v(1); v(2); dot(v, tangentPoint)];
                    infset(obs, k) = 1;
                else
                    % no constraint
                    V(obs, :, k) = [0; 0; 0];
                end
            end
        end
        Xprev = X;

        if debug
            for i = 1:numel(linesDrawn)
                delete(linesDrawn(i));
            end
            for k = 1:cfg.controller.N
                for obs = 1:numel(cfg.simulation.obstacles)
                    obstacle = cfg.simulation.obstacles{obs};
                    v = V(obs, 1:2, k);
                    if any(v)
                        oC = obstacle.center;
                        oR = obstacle.radius*adjustmentRadFactor(k);
                        line = plot([oC(1), oC(1)+oR*v(1)], [oC(2), oC(2)+oR*v(2)], 'k');
                        linesDrawn = [linesDrawn, line];
                        fprintf("Line drawn for obstacle %d in (%.0f, %.0f)\n", obs, obstacle.center(1), obstacle.center(2));
                    end
                end
            end
            N = cfg.controller.N;
            n = length(cfg.simulation.x0);
            predictedStates = zeros(n, N+1);
            predictedStates(:, 1) = xk;
            fDynamics = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);
            for ii = 1:N
                predictedStates(:, ii+1) = fDynamics(predictedStates(:, ii), u(:, ii));
    %             predictedStates(:, ii+1) = fDynamics(predictedStates(:, ii), [0, 0, 0, 0, 10, 0]);
            end
            % the x, z position coordinates are the first and third rows of the state
            % plot each point (not connected by lines) and wait for the user to press a key
            linePredicted = plot(predictedStates(1, :), predictedStates(3, :), 'x', 'Linewidth', 1.2, 'Color', [0.8500, 0.3250, 0.0980]);
            % plot arrows for the control input 
            for ii = 1:N
                inputSum = @(horizonStep) sumInline(@(thruster) u(thruster, horizonStep)*cfg.system.W(thruster, :)*cfg.system.F(thruster), 1:cfg.controller.M);
                inputSumI = inputSum(ii);
                inputSumI = [inputSumI(1); inputSumI(3)]*0.1;
                arrow = quiver(predictedStates(1, ii), predictedStates(3, ii), inputSumI(1), inputSumI(2), 'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.1);

                linesDrawn = [linesDrawn, arrow];
            end
        end
        res = optimizerMpc(xk, xref, V); u = res{1}; X = res{2};
        % if ~any(u, 'all')
        %     warning('No control input - check if the problem is feasible');
        % end
        newInfeasibles = infeasibleSet(cfg, X);
        
        if debug
            delete(linePredicted);
        end
        
        for k = 1:cfg.controller.N
            for obs = 1:numel(cfg.simulation.obstacles)
                infset(obs, k) = infset(obs, k) || newInfeasibles(obs, k);
            end
        end

%         if ~any(newInfeasibles)
%             disp('No more infeasibles')
%         end
        itercount = itercount + 1;
        dX = norm(X - Xprev);
%         if itercount > maxiter
%             break;
%         end
        if debug
            fprintf("iter %d/%d, dX=%.2f\n", itercount, maxiter, dX);
        end
    end
    if itercount == maxiter
        fprintf("Max iterations reached (%d) at dX=%.2f>%.2f\n", maxiter, dX, tol);
    end
    if any(newInfeasibles, 'all')
        warning('Still infeasible (%d states x obstacles) after %d iterations, dX=%.2f', sum(newInfeasibles, 'all'), itercount, dX);
    end
end

function infset = infeasibleSet(cfg, X)
    
    E = [1, 0, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;];
    N = cfg.controller.N;
    infset = zeros(numel(cfg.simulation.obstacles), N);

    radFactorPer10= 0.05*0;
    adjustmentRadFactor = @(k) 1 + radFactorPer10*k/10;

    for k = 1:N
        p = E*X(:, k+1);
%         fprintf("%.2f, %.2f\n", p(1), p(2));
        for obs = 1:numel(cfg.simulation.obstacles)
            obstacle = cfg.simulation.obstacles{obs};
            if (norm((p-obstacle.center)) <= obstacle.radius*adjustmentRadFactor(k))
                infset(obs, k) = 1;
            end
        end
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
    tmax = min([cfg.controller.tmax, cfg.simulation.dt]);
    fDynamics = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);

    % initialization
    S = sdpvar(M, N, 'full'); % control input
    X = sdpvar(6, N+1, 'full'); % state
    o = binvar(M, N, 'full'); % binary variables (thruster on/off)
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
        con = [con, 0 <= S(:,k), S(:,k) <= tmax];
        for obs = 1:numel(cfg.simulation.obstacles)
            con = [con, dot(V(obs, 1:2, k), E*X(:, k+1) ) >= V(obs, 3, k)];
        end
        for i = 1:M
            con = [con, implies(o(i,k) == 1, tmin <= S(i,k))];
            con = [con, implies(o(i,k) == 0, S(i,k) == 0)];
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

    % options
    ops = sdpsettings('verbose', 0, 'solver', 'gurobi');

    % create optimizer object
    if numel(cfg.simulation.obstacles) > 0
        opt = optimizer(con, obj, ops, {x0, xRef, V}, {S, X});
    else
        opt = optimizer(con, obj, ops, {x0, xRef}, S);
    end

    

end
