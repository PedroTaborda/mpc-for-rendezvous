function u = mpcCircleToLinearObst(cfg, mpcSolver)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    E = [1, 0, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;];
    debug = 0;

    persistent linesDrawn;

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
    res = mpcSolver(V); u = res{1}; X = res{2};

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
                    % v.x>= v.tangentPoint
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
        res = mpcSolver(V); u = res{1}; X = res{2};
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
        if debug
            fprintf("iter %d/%d, dX=%.2f\n", itercount, maxiter, dX);
        end
    end
    if itercount == maxiter && debug
        fprintf("Max iterations reached (%d) at dX=%.2f>%.2f\n", maxiter, dX, tol);
    end
    if any(newInfeasibles, 'all') && debug
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


