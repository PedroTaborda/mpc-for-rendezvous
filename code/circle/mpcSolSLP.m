function u_sequence = mpcSolSLP(A, B, umin, umax, x0, N, Q, R, radius, center, doPlot, nhat, K)
    persistent optimizerMPCunconstrained;
    persistent optimizerMPClinearConstrained;
    if nargin < 9
        radius = 0;
        center = [0, 0];
    end
    szA = size(A);
    szB = size(B);
    nx=szA(1);
    nu=szB(2);        
    E = [1 0 0 0;
         0 0 1 0];

    if (isempty(optimizerMPCunconstrained) || isempty(optimizerMPClinearConstrained))
        u = sdpvar(repmat(nu,1,N),ones(1,N));
        X = sdpvar(repmat(nx,1,N+1),ones(1,N+1));
        V = sdpvar(3, N);

        baseConstraints = [];
        objective = 0;
        for k = 1:N

            baseConstraints = [baseConstraints, X{k+1} == A*X{k} + B*u{k}, -umax <= u{k} <= umax ];
            objective = objective + X{k+1}'*Q*X{k+1} + u{k}'*R*u{k};
        end
        optimizerMPCunconstrained = optimizer(baseConstraints, objective, sdpsettings('solver', 'gurobi'), X{1}, u);

        extendedConstraints = baseConstraints;
        % prepare the tangent line constraints, dependent on the current state at each time step
        E = [1 0 0 0;
             0 0 1 0];
        x = x0;
        for k = 1:N
            extendedConstraints = [extendedConstraints, 
                dot(V(1:2, k), E*X{k+1} ) >= V(3, k)
            ];
        end
        % we want to supply the initial state and constraints vector V to the optimizer
        optimizerMPClinearConstrained = optimizer(extendedConstraints, objective, sdpsettings('solver', 'gurobi'), {X{1}, V}, u);
    end

    % solve the unconstrained problem
    u = optimizerMPCunconstrained{x0};

    infeasible = false;
    x = x0;
    infeasibleSet = [];
    for k = 1:N
        x = A*x + B*u{k};
        p = (E*x)';
        if (norm((p-center)) <= radius)
            infeasible = true;
            infeasibleSet = [infeasibleSet, k];
        end
    end
    u_sequence = zeros(N, nu);
    for k=1:N
        u_sequence(k, :) = value(u{k});
    end

    lastu = inf*ones(size(cell2mat(u)));
    tol = 0.1;
    tolCur = 2*tol;
    maxIter = 5;
    iter = 1;
    if infeasible
        V = zeros(3, N);
        while tolCur > tol && iter <= maxIter
            x = x0;
            for k = 1:N
                x = A*x + B*u{k};
                p = (E*x)';
                if ismember(k, infeasibleSet)
                    v = (p-center)/norm(p-center);
                    tangentPoint = center + radius*v;
                    % fill V with the tangent line constraints
                    V(:, k) = [v(1); v(2); dot(v, tangentPoint)];
                else
                    if (norm((p-center)) <= radius)
                        v = (p-center)/norm(p-center);
                        tangentPoint = center + radius*v;
                        % fill V with the tangent line constraints
                        V(:, k) = [v(1); v(2); dot(v, tangentPoint)];
                        infeasibleSet = [infeasibleSet, k];
                    else
                        % no constraint
                        V(:, k) = [0; 0; 0];
                    end
                end
            end
            u = optimizerMPClinearConstrained{x0, V};
            tolCur = norm(lastu - cell2mat(u), "fro");
            iter = iter + 1;
            lastu = cell2mat(u);
        end
        u_sequence = cell2mat(u);

    end

    x = x0;
    infeasible = false;
    for k = 1:N
        x = A*x + B*u{k};
        p = (E*x)';
        if (norm((p-center)) <= radius)
            infeasible = true;
        end
    end
    u_sequence = cell2mat(u)';
end

