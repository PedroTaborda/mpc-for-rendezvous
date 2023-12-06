function u_sequence = mpcSolSLPsemiLC(A, B, umin, umax, x0, N, Q, R, radius, center, doPlot, nhat, K)
    if nargin < 9
        radius = 0;
        center = [0, 0];
    end
    szA = size(A);
    szB = size(B);
    nx=szA(1);
    nu=szB(2);

    M = length(nhat);
    nhatmat = normc(cell2mat(nhat));
    u = sdpvar(N, M);
    sigma = sdpvar(N, M);
    gamma = sdpvar(N, M);
    x0_ = sdpvar(nx,1);

    baseConstraints = [u >= 0, 0 <= gamma <= 1];
    objective = 0;
    x = x0;
    for k = 1:N
        for m = 1:M
            baseConstraints = [baseConstraints,
                gamma(k, m)*umin <= sigma(k, m) <= gamma(k, m)*umax,
                u(k, m) <= sigma(k, m)
            ];
        end
        baseConstraints = [baseConstraints,
            sum(gamma(k, :)) <= K
        ];
        uk = sum(nhatmat.*repmat(u(k, :), 2, 1), 2);
        x = A*x + B*uk;
        objective = objective + x'*Q*x + uk'*R*uk + sum(gamma(k, :));
        
    end
    optimize([baseConstraints, x0_ == x0], objective);
    
    X = zeros(N, nx);
    X(1, :) = x0;
    inCircle = zeros(N, 1);
    for jj = 2:N
        uj_1 = sum(nhatmat.*repmat(value(u(jj-1, :)), 2, 1), 2);
        X(jj, :) = A*X(jj-1, :)' + B*value(uj_1);
        p = X(jj, [1, 3]);
        if ((p-center)'*(p-center) < radius^2)
            inCircle(jj) = 1;
        end
    end
    u_sequence = zeros(N, nu);
    for k=1:N
        u_sequence(k, :) = sum(nhatmat.*repmat(value(u(k, :)), 2, 1), 2);
    end

    lastu = inf*ones(size(u_sequence));
    tol = 0.01;
    niter = 4;
    iter = 1;
    if any(inCircle)
        E = [1 0 0 0;
             0 0 1 0];
        lines = cell(N, 1);
        while norm(lastu - u_sequence) > tol && iter <= niter
            extendedConstraints = baseConstraints;
            x = x0;
            for k = 1:N
                uk = sum(nhatmat.*repmat(u(k, :), 2, 1), 2);
                x = A*x + B*uk;

                if inCircle(k)
                    p = X(k, [1, 3]);
                    v = (p-center)/norm(p-center);
                    tangentPoint = center + radius*v;
                    extendedConstraints = [extendedConstraints, dot(v, E*x) >= dot(v, tangentPoint)];
                    tangentLinePoints = [tangentPoint - [v(2), -v(1)]; tangentPoint + [v(2), -v(1)]];
                    if doPlot
                        lines{k} = plot(tangentLinePoints(:, 1), tangentLinePoints(:, 2), 'Color', [0.3010 0.7450 0.9330, 0.5]);
                    end
                end
            end
            optimize([extendedConstraints, x0_ == x0], objective);
            for k=1:N
                lastu(k, :) = sum(nhatmat.*repmat(value(u(k, :)), 2, 1), 2);
            end
            for jj = 2:N
                X(jj, :) = A*X(jj-1, :)' + B*lastu(k, :)';
            end
            iter = iter + 1;
            if doPlot
                for k = 1:N
                    if inCircle(k)
                        delete(lines{k});
                    end
                end
            end
        end
        u_sequence = lastu;

    end
    u_sequence = value(u);
end

