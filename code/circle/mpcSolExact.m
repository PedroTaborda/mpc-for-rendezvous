function u_sequence = mpcSolExact(A, B, umin, umax, x0, N, Q, R, radius, center, plot, nhat, K)
    persistent optimizerMPC;
    
    szA = size(A);
    szB = size(B);
    nx=szA(1);
    nu=szB(2);
    if isempty(optimizerMPC)
        if nargin < 9
            radius = 0;
            center = [0, 0];
        end

        u = sdpvar(repmat(nu,1,N),ones(1,N));
        x0_ = sdpvar(nx,1);
        
        E = [1 0 0 0;
            0 0 1 0];
        constraints = [];
        objective = 0;
        x = x0_;
        for k = 1:N
            x = A*x + B*u{k};
            objective = objective + x'*Q*x + u{k}'*R*u{k};
            constraints = [constraints,
            -umax <= u{k} <= umax, (E*x-center')'*(E*x-center') >= radius^2
            ];
        end
        optimizerMPC = optimizer(constraints, objective, [], x0_, u);

    end

    u = optimizerMPC{x0};
    
    u_sequence = zeros(N, nu);
    for k=1:N
        u_sequence(k, :) = value(u{k});
    end
end

