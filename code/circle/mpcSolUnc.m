function u_sequence = mpcSolUnc(A, B, umin, umax, x0, N, Q, R, radius, center, plot)
    if nargin < 9
        radius = 0;
        center = [0, 0];
    end
    szA = size(A);
    szB = size(B);
    nx=szA(1);
    nu=szB(2);

    u = sdpvar(repmat(nu,1,N),ones(1,N));
    x0_ = sdpvar(nx,1);
    
    constraints = [];
    objective = 0;
    x = x0;
    for k = 1:N
        x = A*x + B*u{k};
        objective = objective + norm(Q*x,1) + norm(R*u{k},1);
        constraints = [constraints, umin <= u{k}, u{k} <= umax];
    end
    
    optimize([constraints, x0_ == x0], objective);
    
    u_sequence = zeros(N, nu);
    for k=1:N
        u_sequence(k, :) = value(u{k});
    end
end

