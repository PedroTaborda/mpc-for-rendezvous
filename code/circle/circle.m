
h = 0.05; % Sample time
T = 10; % Simulation time

x0 = [0.8 0 1 0]'; % initial state


N = 15; % Horizon
umaxAbs = 10;
uminAbs = 0;

% circle center and radius
c = [0.5 0.5];
r = 0.4;

%% State space model definition
Ad = [1 h;
     0 1];

A = blkdiag(Ad, Ad);

B = [h*h/2 0     ;
     h     0     ;
     0     h*h/2 ;
     0     h     ];

C = [1 0 0 0;
     0 0 1 0];

D = [0  0  0  0;
     0  0  0  0];

szA = size(A);
szB = size(B);
szC = size(C);

nx = szA(1);
nu = szB(2);
ny = szC(1);

clear szA szB szC


%% Cost and contraints

Q = diag([1, 0.001, 1, 0.001])*1e3;
R = eye(nu);
onCost = 1;

nhat = {[ 1, 0]',...
        [-1, 0]',...
        [0, 1]', ...
        [0, -1]' ...
        [1, 1]'  ...
        [-1, 1]',...
        [1, -1]',...
        [-1, -1]'...
        };
K = 5;
M = length(nhat);
nhatmat = normc(cell2mat(nhat));

umin = uminAbs;
umax = umaxAbs;
clear umaxAbs

%% Solver style

% solver = @mpcSolSLPsemiMICP;
% solver = @mpcSolSLPsemiLC;

% solver = @mpcSolSLP;
solver = @mpcSolExact;

%% Setup plots
hist_fig = 2;
traj_fig = 1;

figure(hist_fig);
clf;

figure(traj_fig);
clf;
hold on;
a = 1.0;
xlim([-0.4*a, 1.1*a]);
ylim([-0.4*a, 1.1*a]);
clear a

%% 
Q = Q/onCost;
R = R/onCost;

%% MPC simulation
t = 0:h:T;
Um= zeros(length(t), length(nhat));
U = zeros(length(t), nu);
X = zeros(length(t), nx);
Y = zeros(length(t), ny);
Xpred = zeros(N, nx);

X(1, :) = x0;
doPlot = false;
u = solver(A, B, umin, umax, x0, N, Q, R, r, c, doPlot, nhat, K);
doPlot = true;
Y(1, :) = C*X(1, :)';
Xpred(1, :) = X(1, :);
for jj = 2:N
    szu = size(u);
    if szu(2) == 2
        uApply = u(1, :)';
        mode = "free";
    else
        uApply = sum(nhatmat.*repmat(u(jj-1, :), 2, 1), 2);
        mode = "locked";
    end
    Xpred(jj, :) = A*Xpred(jj-1, :)' + B*uApply;
end
Ypred = Xpred*C';

trajectory_line = plot(Y(1, 1), Y(1, 2), '-o', 'MarkerSize', 2);
predicted_line = plot(Ypred(:, 1), Ypred(:, 2), '-o', 'MarkerSize', 2);
current_pos_marker = scatter(Y(1, 1), Y(1, 2), 'x');
drawCircle(c, r, {"Color", [62 150 81]/255});

arrow_scaling = 0.005;
u_arrows = quiver(Y(1, 1), Y(1, 2), 0, 0, "off", "color", "#A2142F");

for ii = 2:length(t)
    X(ii, :) = A*X(ii-1, :)' + B*U(ii-1, :)';
%     tic
    u = solver(A, B, umin, umax, X(ii, :)', N, Q, R, r, c, doPlot, nhat, K);
%     toc;

    if mode == "locked"
        Um(ii, :) = u(1, :);
    end
    if any(isnan(u))
        disp('Returned ');
        u
        error('Optimization failed');
    end

    if mode == "free"
        uApply = u(1, :)';
    else
        uApply = sum(nhatmat.*repmat(u(1, :), 2, 1), 2);
    end

    U(ii, :) = uApply;
    Y(ii, :) = C*X(ii, :)';
    Xpred(1, :) = X(ii, :);
    for jj = 2:N
        Xpred(jj, :) = A*Xpred(jj-1, :)' + B*uApply;
    end
    Ypred = Xpred*C';
    title(sprintf("%d/%d", ii, length(t)));
    trajectory_line.XData = Y(1:ii, 1);
    trajectory_line.YData = Y(1:ii, 2);
    predicted_line.XData = Ypred(:, 1);
    predicted_line.YData = Ypred(:, 2);
    current_pos_marker.XData = Y(ii, 1);
    current_pos_marker.YData = Y(ii, 2);
    u_arrows.XData = Y(1:ii, 1);
    u_arrows.YData = Y(1:ii, 2);
    u_arrows.UData = [u_arrows.UData; U(ii, 1)*arrow_scaling];
    u_arrows.VData = [u_arrows.VData; U(ii, 2)*arrow_scaling];
end

if mode == "locked"
    figure(hist_fig);
    clf;
    U_hist = Um(Um ~= 0);
    hold on;
    histogram(U_hist(abs(U_hist)<umin | abs(U_hist) >umax), "FaceColor", "#D95319"); % Actuation does not satisfy constraints
    histogram(U_hist(abs(U_hist)>umin & abs(U_hist) <umax), "FaceColor", "#77AC30"); % Actuation satisfies constraints
    xlabel('input u');
end