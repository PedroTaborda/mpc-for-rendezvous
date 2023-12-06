h = 0.025; % Sample time
T = 10; % Simulation time

x0 = [0.8 0 1 0]'; % initial state

N = 15; % Horizon
umaxAbs = 5;

% circle center and radius
c = [0.5 0.5];
r = 0.4;

%% State space model definition
Ad = [1 h;
     0 1];

A = blkdiag(Ad, Ad); % state: x, xdot, y, ydot

B = [h*h/2 0     ;
     h     0     ;
     0     h*h/2 ;
     0     h     ];

%% Cost and contraints

Q = diag([1, 0.00001, 1, 0.00001])*1e5;
R = eye(2);

umax = umaxAbs;
clear umaxAbs

%% Solver style (clear persistent variables

clear mpcSolExact mpcSolSLP

solver1 = @mpcSolExact;
solver2 = @mpcSolSLP;

%% MPC simulation

figure(1);
clf; hold on; grid on; 
xlim([-0.4 1.03]); ylim([-0.25 1.03]);
xlabel("x"); ylabel("y");
drawCircle(c, r, {"Color", [62 150 81]/255});
[XExact, UExact, tRecordedExact] = recordAndShowFinal(solver1, A, B, N, h, Q, R, r, c, umax, x0, T);
plot(XExact(:, 1), XExact(:, 3), "color", "#0072BD")
plot(XExact(1, 1), XExact(1, 3), "o", "color", 	"#A2142F")
plot(XExact(end, 1), XExact(end, 3), "x", "color", "#A2142F")


[XSLP, USLP, tRecordedSLP] = recordAndShowFinal(solver2, A, B, N, h, Q, R, r, c, umax, x0, T);
plot(XSLP(:, 1), XSLP(:, 3), "color", "#D95319")
plot(XSLP(1, 1), XSLP(1, 3), "o", "color", 	"#A2142F")
plot(XSLP(end, 1), XSLP(end, 3), "x", "color", "#A2142F")
legend("Obstacle", "fmincon", "start", "end", "proposed", "", "", 'Location','southeast');

xlabel("x [m]"); ylabel("y [m]");
xlim([-0.25, 1.5]);
axis equal;
setFigSize(gcf, 11, 7.5);
stylingAndSaveAsPdf(gcf, "obst_trajectories.pdf");

fprintf("fmincon: %f\n", sum(tRecordedExact));
fprintf("proposed: %f\n", sum(tRecordedSLP));
fprintf("(after first) fmincon: %f\n", sum(tRecordedExact(3: end)));
fprintf("(after first) proposed: %f\n", sum(tRecordedSLP(3: end)));

figure(3);
clf; hold on; grid on;
plot(tRecordedExact, "color", "#0072BD");
plot(tRecordedSLP, "color", "#D95319");
legend("fmincon", "proposed");
xlabel("Iteration"); ylabel("Time [s]");

setFigSize(gcf, 11, 7.5);
stylingAndSaveAsPdf(gcf, "obst_times.pdf");

figure(4);
clf; hold on; grid on;
plot(UExact(:, 1), "color", "#0072BD");
plot(UExact(:, 2), "color", "#0072BD");
plot(USLP(:, 1), "color", "#D95319");
plot(USLP(:, 2), "color", "#D95319");
legend("fmincon u_x", "fmincon u_y", "proposed u_x", "proposed u_y");

setFigSize(gcf, 11, 7.5);
stylingAndSaveAsPdf(gcf, "obst_actuation.pdf");

function [X, U, tRecorded] = recordAndShowFinal(solver, A, B, N, h, Q, R, r, c, umax, x0, T)

    szA = size(A);
    szB = size(B);

    nx = szA(1);
    nu = szB(2);

    t = 0:h:T;
    tRecorded = zeros(length(t), 1);
    X = zeros(length(t), nx);
    X(1, :) = x0';
    U = zeros(length(t)-1, nu);
    for ii = 2:length(t)
        tic;
        Umpc = solver(A, B, [], umax, X(ii-1, :)', N, Q, R, r, c, [], [], []);
        tRecorded(ii) = toc;
        u = Umpc(1, :);
        if any(isnan(u))
            warning("Optimization failed, u is NaN");
        end
        X(ii, :) = (A*X(ii-1, :)' + B*u')';
        U(ii-1, :) = u;
    end

end
