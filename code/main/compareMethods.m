cfg = params();
cfg.controller.N = 10; % 5 10 15
cfg.simulation.steps = 100;
cfg.simulation.dt = 10;
cfg.simulation.T = cfg.simulation.steps * cfg.simulation.dt;
cfg.controller.tmin = 1/4*cfg.simulation.dt;

cfg.simulation.method = 'standard';
[X, S, T] = simulate(cfg, [], 0);
doPlots(cfg, X, S, 0);


%%

% cfg.simulation.method = 'projected';
% [Xp, Sp] = simulate(cfg, [], 1);
% doPlots(cfg, Xp, Sp, 10);

cfg.simulation.method = 'relaxed';
[Xl, Sl] = simulate(cfg, [], 1);
doPlots(cfg, Xl, Sl, 20);

figure(5);
clf;

% Energy spent (time open - for now)
Etotal = sum(energySpent(S, cfg));
Etotalp = sum(energySpent(Sp, cfg));

ax1 = subplot(2,1,1);
imagesc(S);
xlabel('time step');
ylabel('thruster');
title(sprintf('standard (%d seconds open)', round(Etotal)));
colorbar;

ax2 = subplot(2,1,2);
imagesc(Sp);
xlabel('time step');
ylabel('thruster');
title(sprintf('projected (%d seconds open)', round(Etotalp)));
colorbar;

% ax3 = subplot(3,1,3);
% imagesc(Sl);
% xlabel('time step');
% ylabel('thruster');
% title('linearized');
% colorbar;

linkaxes([ax1,ax2],'xy');

%% Comparison LVLH
% Trajectories side by side
figure(6);
clf;
plot(X(1, :), X(3, :));
hold on;
plot(Xp(1, :), Xp(3, :));
xlabel("x");
ylabel("z");
title("Trajectory LVLH (m)");
legend('standard', 'linear constraints');

% Trajectory difference
figure(7);
clf;
plot3(Xp(1, :)-X(1, :), Xp(2, :)-X(2, :), Xp(3, :)-X(3, :));
xlabel("x");
ylabel("y");
zlabel("z");
title("Error LVLH (m)");
legend('standard', 'linear constraints');

