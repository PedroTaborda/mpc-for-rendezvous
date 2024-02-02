% %% Reference frame transformation
% 
% cfg = params();
% w = cfg.system.w;
% dt = cfg.simulation.dt;
% t = 0:dt:dt*1000;
% 
% % origin across time
% 
% Xlvlh = zeros(6, length(t));
% Xeci = zeros(6, length(t));
% 
% Xeci_ = lvlh2eci(cfg, Xlvlh, t(1));
% Xlvlh_ = eci2lvlh(cfg, Xeci, t(1));	
% 
% Xlvlh__ = eci2lvlh(cfg, Xeci_, t(1));
% Xeci__ = lvlh2eci(cfg, Xlvlh_, t(1));
% 
% tol = 1e-6;
% assert(all(abs(Xlvlh - Xlvlh__) < tol, 'all'));
% assert(all(abs(Xeci - Xeci__) < tol, 'all'));
% 
% % perform small "dummy" simulations to get some data
% 
% x0 = [0, 0, 10000, 0, 0, 0]';
% Xcwlvlh = zeros(6, length(t) + 1);
% Xcwlvlh(:, 1) = x0;
% u = zeros(6, 1);
% 
% for i = 2:length(t)+1
%     Xcwlvlh(:, i) = cfg.system.dynamics.fCW(Xcwlvlh(:, i-1), u);
% end
% 
% Xcweci_ = lvlh2eci(cfg, Xcwlvlh, t(1));
% Xcwlvlh__ = eci2lvlh(cfg, Xcweci_, t(1));
% 
% assert(all(abs(Xcwlvlh(:, 1) - Xcwlvlh__(:, 1)) < tol, 'all'));
% 
% figure(11);
% clf;
% ax = subplot(1, 2, 1);
% hold on;
% orbit = zeros(6, length(t));
% orbiteci = lvlh2eci(cfg, orbit, t(1));
% plot(ax, Xcweci_(1, :), Xcweci_(3, :));
% plot(ax, orbiteci(1, :), orbiteci(3, :), '--');
% plot(ax, Xcweci_(1, 1), Xcweci_(3, 1), 'x');
% % plot velocities with quiver
% quiver(ax, Xcweci_(1, :), Xcweci_(3, :), Xcweci_(4, :), Xcweci_(6, :));
% legend('chaser', 'orbit');
% title('ECI');
% axis equal;
% ax = subplot(1, 2, 2);
% hold on;
% plot(ax, Xcwlvlh(1, :), Xcwlvlh(3, :));
% plot(ax, Xcwlvlh__(1, :), Xcwlvlh__(3, :), '--');
% plot(ax, Xcwlvlh(1, 1), Xcwlvlh(3, 1), 'x');
% legend('original', 'transformed');
% title('LVLH');
% set(ax, 'YDir', 'reverse');
% axis equal;
% 
% % perform small "dummy" simulations to get some data
% 
% x0 = [100000, 0, 100000, 0, 0, 0]';
% Xcwlvlh = zeros(6, length(t) + 1);
% Xcwlvlh(:, 1) = x0;
% u = zeros(6, 1);
% 
% for i = 2:length(t)+1
%     Xcwlvlh(:, i) = cfg.system.dynamics.fCW(Xcwlvlh(:, i-1), u);
% end
% 
% Xcweci_ = lvlh2eci(cfg, Xcwlvlh, t(1));
% Xcwlvlh__ = eci2lvlh(cfg, Xcweci_, t(1));
% 
% for ii = 1:6
%     assert(all(abs(Xcwlvlh(:, ii) - Xcwlvlh__(:, ii)) < tol, 'all'));
% end
% 
% figure(12);
% clf;
% ax = subplot(1, 2, 1);
% hold on;
% orbit = zeros(6, length(t));
% orbiteci = lvlh2eci(cfg, orbit, t(1));
% plot(ax, Xcweci_(1, :), Xcweci_(3, :));
% plot(ax, orbiteci(1, :), orbiteci(3, :), '--');
% plot(ax, Xcweci_(1, 1), Xcweci_(3, 1), 'x');
% % plot velocities with quiver
% quiver(ax, Xcweci_(1, :), Xcweci_(3, :), Xcweci_(4, :), Xcweci_(6, :));
% legend('chaser', 'orbit');
% title('ECI');
% axis equal;
% ax = subplot(1, 2, 2);
% hold on;
% plot(ax, Xcwlvlh(1, :), Xcwlvlh(3, :));
% plot(ax, Xcwlvlh__(1, :), Xcwlvlh__(3, :), '--');
% plot(ax, Xcwlvlh(1, 1), Xcwlvlh(3, 1), 'x');
% legend('original', 'transformed');
% title('LVLH');
% set(ax, 'YDir', 'reverse');
% axis equal;
% 
% % ========================================================================
% % ========================================================================
% % ========================================================================

%% Dynamics (Newton vs Clohessy-Wiltshire)


cfg = params();

cfg.controller.N = 20;
cfg.controller.tmin = 1;
cfg.simulation.T = cfg.simulation.dt*600;
cfg.simulation.dt = 10;
cfg.simulation.steps = ceil(cfg.simulation.T/cfg.simulation.dt);

dt = cfg.simulation.dt;
T = cfg.simulation.T;
% x0 = [0, 0, -100000, 1000, 0, 0]';
x0 = [0, 0, 0, 100, 0, 0]';
% x, y, z, x', y', z'

t = 0:dt:T;
XNewtonECI = zeros(length(x0), length(t) + 1);
Xcw = zeros(length(x0), length(t) + 1);
Xcwlin = zeros(length(x0), length(t) + 1);
Xcwexact = zeros(length(x0), length(t) + 1);

w = cfg.system.w;
xcwexact = @(t) (4*x0(4)/w - 6*x0(3)) * sin(w*t) - 2*x0(6) * cos(w*t)/w + (6*w*x0(3)-3*x0(4))*t + x0(1) + 2*x0(6)/w;
ycwexact = @(t) x0(2)*cos(w*t) + x0(5)*sin(w*t)/w;
zcwexact = @(t) (2*x0(4)/w-3*x0(3)) * cos(w*t) + x0(6)*sin(w*t)/w + (4*x0(3)-2*x0(4)/w);
xdotcwexact = @(t) (4*x0(1)/w - 6*x0(3)) * w * cos(w*t) + 2*x0(6) * sin(w*t) + (6*w*x0(3)-3*x0(4));
ydotcwexact = @(t) -x0(2)*w*sin(w*t) + x0(5)*cos(w*t);
zdotcwexact = @(t) -(2*x0(4)/w-3*x0(3)) * w * sin(w*t) + x0(6)*cos(w*t);
Xcwexactfun = @(t) [xcwexact(t); ycwexact(t); zcwexact(t); xdotcwexact(t); ydotcwexact(t); zdotcwexact(t)];

X0 = x0(1);
Y0 = x0(2);
Z0 = x0(3);
Xdot0 = x0(4);
Ydot0 = x0(5);
Zdot0 = x0(6);
XCWexact = @(t) (4*Xdot0/w - 6*Z0) * sin(w*t) - 2*Zdot0 * cos(w*t)/w + (6*w*Z0-3*Xdot0)*t + X0 + 2*Zdot0/w;
YCWexact = @(t) Y0*cos(w*t) + Ydot0*sin(w*t)/w;
ZCWexact = @(t) (2*Xdot0/w-3*Z0) * cos(w*t) + Zdot0*sin(w*t)/w + (4*Z0-2*Xdot0/w);
XdotCWexact = @(t) (4*Xdot0 - 6*w*Z0) * cos(w*t) + 2*Zdot0 * sin(w*t) + (6*w*Z0-3*Xdot0);
YdotCWexact = @(t) -Y0*w*sin(w*t) + Ydot0*cos(w*t);
ZdotCWexact = @(t) (3*Z0*w - 2*Xdot0) * sin(w*t) + Zdot0*cos(w*t);

iterationTime = zeros(1, length(t));
XNewtonECI(:,1) = x0;
Xcw(:,1) = x0;
Xcwlin(:,1) = x0;
Xcwexact(:,1) = x0;

u = zeros(length(x0), 1);
fDynamicscwlin = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);

for i = 2:length(t)+1
    % simulate for one time step
    [tR, yR] = ode45(@(tO, xO) newtonDynamicsECI(tO, xO, t(i-1), u, cfg), [t(i-1), t(i-1)+dt], lvlh2eci(cfg, XNewtonECI(:, i-1), t(i-1)));
    XNewtonECI(:, i) = eci2lvlh(cfg, yR(end, :)', t(i-1)+dt);
    
    Xcw(:, i) = cfg.system.dynamics.fCW(Xcw(:, i-1), u);
    
    Xcwlin(:, i) = fDynamicscwlin(Xcwlin(:, i-1), u);

    Xcwexact(:, i) = Xcwexactfun(t(i-1)+dt);
end

trajectoriesLVLHframe = {XNewtonECI, Xcw, Xcwlin, Xcwexact};
names = {'NewtonECI', 'CW', 'CW linearized', 'CW exact'};
idxsToPlot = [1, 2];

orbit = zeros(6, length(t));
orbiteci = lvlh2eci(cfg, orbit, t(1));

figure(13);
clf;
ax = subplot(3, 2, [1, 3]);
hold on;
for i = idxsToPlot
    trajECI = lvlh2eci(cfg, trajectoriesLVLHframe{i}, t(1));
    plot(ax, trajECI(1, :), trajECI(3, :));
end
for i = idxsToPlot
    trajECI = lvlh2eci(cfg, trajectoriesLVLHframe{i}, t(1));
    quiver(ax, trajECI(1, :), trajECI(3, :), trajECI(4, :), trajECI(6, :));
end
plot(ax, orbiteci(1, :), orbiteci(3, :), '--');
plot(ax, trajECI(1, 1), trajECI(3, 1), 'x');
legend(names{idxsToPlot}, names{idxsToPlot}, 'orbit');
title('ECI');
axis equal;

ax = subplot(3, 2, [2, 4]);
hold on;
for i = idxsToPlot
    plot(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :));
end
for i=idxsToPlot
    quiver(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), trajectoriesLVLHframe{i}(4, :), trajectoriesLVLHframe{i}(6, :));
end
plot(ax, trajectoriesLVLHframe{1}(1, 1), trajectoriesLVLHframe{1}(3, 1), 'x');
legend(names{idxsToPlot}, names{idxsToPlot});
title('LVLH');
set(ax, 'YDir', 'reverse');
set(ax, 'XDir', 'reverse');
axis equal;

% figure(7);
% clf;
% hold on;
% newtonEci = lvlh2eci(cfg, trajectoriesLVLHframe{2}, t(1));
% cwexactEci = lvlh2eci(cfg, trajectoriesLVLHframe{5}, t(1));
% plot(newtonEci(1, 1:end-1) - orbiteci(1, :), newtonEci(3, 1:end-1) - orbiteci(3, :));
% % plot(newtonEci(1, 2:end) - orbiteci(1, :), newtonEci(3, 2:end) - orbiteci(3, :));
% % plot(newtonEci(1, :) - cwexactEci(1, :), newtonEci(3, :) - cwexactEci(3, :));
% title('pc-pt');

% rotMat = [sin(w*t(2)), 0, -cos(w*t(2)); 
%           0,         1,        0; 
%           cos(w*t(2)),  0, sin(w*t(2))];

