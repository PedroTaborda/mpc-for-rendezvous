%% Graphics settings and ensure image directory is available
plotOptions = {'Linewidth', 1.2};


dirname = 'imgs';
if ~exist(dirname, 'dir')
    mkdir(dirname);
end
subdirname = [dirname, '/', 'validation'];
if ~exist(subdirname, 'dir')
    mkdir(subdirname);
end
relpath = @(filename) [subdirname, '/', filename];

%% Experiment 1, free motion

cfg = params();

cfg.simulation.T = cfg.simulation.dt*1000;
cfg.simulation.steps = ceil(cfg.simulation.T/cfg.simulation.dt);

dt = cfg.simulation.dt;
T = cfg.simulation.T;
% x0 = [0, 0, -100000, 1000, 0, 0]';
x0 = cfg.simulation.x0;
% x, y, z, x', y', z'

t = 0:dt:T;
XNewtonECI = zeros(length(x0), length(t) + 1);
Xcwcont = zeros(length(x0), length(t) + 1);
Xcwdiscrete = zeros(length(x0), length(t) + 1);
Xcwlin = zeros(length(x0), length(t) + 1);

w = cfg.system.w;

iterationTime = zeros(1, length(t));
XNewtonECI(:,1) = x0;
Xcwcont(:,1) = x0;
Xcwdiscrete(:,1) = x0;
Xcwlin(:,1) = x0;

% S returns a logical array indicating whether each actuator is on at time t
S = @(t) [
    0;
    0;
    0;
    0;
    0;
    0;
];

u = zeros(length(x0), 1);
fDynamicscwlin = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);

for i = 2:length(t)+1
    % simulate for one time step
    u = S(t(i-1))*dt;

    [tR, yR] = ode45(@(tO, xO) newtonDynamicsECI(tO, xO, t(i-1), u, cfg), [t(i-1), t(i-1)+dt], lvlh2eci(cfg, XNewtonECI(:, i-1), t(i-1)));
    XNewtonECI(:, i) = eci2lvlh(cfg, yR(end, :)', t(i-1)+dt);

    [tR, yR] = ode45(@(tO, xO) cfg.system.dynamics.AcCW*xO+cfg.system.dynamics.B*thrusterTimes2Force(cfg, u, tO-t(i-1)), [t(i-1), t(i-1)+dt], Xcwcont(:, i-1));
    Xcwcont(:, i) = yR(end, :)';
    
    Xcwdiscrete(:, i) = cfg.system.dynamics.fCW(Xcwdiscrete(:, i-1), u);
    
    Xcwlin(:, i) = fDynamicscwlin(Xcwlin(:, i-1), u);

end

trajectoriesLVLHframe = {XNewtonECI, Xcwcont, Xcwdiscrete, Xcwlin};
names = {'full dynamics', 'continuous CW', 'discrete CW', 'linearized CW'};
idxsToPlot = [1, 2, 3, 4];

orbit = zeros(6, length(t));
orbiteci = lvlh2eci(cfg, orbit, t(1));

figure(11);
clf;
ax = gca;
hold on;
for i = idxsToPlot
    plot(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), plotOptions{:});
end
% for i=idxsToPlot
%     quiver(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), trajectoriesLVLHframe{i}(4, :), trajectoriesLVLHframe{i}(6, :));
% end
plot(ax, trajectoriesLVLHframe{1}(1, 1), trajectoriesLVLHframe{1}(3, 1), 'x', 'Color', 'black');
% legend(names{idxsToPlot}, names{idxsToPlot});
legend(names{idxsToPlot}, 'start', 'Location', 'best');
% title('LVLH');
% set(ax, 'YDir', 'reverse');
% set(ax, 'YAxisLocation','right');
% set(ax, 'XAxisLocation','top');
% set(ax, 'XDir', 'reverse');
xlabel('x (m)');
ylabel('z (m)');

setFigSize(gcf, 10, 7);
validationFigStylingAndSave(gcf, relpath('exp1-traj-lvlh.pdf'));

figure(12);
clf;
ax = gca;
hold on;
for i = idxsToPlot
    trajECI = lvlh2eci(cfg, trajectoriesLVLHframe{i}, t(1));
    plot(ax, trajECI (1, :), trajECI(3, :), plotOptions{:});
end
% for i=idxsToPlot
%     quiver(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), trajectoriesLVLHframe{i}(4, :), trajectoriesLVLHframe{i}(6, :));
% end
plot(ax, trajECI(1, 1), trajECI(3, 1), 'x');
% legend(names{idxsToPlot}, names{idxsToPlot});
legend(names{idxsToPlot}, 'start', 'Location', 'best');
axis equal;
xlabel('x_{ECI} (m)');
ylabel('z_{ECI} (m)');

setFigSize(gcf, 10, 7);
validationFigStylingAndSave(gcf, relpath('exp1-traj-eci.pdf'));


figure(13);
clf;
ax = gca;
hold on;
for i = idxsToPlot(2:end)
    errorLVLH = trajectoriesLVLHframe{i}(1:3, :) - trajectoriesLVLHframe{1}(1:3, :);
    ax.ColorOrderIndex = i;
    errorLVLHnorm = vecnorm(errorLVLH(:, 2:end));
    plot(ax, t, errorLVLHnorm, plotOptions{:});
end
% for i = idxsToPlot
%     trajECI = lvlh2eci(cfg, trajectoriesLVLHframe{i}, t(1));
%     quiver(ax, trajECI(1, :), trajECI(3, :), trajECI(4, :), trajECI(6, :));
% end
% legend(names{idxsToPlot}, names{idxsToPlot}, 'target orbit');
legend(names{idxsToPlot(2:end)}, 'Location', 'best');
% title('ECI');
xlabel('Simulation Time (s)');
ylabel('Error (m)');

setFigSize(gcf, 8, 6);
validationFigStylingAndSave(gcf, relpath('exp1-err-all.pdf'));

figure(14);
clf;
ax = gca;
hold on;
for i = idxsToPlot(3:end)
    errorLVLH = trajectoriesLVLHframe{i}(1:3, :) - trajectoriesLVLHframe{2}(1:3, :);
    ax.ColorOrderIndex = i;
    errorLVLHnorm = vecnorm(errorLVLH(:, 2:end));
    plot(ax, t, errorLVLHnorm, plotOptions{:});
end
% for i = idxsToPlot
%     trajECI = lvlh2eci(cfg, trajectoriesLVLHframe{i}, t(1));
%     quiver(ax, trajECI(1, :), trajECI(3, :), trajECI(4, :), trajECI(6, :));
% end
% legend(names{idxsToPlot}, names{idxsToPlot}, 'target orbit');
legend(names{idxsToPlot(3:end)}, 'Location', 'best');
% title('ECI');
xlabel('Simulation Time (s)');
ylabel('Error (m)');
setFigSize(gcf, 8, 6);
validationFigStylingAndSave(gcf, relpath('exp1-err-cw.pdf'));

figure(15);
clf;
ax = gca;
hold on;
fullDynamicsDistanceTravelled = cumsum(vecnorm(diff(trajectoriesLVLHframe{1}(1:3, :), 1, 2)));
for i = idxsToPlot(2:end)
    errorLVLH = trajectoriesLVLHframe{i}(1:3, :) - trajectoriesLVLHframe{1}(1:3, :);
    ax.ColorOrderIndex = i;
    % error over distance traveled
    errorLVLHnormrel = vecnorm(errorLVLH(:, 2:end))./fullDynamicsDistanceTravelled*100;
    plot(ax, t, errorLVLHnormrel, plotOptions{:});
end

legend(names{idxsToPlot(2:end)}, 'Location', 'best');
% title('ECI');
xlabel('Simulation Time (s)');
ylabel('Relative Error (%)');

setFigSize(gcf, 8, 6);
validationFigStylingAndSave(gcf, relpath('exp1-err-rel-all.pdf'));

fdynamics = trajectoriesLVLHframe{1}(1:3, :);
cwdiscrete = trajectoriesLVLHframe{3}(1:3, :);
% some table data
keyTimeSteps = [5, 10, 15, 50, 100, 1000];
tabDistTravelled = fullDynamicsDistanceTravelled(keyTimeSteps);
tabAbsError = vecnorm(fdynamics(:, keyTimeSteps) - cwdiscrete(:, keyTimeSteps));
tabRelError = tabAbsError./tabDistTravelled*100;

for ii = 1:length(keyTimeSteps)
    fprintf('\\SI{%d}{\\second} & \\SI{%.2f}{\\meter} & \\SI{%.2f}{\\meter} & \\SI{%.2f}{\\percent} \\\\ \n', keyTimeSteps(ii)*dt, tabDistTravelled(ii), tabAbsError(ii), tabRelError(ii));
end

%% Experiment 2, actuated

cfg = params();

cfg.simulation.T = cfg.simulation.dt*100;
cfg.simulation.steps = ceil(cfg.simulation.T/cfg.simulation.dt);

dt = cfg.simulation.dt;
T = cfg.simulation.T;
cfg.simulation.x0 = [0, 0, 1000, 0, 0, 0]';
x0 = cfg.simulation.x0;
% x, y, z, x', y', z'

t = 0:dt:T;
XNewtonECI = zeros(length(x0), length(t) + 1);
Xcwcont = zeros(length(x0), length(t) + 1);
Xcwdiscrete = zeros(length(x0), length(t) + 1);
Xcwlin = zeros(length(x0), length(t) + 1);
U = zeros(6, length(t));

w = cfg.system.w;

iterationTime = zeros(1, length(t));
XNewtonECI(:,1) = x0;
Xcwcont(:,1) = x0;
Xcwdiscrete(:,1) = x0;
Xcwlin(:,1) = x0;

H = @(xval) double(xval >= 0);

S1 = @(t) [
    H(t - 0.75*T);
    H(t - 0.25*T) - H(t - 0.5*T);
    0;
    0;
    H(t - 0.25*T) - H(t - 0.5*T);
    H(t - 0.75*T);
];

S2 = @(t) [
    H(t - 0.25*T) - H(t - 0.5*T);
    H(t - 0.75*T);
    0;
    0;
    H(t - 0.75*T);
    H(t - 0.25*T) - H(t - 0.5*T);
];

S3 = @(t) [
    0;
    1;
    0;
    0;
    0;
    1;
];

S4 = @(t) [
    0;
    0;
    0;
    0;
    1;
    0;
];
% S4 = @(t) [
%     0;
%     0;
%     0;
%     0;
%     0;
%     0;
% ];


Schoices = {S1, S2, S3, S4};

% Schoices = {S4};

for ss = 1:length(Schoices)
    fprintf('S %d \n', ss);
    S = Schoices{ss};

    u = zeros(length(x0), 1);
    fDynamicscwlin = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, 5);
    
    for i = 2:length(t)+1
        % simulate for one time step
        u = S(t(i-1))*dt;
        U(:, i-1) = u;
    
        [tR, yR] = ode45(@(tO, xO) newtonDynamicsECI(tO, xO, t(i-1), u, cfg), [t(i-1), t(i-1)+dt], lvlh2eci(cfg, XNewtonECI(:, i-1), t(i-1)));
        XNewtonECI(:, i) = eci2lvlh(cfg, yR(end, :)', t(i-1)+dt);
    
        [tR, yR] = ode45(@(tO, xO) cfg.system.dynamics.AcCW*xO+cfg.system.dynamics.B*sumInline(@(idx) cfg.system.F(idx)*cfg.system.W(idx, :)'*double(u(idx)>0), 1:length(u)), [t(i-1), t(i-1)+dt], Xcwcont(:, i-1));
        Xcwcont(:, i) = yR(end, :)';
        
        Xcwdiscrete(:, i) = cfg.system.dynamics.fCW(Xcwdiscrete(:, i-1), u);
        
        Xcwlin(:, i) = fDynamicscwlin(Xcwlin(:, i-1), u);
    
    end
    
    trajectoriesLVLHframe = {XNewtonECI, Xcwcont, Xcwdiscrete, Xcwlin};
    names = {'full dynamics', 'continuous CW', 'discrete CW', 'linearized CW'};
    idxsToPlot = [1, 2, 3, 4];
    
    orbit = zeros(6, length(t));
    orbiteci = lvlh2eci(cfg, orbit, t(1));

    figure(201 + (ss-1)*4);
    clf;
    ax = gca;
    hold on;
    for i = idxsToPlot
        plot(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), plotOptions{:});
    end
    % for i=idxsToPlot
    %     quiver(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), trajectoriesLVLHframe{i}(4, :), trajectoriesLVLHframe{i}(6, :));
    % end
    plot(ax, trajectoriesLVLHframe{1}(1, 1), trajectoriesLVLHframe{1}(3, 1), 'x', 'Color', 'black');
    % legend(names{idxsToPlot}, names{idxsToPlot});
    legend(names{idxsToPlot}, 'start', 'Location', 'best');
    % title('LVLH');
    set(ax, 'YDir', 'reverse');
    set(ax, 'XDir', 'reverse');
%     set(ax, 'YAxisLocation','right');
%     set(ax, 'XAxisLocation','top');
    xlabel('x (m)');
    ylabel('z (m)');
    
    setFigSize(gcf, 13, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp2-%d-traj-lvlh.pdf', ss)));
    
    figure(202 + (ss-1)*4);
    clf;
    ax = gca;
    % control_signal = results{actual_idxs(ii)}.S([1, 2, 5, 6], :);
    im = imagesc([0, cfg.simulation.T], [1,4], U([1, 2, 5, 6], :));
    xlabel('Simulation Time (s)');
%     set(ax, 'XAxisLocation','top');
    ylabel('Thruster');
    arrayfun(@(yy) line([0, cfg.simulation.T], [yy, yy], 'Color', 'black', 'LineWidth', 1), [0.5, 1.5, 2.5, 3.5, 4.5]);
    thrusternames = {'Down', 'Up', 'Left', 'Right'};
    thrusterticks = 1:4;
    set(ax,'ytick',thrusterticks,'yticklabel',thrusternames)
    
    setFigSize(gcf, 5, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp2-%d-control.pdf', ss)));
    
    
    figure(203 + (ss-1)*4);
    clf;
    ax = gca;
    hold on;
    for i = idxsToPlot(2:end)
        errorLVLH = trajectoriesLVLHframe{i}(1:3, :) - trajectoriesLVLHframe{1}(1:3, :);
        ax.ColorOrderIndex = i;
        errorLVLHnorm = vecnorm(errorLVLH(:, 2:end));
        plot(ax, t, errorLVLHnorm, plotOptions{:});
    end
    % for i = idxsToPlot
    %     trajECI = lvlh2eci(cfg, trajectoriesLVLHframe{i}, t(1));
    %     quiver(ax, trajECI(1, :), trajECI(3, :), trajECI(4, :), trajECI(6, :));
    % end
    % legend(names{idxsToPlot}, names{idxsToPlot}, 'target orbit');
    legend(names{idxsToPlot(2:end)}, 'Location', 'best');
    % title('ECI');
    xlabel('Simulation Time (s)');
    ylabel('Error (m)');
    
    setFigSize(gcf, 8, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp2-%d-err.pdf', ss)));
    
    figure(204 + (ss-1)*4);
    clf;
    ax = gca;
    hold on;
    fullDynamicsDistanceTravelled = cumsum(vecnorm(diff(trajectoriesLVLHframe{1}(1:3, :), 1, 2)));
    for i = idxsToPlot(2:end)
        errorLVLH = trajectoriesLVLHframe{i}(1:3, :) - trajectoriesLVLHframe{1}(1:3, :);
        ax.ColorOrderIndex = i;
        % error over distance traveled
        errorLVLHnormrel = vecnorm(errorLVLH(:, 2:end))./fullDynamicsDistanceTravelled*100;
        plot(ax, t, errorLVLHnormrel, plotOptions{:});
    end
    
    legend(names{idxsToPlot(2:end)}, 'Location', 'best');
    % title('ECI');
    xlabel('Simulation Time (s)');
    ylabel('Relative Error (%)');
    
    setFigSize(gcf, 8, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp2-%d-err-rel.pdf', ss)));
    
    fdynamics = trajectoriesLVLHframe{1}(1:3, :);
    cwdiscrete = trajectoriesLVLHframe{3}(1:3, :);
    % some table data
%     keyTimeSteps = [5, 10, 15, 50, 100];
%     tabDistTravelled = fullDynamicsDistanceTravelled(keyTimeSteps);
%     tabAbsError = vecnorm(fdynamics(:, keyTimeSteps) - cwdiscrete(:, keyTimeSteps));
%     tabRelError = tabAbsError./tabDistTravelled*100;
%     
%     for ii = 1:length(keyTimeSteps)
%         fprintf('\\SI{%d}{\\second} & \\SI{%.2f}{\\meter} & \\SI{%.2f}{\\meter} & \\SI{%.2f}{\\percent} \\\\ \n', keyTimeSteps(ii)*dt, tabDistTravelled(ii), tabAbsError(ii), tabRelError(ii));
%     end
end


%% aux
for ss=1:4
    figure(201 + (ss-1)*4);
    setFigSize(gcf, 13, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp2-%d-traj-lvlh.pdf', ss)));
        
    figure(202 + (ss-1)*4);
    setFigSize(gcf, 5, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp2-%d-control.pdf', ss)));
    
    figure(203 + (ss-1)*4);
    setFigSize(gcf, 8, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp2-%d-err.pdf', ss)));
    
    figure(204 + (ss-1)*4);
    setFigSize(gcf, 8, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp2-%d-err-rel.pdf', ss)));
end

%% Experiment 3 part 1, linearization point

cfg = params();

cfg.simulation.T = cfg.simulation.dt*100;
cfg.simulation.steps = ceil(cfg.simulation.T/cfg.simulation.dt);

dt = cfg.simulation.dt;
T = cfg.simulation.T;
cfg.simulation.x0 = [0, 0, 1000, 0, 0, 0]';
x0 = cfg.simulation.x0;
% x, y, z, x', y', z'

t = 0:dt:T;
XNewtonECI = zeros(length(x0), length(t) + 1);
Xcwcont = zeros(length(x0), length(t) + 1);
Xcwdiscrete = zeros(length(x0), length(t) + 1);
Xcwlin1 = zeros(length(x0), length(t) + 1);
Xcwlin2 = zeros(length(x0), length(t) + 1);
Xcwlin3 = zeros(length(x0), length(t) + 1);
Xcwlin4 = zeros(length(x0), length(t) + 1);
Xcwlin5 = zeros(length(x0), length(t) + 1);
U = zeros(6, length(t));

w = cfg.system.w;

iterationTime = zeros(1, length(t));
XNewtonECI(:,1) = x0;
Xcwcont(:,1) = x0;
Xcwdiscrete(:,1) = x0;
Xcwlin1(:,1) = x0;
Xcwlin2(:,1) = x0;
Xcwlin3(:,1) = x0;
Xcwlin4(:,1) = x0;
Xcwlin5(:,1) = x0;

H = @(xval) double(xval >= 0);
S = @(t) [
    H(t - 0.75*T);
    H(t - 0.25*T) - H(t - 0.5*T);
    0;
    0;
    H(t - 0.25*T) - H(t - 0.5*T);
    H(t - 0.75*T);
];


tlinchoices = [0, 2.5, 5, 7.5, 10];

u = zeros(length(x0), 1);
fDynamicscwlin1 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(1));
fDynamicscwlin2 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(2));
fDynamicscwlin3 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(3));
fDynamicscwlin4 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(4));
fDynamicscwlin5 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(5));

for i = 2:length(t)+1
    % simulate for one time step
    u = S(t(i-1))*dt;
    U(:, i-1) = u;

    [tR, yR] = ode45(@(tO, xO) newtonDynamicsECI(tO, xO, t(i-1), u, cfg), [t(i-1), t(i-1)+dt], lvlh2eci(cfg, XNewtonECI(:, i-1), t(i-1)));
    XNewtonECI(:, i) = eci2lvlh(cfg, yR(end, :)', t(i-1)+dt);

%     [tR, yR] = ode45(@(tO, xO) cfg.system.dynamics.AcCW*xO+cfg.system.dynamics.B*sumInline(@(idx) cfg.system.F(idx)*cfg.system.W(idx, :)'*double(u(idx)>0), 1:length(u)), [t(i-1), t(i-1)+dt], Xcwcont(:, i-1));
    [tR, yR] = ode45(@(tO, xO) cfg.system.dynamics.AcCW*xO+cfg.system.dynamics.B*sumInline(@(idx) cfg.system.F(idx)*cfg.system.W(idx, :)'*double(u(idx)-(tO - t(i-1))>0), 1:length(u)), [t(i-1), t(i-1)+dt], Xcwcont(:, i-1));
    Xcwcont(:, i) = yR(end, :)';
    
    Xcwdiscrete(:, i) = cfg.system.dynamics.fCW(Xcwdiscrete(:, i-1), u);
    
    Xcwlin1(:, i) = fDynamicscwlin1(Xcwlin1(:, i-1), u);
    Xcwlin2(:, i) = fDynamicscwlin2(Xcwlin2(:, i-1), u);
    Xcwlin3(:, i) = fDynamicscwlin3(Xcwlin3(:, i-1), u);
    Xcwlin4(:, i) = fDynamicscwlin4(Xcwlin4(:, i-1), u);
    Xcwlin5(:, i) = fDynamicscwlin5(Xcwlin5(:, i-1), u);

end
    
trajectoriesLVLHframe = {XNewtonECI, Xcwcont, Xcwdiscrete, Xcwlin1, Xcwlin2, Xcwlin3, Xcwlin4, Xcwlin5};
names = {
    'full dynamics', ...
    'continuous CW', ...
    'discrete CW', ...
    sprintf('s_0=%.1f', tlinchoices(1)), ...
    sprintf('s_0=%.1f', tlinchoices(2)), ...
    sprintf('s_0=%.1f', tlinchoices(3)), ...
    sprintf('s_0=%.1f', tlinchoices(4)), ...
    sprintf('s_0=%.1f', tlinchoices(5))
   };
idxsToPlot = [1, 3, 4, 5, 6, 7, 8];

orbit = zeros(6, length(t));
orbiteci = lvlh2eci(cfg, orbit, t(1));

figure(31);
clf;
ax = gca;
hold on;
for i = idxsToPlot
    plot(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), plotOptions{:});
end
% for i=idxsToPlot
%     quiver(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), trajectoriesLVLHframe{i}(4, :), trajectoriesLVLHframe{i}(6, :));
% end
plot(ax, trajectoriesLVLHframe{1}(1, 1), trajectoriesLVLHframe{1}(3, 1), 'x', 'Color', 'black');
% legend(names{idxsToPlot}, names{idxsToPlot});
legend(names{idxsToPlot}, 'start', 'Location', 'best');
% title('LVLH');
% set(ax, 'YAxisLocation','right');
% set(ax, 'XAxisLocation','top');
set(ax, 'YDir', 'reverse');
set(ax, 'XDir', 'reverse');
xlabel('x (m)');
ylabel('z (m)');

setFigSize(gcf, 8, 6);
validationFigStylingAndSave(gcf, relpath('exp3-1-traj-lvlh.pdf'));

figure(32);
clf;
ax = gca;
im = imagesc([0, cfg.simulation.T], [1,4], U([1, 2, 5, 6], :));
xlabel('Simulation Time (s)');
ylabel('Thruster');
arrayfun(@(yy) line([0, cfg.simulation.T], [yy, yy], 'Color', 'black', 'LineWidth', 1), [1.5, 2.5, 3.5]);
thrusternames = {'Down', 'Up', 'Left', 'Right'};
thrusterticks = 1:4;
set(ax,'ytick',thrusterticks,'yticklabel',thrusternames)

setFigSize(gcf, 8, 5);
validationFigStylingAndSave(gcf, relpath('exp3-1-control.pdf'));
    
figure(33);
clf;
ax = gca;
hold on;
for i = idxsToPlot(2:end)
    errorLVLH = trajectoriesLVLHframe{i}(1:3, :) - trajectoriesLVLHframe{1}(1:3, :);
    ax.ColorOrderIndex = i;
    errorLVLHnorm = vecnorm(errorLVLH(:, 2:end));
    plot(ax, t, errorLVLHnorm, plotOptions{:});
end
% for i = idxsToPlot
%     trajECI = lvlh2eci(cfg, trajectoriesLVLHframe{i}, t(1));
%     quiver(ax, trajECI(1, :), trajECI(3, :), trajECI(4, :), trajECI(6, :));
% end
% legend(names{idxsToPlot}, names{idxsToPlot}, 'target orbit');
legend(names{idxsToPlot(2:end)}, 'Location', 'best');
% title('ECI');
xlabel('Simulation Time (s)');
ylabel('Error (m)');

setFigSize(gcf, 8, 6);
validationFigStylingAndSave(gcf, relpath('exp3-1-err.pdf'));

figure(34);
clf;
ax = gca;
hold on;
fullDynamicsDistanceTravelled = cumsum(vecnorm(diff(trajectoriesLVLHframe{1}(1:3, :), 1, 2)));
for i = idxsToPlot(2:end)
    errorLVLH = trajectoriesLVLHframe{i}(1:3, :) - trajectoriesLVLHframe{1}(1:3, :);
    ax.ColorOrderIndex = i;
    % error over distance traveled
    errorLVLHnormrel = vecnorm(errorLVLH(:, 2:end))./fullDynamicsDistanceTravelled*100;
    plot(ax, t, errorLVLHnormrel, plotOptions{:});
end

legend(names{idxsToPlot(2:end)}, 'Location', 'best');
% title('ECI');
xlabel('Simulation Time (s)');
ylabel('Relative Error (%)');

setFigSize(gcf, 8, 6);
validationFigStylingAndSave(gcf, relpath('exp3-1-err-rel.pdf'));

fdynamics = trajectoriesLVLHframe{1}(1:3, :);
cwdiscrete = trajectoriesLVLHframe{3}(1:3, :);

% Experiment 3 part 2, linearization point

cfg = params();

cfg.simulation.T = cfg.simulation.dt*100;
cfg.simulation.steps = ceil(cfg.simulation.T/cfg.simulation.dt);

dt = cfg.simulation.dt;
T = cfg.simulation.T;
cfg.simulation.x0 = [0, 0, 1000, 0, 0, 0]';
x0 = cfg.simulation.x0;
% x, y, z, x', y', z'

t = 0:dt:T;
XNewtonECI = zeros(length(x0), length(t) + 1);
Xcwcont = zeros(length(x0), length(t) + 1);
Xcwdiscrete = zeros(length(x0), length(t) + 1);
Xcwlin1 = zeros(length(x0), length(t) + 1);
Xcwlin2 = zeros(length(x0), length(t) + 1);
Xcwlin3 = zeros(length(x0), length(t) + 1);
Xcwlin4 = zeros(length(x0), length(t) + 1);
Xcwlin5 = zeros(length(x0), length(t) + 1);
U = zeros(6, length(t));

w = cfg.system.w;

iterationTime = zeros(1, length(t));
XNewtonECI(:,1) = x0;
Xcwcont(:,1) = x0;
Xcwdiscrete(:,1) = x0;
Xcwlin1(:,1) = x0;
Xcwlin2(:,1) = x0;
Xcwlin3(:,1) = x0;
Xcwlin4(:,1) = x0;
Xcwlin5(:,1) = x0;

H = @(xval) double(xval >= 0);
S = @(t) [
    H(t - 0.75*T);
    H(t - 0.25*T) - H(t - 0.5*T);
    0;
    0;
    H(t - 0.25*T) - H(t - 0.5*T);
    H(t - 0.75*T);
];


tlinchoices = [0, 2.5, 5, 7.5, 10];

u = zeros(length(x0), 1);
fDynamicscwlin1 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(1));
fDynamicscwlin2 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(2));
fDynamicscwlin3 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(3));
fDynamicscwlin4 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(4));
fDynamicscwlin5 = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlinchoices(5));

for i = 2:length(t)+1
    % simulate for one time step
    u = S(t(i-1))*dt/2;
    U(:, i-1) = u;

    [tR, yR] = ode45(@(tO, xO) newtonDynamicsECI(tO, xO, t(i-1), u, cfg), [t(i-1), t(i-1)+dt], lvlh2eci(cfg, XNewtonECI(:, i-1), t(i-1)));
    XNewtonECI(:, i) = eci2lvlh(cfg, yR(end, :)', t(i-1)+dt);

%     [tR, yR] = ode45(@(tO, xO) cfg.system.dynamics.AcCW*xO+cfg.system.dynamics.B*sumInline(@(idx) cfg.system.F(idx)*cfg.system.W(idx, :)'*double(u(idx)>0), 1:length(u)), [t(i-1), t(i-1)+dt], Xcwcont(:, i-1));
    [tR, yR] = ode45(@(tO, xO) cfg.system.dynamics.AcCW*xO+cfg.system.dynamics.B*sumInline(@(idx) cfg.system.F(idx)*cfg.system.W(idx, :)'*double(u(idx)-(tO - t(i-1))>0), 1:length(u)), [t(i-1), t(i-1)+dt], Xcwcont(:, i-1));
    Xcwcont(:, i) = yR(end, :)';
    
    Xcwdiscrete(:, i) = cfg.system.dynamics.fCW(Xcwdiscrete(:, i-1), u);
    
    Xcwlin1(:, i) = fDynamicscwlin1(Xcwlin1(:, i-1), u);
    Xcwlin2(:, i) = fDynamicscwlin2(Xcwlin2(:, i-1), u);
    Xcwlin3(:, i) = fDynamicscwlin3(Xcwlin3(:, i-1), u);
    Xcwlin4(:, i) = fDynamicscwlin4(Xcwlin4(:, i-1), u);
    Xcwlin5(:, i) = fDynamicscwlin5(Xcwlin5(:, i-1), u);

end
    
trajectoriesLVLHframe = {XNewtonECI, Xcwcont, Xcwdiscrete, Xcwlin1, Xcwlin2, Xcwlin3, Xcwlin4, Xcwlin5};
names = {
    'full dynamics', ...
    'continuous CW', ...
    'discrete CW', ...
    sprintf('s_0=%.1f', tlinchoices(1)), ...
    sprintf('s_0=%.1f', tlinchoices(2)), ...
    sprintf('s_0=%.1f', tlinchoices(3)), ...
    sprintf('s_0=%.1f', tlinchoices(4)), ...
    sprintf('s_0=%.1f', tlinchoices(5))
   };
idxsToPlot = [1, 3, 4, 5, 6];

orbit = zeros(6, length(t));
orbiteci = lvlh2eci(cfg, orbit, t(1));

figure(35);
clf;
ax = gca;
hold on;
for i = idxsToPlot
    plot(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), plotOptions{:});
end
% for i=idxsToPlot
%     quiver(ax, trajectoriesLVLHframe{i}(1, :), trajectoriesLVLHframe{i}(3, :), trajectoriesLVLHframe{i}(4, :), trajectoriesLVLHframe{i}(6, :));
% end
plot(ax, trajectoriesLVLHframe{1}(1, 1), trajectoriesLVLHframe{1}(3, 1), 'x', 'Color', 'black');
% legend(names{idxsToPlot}, names{idxsToPlot});
legend(names{idxsToPlot}, 'start', 'Location', 'best');
% title('LVLH');
% set(ax, 'YAxisLocation','right');
% set(ax, 'XAxisLocation','top');
set(ax, 'YDir', 'reverse');
set(ax, 'XDir', 'reverse');
xlabel('x (m)');
ylabel('z (m)');

setFigSize(gcf, 8, 6);
validationFigStylingAndSave(gcf, relpath('exp3-2-traj-lvlh.pdf'));

figure(36);
clf;
ax = gca;
im = imagesc([0, cfg.simulation.T], [1,4], U([1, 2, 5, 6], :), [0, dt]);
xlabel('Simulation Time (s)');
ylabel('Thruster');
arrayfun(@(yy) line([0, cfg.simulation.T], [yy, yy], 'Color', 'black', 'LineWidth', 1), [1.5, 2.5, 3.5]);
thrusternames = {'Down', 'Up', 'Left', 'Right'};
thrusterticks = 1:4;
set(ax,'ytick',thrusterticks,'yticklabel',thrusternames)

setFigSize(gcf, 8, 5);
validationFigStylingAndSave(gcf, relpath('exp3-2-%d-control.pdf'));
    
figure(37);
clf;
ax = gca;
hold on;
for i = idxsToPlot(2:end)
    errorLVLH = trajectoriesLVLHframe{i}(1:3, :) - trajectoriesLVLHframe{1}(1:3, :);
    ax.ColorOrderIndex = i;
    errorLVLHnorm = vecnorm(errorLVLH(:, 2:end));
    plot(ax, t, errorLVLHnorm, plotOptions{:});
end
% for i = idxsToPlot
%     trajECI = lvlh2eci(cfg, trajectoriesLVLHframe{i}, t(1));
%     quiver(ax, trajECI(1, :), trajECI(3, :), trajECI(4, :), trajECI(6, :));
% end
% legend(names{idxsToPlot}, names{idxsToPlot}, 'target orbit');
legend(names{idxsToPlot(2:end)}, 'Location', 'best');
% title('ECI');
xlabel('Simulation Time (s)');
ylabel('Error (m)');

setFigSize(gcf, 8, 6);
validationFigStylingAndSave(gcf, relpath('exp3-2-err.pdf'));

figure(38);
clf;
ax = gca;
hold on;
fullDynamicsDistanceTravelled = cumsum(vecnorm(diff(trajectoriesLVLHframe{1}(1:3, :), 1, 2)));
for i = idxsToPlot(2:end)
    errorLVLH = trajectoriesLVLHframe{i}(1:3, :) - trajectoriesLVLHframe{1}(1:3, :);
    ax.ColorOrderIndex = i;
    % error over distance traveled
    errorLVLHnormrel = vecnorm(errorLVLH(:, 2:end))./fullDynamicsDistanceTravelled*100;
    plot(ax, t, errorLVLHnormrel, plotOptions{:});
end

legend(names{idxsToPlot(2:end)}, 'Location', 'best');
% title('ECI');
xlabel('Simulation Time (s)');
ylabel('Relative Error (%)');

setFigSize(gcf, 8, 6);
validationFigStylingAndSave(gcf, relpath('exp3-2-err-rel.pdf'));

fdynamics = trajectoriesLVLHframe{1}(1:3, :);
cwdiscrete = trajectoriesLVLHframe{3}(1:3, :);

%% aux

for part=1:2
    figure(31 + (part-1)*4);
    setFigSize(gcf, 13, 6);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp3-%d-traj-lvlh.pdf', part)));
        
    figure(32 + (part-1)*4);
    setFigSize(gcf, 5, 6);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp3-%d-control.pdf', part)));
    
    figure(33 + (part-1)*4);
    setFigSize(gcf, 8, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp3-%d-err.pdf', part)));
    
    figure(34 + (part-1)*4);
    setFigSize(gcf, 8, 5);
    validationFigStylingAndSave(gcf, relpath(sprintf('exp3-%d-err-rel.pdf', part)));
end

