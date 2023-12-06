cfg = params();

simOverrideCache = false;

cfg.simulation.verbosity = 0;
cfg.controller.N = 10; 
cfg.simulation.steps = 1000;
cfg.simulation.T = cfg.simulation.steps * cfg.simulation.dt;
cfg.controller.tmin = 5;
cfg.simulation.method = 'standard';

id = @(cfglocal) sprintf('$tlin=%.1f$', cfglocal.controller.tlin);

cfgs = cell(5, 1);
results = cell(5, 1);

cfg.controller.tlin = 0;
[X, S, T] = simulate(cfg, [], simOverrideCache);
cfgs{1} = cfg;
res = struct; res.X = X; res.S = S; res.T = T; results{1} = res;

cfg.controller.tlin = cfg.controller.tmin;
[X, S, T] = simulate(cfg, [], simOverrideCache);
cfgs{2} = cfg;
res = struct; res.X = X; res.S = S; res.T = T; results{2} = res;

cfg.controller.tlin = (cfg.simulation.dt + cfg.controller.tmin)/2;
[X, S, T] = simulate(cfg, [], simOverrideCache);
cfgs{3} = cfg;
res = struct; res.X = X; res.S = S; res.T = T; results{3} = res;

cfg.controller.tlin = cfg.simulation.dt;
[X, S, T] = simulate(cfg, [], simOverrideCache);
cfgs{4} = cfg;
res = struct; res.X = X; res.S = S; res.T = T; results{4} = res;

cfg.controller.tlin = cfg.controller.tmin/2;
[X, S, T] = simulate(cfg, [], simOverrideCache);
cfgs{5} = cfg;
res = struct; res.X = X; res.S = S; res.T = T; results{5} = res;


[resCfgs, resNonlin] = smallSims(cfgs);
errors = cell(size(resCfgs));
for ii = 1:numel(resCfgs)
    err = struct;
    err.Xact = resNonlin.Xact -resCfgs{ii}.Xact;
    err.Xfree = resNonlin.Xfree -resCfgs{ii}.Xfree;
    errors{ii} = err;
end


printstats(1:numel(cfgs), id, cfgs, results, errors);

figname = 'comparelin_errors';
names = cell(size(cfgs));
imagefolder = 'imgs';

if ~exist(imagefolder, 'dir') 
    mkdir(imagefolder)
end
relpathinimgfolder = @(filename) sprintf('%s/compareLin_%s', imagefolder, filename);
colors = colororder;
trajPlotOptions = {'Linewidth', 1.2};
h = findall(groot, 'Type', 'figure', 'Name', figname);
if isempty(h)
    h = figure('Name',figname,'NumberTitle','off');
end
figure(h);
clf;
h.Position(3) = 600;
h.Position(4) = 300;
ax = subplot(1, 2, 1);
hold on;
plotlvlh(gca, cfgs{ii}, resNonlin.Xact, trajPlotOptions);
for ii = 1:numel(resCfgs)
    plotlvlh(gca, cfgs{ii}, resCfgs{ii}.Xact, trajPlotOptions);
    names{ii} = id(cfgs{ii});
end
legend('nonlinear',names{:},'Interpreter','latex');    
xlabel('x (m)');
ylabel('z (m)');

ax = subplot(1, 2, 2);
hold on;
ax.ColorOrderIndex = 2; % same colors as before
for ii = 1:numel(resCfgs)
    plotlvlh(gca, cfgs{ii}, errors{ii}.Xact, trajPlotOptions);
    names{ii} = id(cfgs{ii});
end
xlabel('x (m)');
ylabel('z (m)');


function [resCfgs, resNonlin] = smallSims(cfgs)
    resCfgs = cell(size(cfgs));
    for jj=1:numel(cfgs)
        cfg=cfgs{jj};
        dt = cfg.simulation.dt;
        T = cfg.simulation.T;
        x0 = cfg.simulation.x0;
        xref = cfg.simulation.xTarget;
        t = 0:dt:T;
        Xfree = zeros(length(x0), length(t) + 1);
        Xact = zeros(length(x0), length(t) + 1);
        U = zeros(length(x0), length(t));
        iterationTime = zeros(1, length(t));
        steps = cfg.simulation.steps;
        tmin = cfg.controller.tmin;
        tlin = cfg.controller.tlin;
        alpha = @(t, ton, toff) heaviside(t - ton) - heaviside(t - toff);
        sOdd = @(step) dt*alpha(step, 0, steps/8) + 0.5*(tmin+dt)*alpha(step, steps/8, 3*steps/8);
        sEven = @(step) (0.2*dt+0.8*tmin)*alpha(step, steps/8, 3*steps/8) + (0.8*tmin+0.8*dt)*alpha(step, steps/2, 5*steps/8);
        S = @(step) [
            sOdd(step), ...
            sEven(step), ...
            sOdd(step), ...
            sEven(step), ...
            sOdd(step), ...
            sEven(step)
        ];
        fDynamicsLin = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, tlin);
        Xfree(:, 1) = x0;
        Xact(:, 1) = x0;
        for k=1:length(t)
            
            Xfree(:,k+1) = fDynamicsLin(Xfree(:,k), zeros(length(cfg.system.F), 1));
            Xact(:,k+1) = fDynamicsLin(Xact(:,k), S(k));
        end
        resCfg = struct; resCfg.Xfree = Xfree; resCfg.Xact = Xact;
        resCfgs{jj} = resCfg;
    end
    fDynamicsNonlin = cfg.system.dynamics.fCW;
    for k=1:length(t)
        Xfree(:,k+1) = fDynamicsNonlin(Xfree(:,k), zeros(length(cfg.system.F), 1));
        Xact(:,k+1) = fDynamicsNonlin(Xact(:,k), S(k));
    end
    resNonlin = struct; resNonlin.Xfree = Xfree; resNonlin.Xact = Xact;
end


function printstats(idxs, labelfnc, cfgs, results, errors)
    fprintf('id\t\t\t fuelSpent\t\t   finishTime\t\terrorXact\t\t errorXfree\t\t simTime\n')
    for ii=1:length(idxs)
        result = results{idxs(ii)};
        cfg = cfgs{idxs(ii)};
        error = errors{idxs(ii)};
        errorXact = sum(abs(error.Xact), 'all');
        errorXfree = sum(abs(error.Xfree), 'all');
        finishIdx = getFinishTime(result.X);
        fuelSpent = sum(result.S(:, 1:(finishIdx-1)), 'all');
        finishTime = (finishIdx-1)*cfg.simulation.dt;
        simTime = sum(result.T, 'all');
        if ii < length(idxs)
            linebreak = '\\';
        else
            linebreak = '';
        end
        if (finishIdx-1) > cfg.simulation.steps
            fprintf('%s & \\SI{%.2f}{s} & DNF\\tnote{1} & \\SI{%.2f}{} & \\SI{%.2f}{} & \\SI{%.2f}{} %s\n', labelfnc(cfg), fuelSpent, errorXact, errorXfree, simTime, linebreak);
        else
            fprintf('%s & \\SI{%.2f}{s} & \\SI{%.0f}{s} & \\SI{%.2f}{} & \\SI{%.2f}{} & \\SI{%.2f}{} %s\n', labelfnc(cfg), fuelSpent, finishTime, errorXact, errorXfree, simTime, linebreak);
        end
    end
end

function makeplots(actual_idxs, legendfunc, figname, cfgs, results)
    imagefolder = 'imgs';
    if ~exist(imagefolder, 'dir')
        mkdir(imagefolder)
    end

    relpathinimgfolder = @(filename) sprintf('%s/compareLin_%s', imagefolder, filename);
    colors = colororder;

    thrusternames = {'Right', 'Left', 'Up', 'Down'};
    thrusterticks = 1:4;
    trajPlotOptions = {'Linewidth', 1.2};

    h = findall(groot, 'Type', 'figure', 'Name', figname);
    if isempty(h)
        h = figure('Name',figname,'NumberTitle','off');
    end
    figure(h);
    clf;
    h.Position(3) = 600;
    h.Position(4) = 300;
    ax = subplot(3, 2, [1, 3, 5]);
    hold on;
    plotlvlh(gca, cfgs{actual_idxs(1)}, results{actual_idxs(1)}.X, trajPlotOptions);
    plotlvlh(gca, cfgs{actual_idxs(2)}, results{actual_idxs(2)}.X, trajPlotOptions);
    plotlvlh(gca, cfgs{actual_idxs(3)}, results{actual_idxs(3)}.X, trajPlotOptions);
    leg = legend(legendfunc(cfgs{actual_idxs(1)}), legendfunc(cfgs{actual_idxs(2)}), legendfunc(cfgs{actual_idxs(3)}),'Interpreter','latex');
    xlabel('x (m)');
    ylabel('z (m)');
    pos = ax.Position;
    ax.Position = pos + [0, 0.03, 0, 0];

    thruster_axes = cell(3, 1);
    for ii=1:3
        axi = subplot(3, 2, 2*ii);
        thruster_axes{ii} = axi;
        control_signal = results{actual_idxs(ii)}.S([1, 2, 5, 6], :);
        im = imagesc([0, cfgs{actual_idxs(ii)}.simulation.T], [1,4], control_signal);
        xlabel('Simulation Time (s)');
        ylabel('Thruster');
        set(axi,'ytick',thrusterticks,'yticklabel',thrusternames)
    end
    
    axc = axes('Position',[0.91 0.168 0.01 0.7], 'Visible', 'off');
    cb = colorbar(axc, 'Ticks', [0, 1], 'TickLabels', {'0%', '100%'});
    cb.Position = [0.92, 0.1685, 0.03, 0.7];
    cb.Label.String = 'Time Open';
    cb.Label.Rotation = 270;
    cb.Label.Position = [2, 0.5, 0.5];
    
    saveax([ax, leg], sprintf('%s.pdf', relpathinimgfolder(figname)), h.Position);
    saveax(thruster_axes{1}, sprintf('%s_t1.pdf', relpathinimgfolder(figname)), h.Position);
    saveax(thruster_axes{2}, sprintf('%s_t2.pdf', relpathinimgfolder(figname)), h.Position);
    saveax(thruster_axes{3}, sprintf('%s_t3.pdf', relpathinimgfolder(figname)), h.Position);
    saveax([cb, axc], sprintf('%s.pdf', relpathinimgfolder('cb')), h.Position);
end
