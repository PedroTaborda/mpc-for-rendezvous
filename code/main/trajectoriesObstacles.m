
cfgdefault = params();

% N = [5, 10, 20, 40];
N = [5, 10, 15];
Ffactor = [1];
methods = {'standard', 'standard-o'};
% methods = {'standard-o'};

dn=2; % index containing default values
dffac=1;
dmethod = 2;

cfgdefault.simulation.obstacles = {
    struct('center', [-10000.0; 70000.0], 'radius', 20000), ...
    struct('center', [10000.0; 30000.0], 'radius', 15000), ...
};
cfgdefault.simulation.dt = 10;
cfgdefault.controller.tmin = 1;
% cfgdefault.simulation.T = cfgdefault.simulation.dt*40;
% cfgdefault.simulation.steps = ceil(cfgdefault.simulation.T/cfgdefault.simulation.dt);

cfgs = cell(length(N), length(Ffactor), length(methods));
results = cell(length(N), length(Ffactor), length(methods));

for ii = 1:length(N)
    for jj = 1:length(Ffactor)
        for kk = 1:numel(methods)
            if nnz([(ii-dn), (jj-dffac), (kk-dmethod)]) > 1
                continue
            end
            cfg = cfgdefault;
            cfg.controller.N = N(ii);
            cfg.system.F = Ffactor(jj)*cfgdefault.system.F;
            cfg.simulation.method = methods{kk};
            
            cfgs{ii, jj, kk} = cfg;
            res = struct();
            [res.X, res.S, res.T] = simulate(cfg, [], false);
            results{ii, jj, kk} = res;
        end
    end
end

unidimensional_index = @(array_idx) (array_idx(1)) + (array_idx(2)-1)*length(N) + (array_idx(3)-1)*length(Ffactor)*length(N);

whatN = @(cfglocal) sprintf('$N=%d$', cfglocal.controller.N);
idxs_N = arrayfun(@(idx) unidimensional_index([idx, dffac, dmethod]), 1:length(N));
makeplots(idxs_N, whatN, 'N', cfgs, results);
printstats(idxs_N, whatN, cfgs, results);

whatMethod = @(cfglocal) sprintf('%s', cfglocal.simulation.method);
idxs_method = arrayfun(@(idx) unidimensional_index([dn, dffac, idx]), 1:length(methods));
makeplots(idxs_method, whatMethod, 'method', cfgs, results);
printstats(idxs_method, whatMethod, cfgs, results);

function printstats(idxs, labelfnc, cfgs, results)
    for ii=1:length(idxs)
        result = results{idxs(ii)};
        cfg = cfgs{idxs(ii)};
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
            fprintf('%s & \\SI{%.2f}{s} & DNF\\tnote{1} & \\SI{%.2f}{s} %s\n', labelfnc(cfg), fuelSpent, simTime, linebreak);
        else
            fprintf('%s & \\SI{%.2f}{s} & \\SI{%.0f}{s} & \\SI{%.2f}{s} %s\n', labelfnc(cfg), fuelSpent, finishTime, simTime, linebreak);
        end 
    end
end

function makeplots(actual_idxs, legendfunc, figname, cfgs, results, saveonly)
    if nargin < 6
        saveonly = false;
    end
    imagefolder = 'imgs';
    if ~exist(imagefolder, 'dir')
        mkdir(imagefolder)
    end

    relpathinimgfolder = @(filename) sprintf('%s/trajStandardObs_%s', imagefolder, filename);
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
    set(h, "DefaultAxesFontSize", 12);
    set(h, "DefaultTextFontSize", 12);
    % check if docked
    if ~strcmp(get(h, 'WindowStyle'), 'docked')
        h.Position(3) = 600;
        h.Position(4) = 300;
    end
    ax = subplot(length(actual_idxs), 2, 1:2:2*length(actual_idxs));
    hold on;
    names = cell(1, length(actual_idxs));
    for idx = 1:length(actual_idxs)
        plotlvlh(gca, cfgs{actual_idxs(idx)}, results{actual_idxs(idx)}.X, trajPlotOptions);
        names{idx} = legendfunc(cfgs{actual_idxs(idx)});
    end
    % plot an x on the starting point
    x0 = cfgs{actual_idxs(1)}.simulation.x0;
    plot(x0(1), x0(3), 'x', 'Color', 'black', 'Linewidth', 1);
    plot(0,0, 'o', 'Color', 'black', 'Linewidth', 1);
    
    set(gca, 'YDir','reverse');
    set(gca, 'XDir','reverse');
    axis equal;    
    xlimcur = xlim;
    xlim([xlimcur(1)-10^4, xlimcur(2)-10^4]);
    for idx = 1:length(actual_idxs)
        if ~endsWith(cfgs{actual_idxs(idx)}.simulation.method, '-o')
            continue;
        end
        if numel(cfgs{actual_idxs(idx)}.simulation.obstacles) > 0
            plotObstacles(gca, cfgs{actual_idxs(idx)}, {'Color', colors(5, :), 'Linewidth', 0.5});
            break; % only plot obstacles once
        end
    end
    leg = legend(names{:}, 'start', 'target', 'Location', 'south east', 'interpreter', 'latex');
    xlabel('x (m)');
    ylabel('z (m)');
    pos = ax.Position;
    ax.Position = pos + [0, 0.03, 0, 0];

    thruster_axes = cell(3, 1);
    for ii=1:length(actual_idxs)
        axi = subplot(length(actual_idxs), 2, 2*ii);
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

