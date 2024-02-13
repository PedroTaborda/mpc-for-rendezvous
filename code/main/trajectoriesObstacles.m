
cfgdefault = params();
cfgdefault.controller.Q=diag([1, 1, 1, 1, 1, 1])*10;
% cfgdefault.controller.Xfonly=false;

% N = [5, 10, 20, 40];
N = [10, 50, 100];
Ffactor = [1];
methods = {'relaxed', 'projected'};
% methods = {'standard-o'};

dn=2; % index containing default values
dffac=1;
dmethod = 2;

cfgdefault.simulation.obstacles = {
%     struct('center', [15000.0; 30000.0], 'radius', 5000), ...
%     struct('center', [-10000.0; 70000.0], 'radius', 20000), ...
%     struct('center', [10000.0; 30000.0], 'radius', 15000), ...
    struct('center', [-10000.0; 70000.0], 'radius', 10000), ...
    struct('center', [10000.0; 30000.0], 'radius', 7500), ...
};
% cfgdefault.simulation.dt = 10;
cfgdefault.controller.tmin = 1;
cfgdefault.simulation.T = cfgdefault.simulation.dt*100;
cfgdefault.simulation.steps = ceil(cfgdefault.simulation.T/cfgdefault.simulation.dt);

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
makeplots(idxs_N, whatN, 'N2', cfgs, results);
printstats(idxs_N, whatN, cfgs, results);

whatMethod = @(cfglocal) sprintf('%s', cfglocal.simulation.method);
idxs_method = arrayfun(@(idx) unidimensional_index([dn, dffac, idx]), 1:length(methods));
makeplots(idxs_method, whatMethod, 'method2', cfgs, results);
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
function makeplots(actual_idxs, legendfunc, figname, cfgs, results)
    imagefolder = 'imgs';
    if ~exist(imagefolder, 'dir')
        mkdir(imagefolder)
    end

    relpathinimgfolder = @(filename) sprintf('%s/trajStandard_%s', imagefolder, filename);
    colors = colororder;

    thrusternames = {'Down', 'Up', 'Left', 'Right'};
    thrusterticks = 1:4;
    trajPlotOptions = {'Linewidth', 1.5};

    fignameTraj = sprintf('%s_traj.pdf', figname);
    h = findall(groot, 'Type', 'figure', 'Name', fignameTraj);
    if isempty(h)
        h = figure('Name',fignameTraj,'NumberTitle','off');
    end
    figure(h);
    clf;
    % set default font size for all plots and figure children
    
    set(h, "DefaultAxesFontSize", 8);
    set(h, "DefaultTextFontSize", 8);

    % check if docked
    if ~strcmp(get(h, 'WindowStyle'), 'docked')
        h.OuterPosition(3) = 244.0000;
        h.OuterPosition(4) = 300;
    end
    ax = subplot(1, 1, 1);
    hold on; box on;
    names = cell(1, length(actual_idxs));
    for idx = 1:length(actual_idxs)
        results{actual_idxs(idx)}.X = results{actual_idxs(idx)}.X*1e-3;
        plotlvlh(gca, cfgs{actual_idxs(idx)}, results{actual_idxs(idx)}.X, trajPlotOptions);
        names{idx} = legendfunc(cfgs{actual_idxs(idx)});
    end
    % plot an x on the starting point
    ax.Children = flipud(ax.Children);
    set(gca, 'Linewidth', 1.5);
    x0 = cfgs{actual_idxs(1)}.simulation.x0*1e-3;
    plot(x0(1), x0(3), 'x', 'Color', 'black', 'Linewidth', 1);
    plot(0,0, 'o', 'Color', 'black', 'Linewidth', 1);
    
    set(gca, 'YDir','reverse');
    set(gca, 'XDir','reverse');

    
    xlimcur = xlim;
%     xlim([xlimcur(1)+10^4, xlimcur(2)+10^4]);
    for idx = 1:length(actual_idxs)
        if numel(cfgs{actual_idxs(idx)}.simulation.obstacles) > 0
            plotObstacles(gca, cfgs{actual_idxs(idx)}, {'Color', colors(5, :), 'Linewidth', 0.5});
            break; % only plot obstacles once
        end
    end
%     leg = legend(names{:}, 'start', 'target','Interpreter','latex', 'Location', 'north west');
    xlabel('x [km]');
    ylabel('z [km]');
    pos = ax.Position;
    ax.Position = pos + [0, 0.03, 0, 0];
    saveas(h, sprintf('%s.pdf', relpathinimgfolder(fignameTraj)));
    system(sprintf("pdfcrop %s %s", sprintf('%s.pdf', relpathinimgfolder(fignameTraj)), sprintf('%s.pdf', relpathinimgfolder(fignameTraj))));

    fignameThrusters = sprintf('%s_thrusters.pdf', figname);
    ht = findall(groot, 'Type', 'figure', 'Name', fignameThrusters);
    if isempty(ht)
        ht = figure('Name',fignameThrusters,'NumberTitle','off');
    end
    figure(ht);
    clf;
    % set default font size for all plots and figure children
    
    set(ht, "DefaultAxesFontSize", 8);
    set(ht, "DefaultTextFontSize", 8);

    % check if docked
    if ~strcmp(get(ht, 'WindowStyle'), 'docked')
        ht.OuterPosition(3) = 244.0000;
        ht.OuterPosition(4) = 300;
    end


    thruster_axes = cell(3, 1);
    for ii=1:length(actual_idxs)
        axi = subplot(length(actual_idxs), 1, ii);
        thruster_axes{ii} = axi;
        control_signal = results{actual_idxs(ii)}.S([1, 2, 5, 6], :);
        im = imagesc([0, cfgs{actual_idxs(ii)}.simulation.T], [1,4], control_signal);
        line([cfgs{actual_idxs(ii)}.simulation.T, cfgs{actual_idxs(ii)}.simulation.T], [0.5, 4.5], 'Color', colors(ii, :), 'LineWidth', 3)
        if ii == length(actual_idxs)
            xlabel('Time [s]');
        else
            set(gca,'XTick',[]);
        end
        axi.Position = (diag([1, 1, 1, 1.1])*axi.Position')';
        axi.Position = axi.Position + [0, 0.05, 0, 0];
%         ylabel('Thruster');
        set(axi,'ytick',thrusterticks,'yticklabel',thrusternames)
    end
    
    saveas(ht, sprintf('%s.pdf', relpathinimgfolder(fignameThrusters)));
    system(sprintf("pdfcrop %s %s", sprintf('%s.pdf', relpathinimgfolder(fignameThrusters)), sprintf('%s.pdf', relpathinimgfolder(fignameThrusters))));
%     axc = axes('Position',[0.91 0.168 0.01 0.7], 'Visible', 'off');
%     cb = colorbar(axc, 'Ticks', [0, 1], 'TickLabels', {'0%', '100%'});
%     cb.Position = [0.92, 0.1685, 0.03, 0.7];
%     cb.Label.String = 'Time Open';
%     cb.Label.Rotation = 270;
%     cb.Label.Position = [2, 0.5, 0.5];

    

%     saveax([ax, leg], sprintf('%s.pdf', relpathinimgfolder(figname)), h.Position);
%     saveax(thruster_axes{1}, sprintf('%s_t1.pdf', relpathinimgfolder(figname)), h.Position);
%     saveax(thruster_axes{2}, sprintf('%s_t2.pdf', relpathinimgfolder(figname)), h.Position);
%     saveax(thruster_axes{3}, sprintf('%s_t3.pdf', relpathinimgfolder(figname)), h.Position);
%     saveax([cb, axc], sprintf('%s.pdf', relpathinimgfolder('cb')), h.Position);
end

