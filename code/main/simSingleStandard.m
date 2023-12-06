
cfg = params();

cfg.controller.N = 20;
cfg.simulation.dt = 10;
cfg.simulation.method = 'projected';
cfg.system.F = cfg.system.F;
cfg.controller.tmin = 1;
cfg.simulation.T = cfg.simulation.dt*140;
cfg.simulation.steps = ceil(cfg.simulation.T/cfg.simulation.dt);
res = struct();

figure(1); clf;
ax = subplot(1, 1, 1); 
hold on;
axis equal;
plotObstacles(ax, cfg, {});
% simulate executes an arbitrary function of the state (X(:, 1:currentStep), Umpc) where Umpc is the MxN matrix of control inputs computed by the MPC controller
% we use this to plot the predicted trajectory at each step and wait for the user to press a key to continue
plotWait = @(X, Umpc) plotPredictedAndWait(ax, cfg, X, Umpc);
[res.X, res.S, res.T] = simulate(cfg, plotWait, true);

%%

makeplots("", 'N', cfg, res);
% printstats("", cfg, res);


function plotPredictedAndWait(ax, cfg, X, Umpc)
    curState = X(:, end-1);
    M = cfg.controller.M;
    N = cfg.controller.N;
    n = length(cfg.simulation.x0);
    plot(ax, curState(1), curState(3), 'x', 'Linewidth', 1.2, 'Color', [0, 0.4470, 0.7410]);
%     return;
    predictedStates = zeros(n, N+1);
    predictedStates(:, 1) = curState;
    fDynamics = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);
    for ii = 1:N
        predictedStates(:, ii+1) = fDynamics(predictedStates(:, ii), Umpc(:, ii));
    end
    % the x, z position coordinates are the first and third rows of the state
    % plot each point (not connected by lines) and wait for the user to press a key
    line = plot(ax, predictedStates(1, :), predictedStates(3, :), 'x', 'Linewidth', 1.2, 'Color', [0.8500, 0.3250, 0.0980]);
    % also plot the current state
    drawnow;
%     input('', 's');
    delete(line);
end


function printstats(label, cfg, result)
    finishIdx = getFinishTime(result.X);
    fuelSpent = sum(result.S(:, 1:(finishIdx-1)), 'all');
    finishTime = (finishIdx-1)*cfg.simulation.dt;
    simTime = sum(result.T, 'all');
    linebreak = '';
    if (finishIdx-1) > cfg.simulation.steps
        fprintf('%s & \\SI{%.2f}{s} & DNF\\tnote{1} & \\SI{%.2f}{s} %s\n', label, fuelSpent, simTime, linebreak);
    else
        fprintf('%s & \\SI{%.2f}{s} & \\SI{%.0f}{s} & \\SI{%.2f}{s} %s\n', label, fuelSpent, finishTime, simTime, linebreak);
    end 
end

function makeplots(legendstr, figname, cfg, result)
    imagefolder = 'imgs';
    if ~exist(imagefolder, 'dir')
        mkdir(imagefolder)
    end

    relpathinimgfolder = @(filename) sprintf('%s/trajStandard_%s', imagefolder, filename);
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
    plotlvlh(gca, cfg, result.X, trajPlotOptions);
    for cc = 1:numel(cfg.simulation.obstacles)
        obstacle = cfg.simulation.obstacles{cc};
        drawCircle(ax, obstacle.center, obstacle.radius, {}, 30);
    end
    xlabel('x (m)');
    ylabel('z (m)');
    pos = ax.Position;
    ax.Position = pos + [0, 0.03, 0, 0];

    thruster_axes = cell(3, 1);
    axi = subplot(3, 2, 2);
    control_signal = result.S([1, 2, 5, 6], :);
    im = imagesc([0, cfg.simulation.T], [1,4], control_signal);
    xlabel('Simulation Time (s)');
    ylabel('Thruster');
    set(axi,'ytick',thrusterticks,'yticklabel',thrusternames)
    
    axc = axes('Position',[0.91 0.168 0.01 0.7], 'Visible', 'off');
    cb = colorbar(axc, 'Ticks', [0, 1], 'TickLabels', {'0%', '100%'});
    cb.Position = [0.92, 0.1685, 0.03, 0.7];
    cb.Label.String = 'Time Open';
    cb.Label.Rotation = 270;
    cb.Label.Position = [2, 0.5, 0.5];
    
end

