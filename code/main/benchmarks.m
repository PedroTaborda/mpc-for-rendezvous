%% Solver benchmarking

cfgDefault = params();
cfgDefault.simulation.verbosity = 0;



Ns = [5, 10, 15]; % MPC controller horizon
%% Parameters to compare
for ii = 1:3
%     N = [5, 10, 15]; % MPC controller horizon
    N=Ns(ii);
    dt = 10; % simulation step size
    method = {'standard', 'projected', 'relaxed'}; % MPC solver method
    rerunID = 0:99;
    % rerunID = 0;
    suffix =sprintf('N%d.pdf', N);
    % suffix ='Overall.pdf';
    
    tmin = [5];
    
    steps = ceil(cfgDefault.simulation.T/dt);
    simt = dt:dt:cfgDefault.simulation.T;
    
    %% Load (or perform) all simulations
    
    % Preallocate
    t = zeros(length(N), length(tmin), length(dt), length(rerunID), length(method), steps);
    X = zeros(length(N), length(tmin), length(dt), length(rerunID), length(method), steps, 2);
    U = zeros(length(N), length(tmin), length(dt), length(rerunID), length(method), steps, 6);
    B = [1, 0, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0];
    
    ii = 0;
    NN=length(N)*length(tmin)*length(dt)*length(rerunID)*length(method);
    for n = 1:length(rerunID)
        fprintf("RERUN: %d/%d\n", n, length(rerunID));
        for i = 1:length(N)
            for j = 1:length(tmin)
                for k = 1:length(dt)
                    for l = 1:length(method)
                        cfg = cfgDefault;
                        cfg.controller.N = N(i);
                        cfg.controller.tmin = tmin(j);
                        cfg.simulation.dt = dt(k);
                        cfg.simulation.steps = ceil(cfg.simulation.T/dt(k));
    %                     disp(cfg.simulation.T)
                        cfg.simulation.method = method{l};
                        cfg.simulation.rerun = rerunID(n);
                        
                        % Run simulation
                        [x, u, titer] = simulate(cfg);
                        t(i,j,k,n,l,:) = titer(2:end);
                        X(i,j,k,n,l,:,:) = (B*x(:, 3:end))';
                        U(i,j,k,n,l,:,:) = u(:, 2:end)';
                        ii = ii+1;
                        fprintf("%d/%d\n", ii, NN);
                    end
                end
            end
        end
    end
    
    %% Get finishing times for each method, as an average over all other parameters, using getFinishTime(X) 
    fIdxStandard = zeros(length(N), length(tmin), length(dt), length(rerunID));
    fIdxProjected = zeros(length(N), length(tmin), length(dt), length(rerunID));
    fIdxRelaxed = zeros(length(N), length(tmin), length(dt), length(rerunID));
    
    for i = 1:length(N)
        for j = 1:length(tmin)
            for k = 1:length(dt)
                for n = 1:length(rerunID)
                    fIdxStandard(i,j,k,n) = getFinishTime(reshape(X(i,j,k,n,1,:,:), 2, steps));
                    fIdxProjected(i,j,k,n) = getFinishTime(reshape(X(i,j,k,n,2,:,:), 2, steps));
                    fIdxRelaxed(i,j,k,n) = getFinishTime(reshape(X(i,j,k,n,3,:,:), 2, steps));
                end
            end
        end
    end
    
    fTimeStandardMean = mean(fIdxStandard, 'all')*dt;
    fTimeProjectedMean = mean(fIdxProjected, 'all')*dt;
    fTimeRelaxedMean = mean(fIdxRelaxed, 'all')*dt;
    
    %% Variable packing
    
    tstandard = reshape(t(:,:,:,:,1,:), [], steps);
    tprojected = reshape(t(:,:,:,:,2,:), [], steps);
    trelaxed = reshape(t(:,:,:,:,3,:), [], steps);
    
    Xstandard = X(:,:,:,:,1,:,:);
    Xprojected = X(:,:,:,:,2,:,:);
    Xrelaxed = X(:,:,:,:,3,:,:);
    
    Eprojected = Xprojected - Xstandard;
    Erelaxed = Xrelaxed - Xstandard;
    
    Eprojected = reshape(Eprojected, [], 2, steps);
    Erelaxed = reshape(Erelaxed, [], 2, steps);
    
    % norm
    EprojectedNorm = squeeze(vecnorm(Eprojected, 2, 2));
    ErelaxedNorm = squeeze(vecnorm(Erelaxed, 2, 2));
    
%     Ustandard = U(:,:,:,:,1,:,:);
%     Uprojected = U(:,:,:,:,2,:,:);
%     Urelaxed = U(:,:,:,:,3,:,:);
%     
%     Ustandard = reshape(Ustandard, [], 6, steps);
%     Uprojected = reshape(Uprojected, [], 6, steps);
%     Urelaxed = reshape(Urelaxed, [], 6, steps);
%     
%     % norm
%     UstandardNorm = squeeze(vecnorm(Ustandard, 2, 2));
%     UprojectedNorm = squeeze(vecnorm(Uprojected, 2, 2));
%     UrelaxedNorm = squeeze(vecnorm(Urelaxed, 2, 2));
%     
%     UstandardNormAccumulated = cumsum(UstandardNorm, 2);
%     UprojectedNormAccumulated = cumsum(UprojectedNorm, 2);
%     UrelaxedNormAccumulated = cumsum(UrelaxedNorm, 2);
    
    Ustandard = squeeze(U(:,:,:,:,1,:,:));
    Uprojected = squeeze(U(:,:,:,:,2,:,:));
    Urelaxed = squeeze(U(:,:,:,:,3,:,:));
    
    if length(N) == 1
        offset=1;
    else
        offset=0;
    end
    % sum
    UstandardSum = squeeze(sum(mean(squeeze(Ustandard), 2-offset), 4-offset));
    UprojectedSum = squeeze(sum(mean(squeeze(Uprojected), 2-offset), 4-offset));
    UrelaxedSum = squeeze(sum(mean(squeeze(Urelaxed), 2-offset), 4-offset));
    
    UstandardNormAccumulated = cumsum(UstandardSum, 2);
    UprojectedNormAccumulated = cumsum(UprojectedSum, 2);
    UrelaxedNormAccumulated = cumsum(UrelaxedSum, 2);
    
    % Average over all variables
    tstandardmean = mean(tstandard(tstandard~=0), 'all');
    tprojectedmean = mean(tprojected(tprojected~=0), 'all');
    trelaxedmean = mean(trelaxed(trelaxed~=0), 'all');
    
    Eprojectedmean = mean(Eprojected, 'all');
    Erelaxedmean = mean(Erelaxed, 'all');
    
    % Average over N, tmin, dt, rerunID
    tstandardNTDmean = mean(tstandard, [1,2,3,4]);
    tprojectedNTDmean = mean(tprojected, [1,2,3,4]);
    trelaxedNTDmean = mean(trelaxed, [1,2,3,4]);
    
    disp('Successfully loaded all simulations');
    
    %% Plotting
    imgsdir = 'imgs/';
    imgssubdir = 'benchmarks/';
    if ~exist(imgsdir, 'dir')
        mkdir(imgsdir);
    end
    if ~exist([imgsdir, imgssubdir], 'dir')
        mkdir([imgsdir, imgssubdir]);
    end
    impath = @(name) [imgsdir, imgssubdir, sprintf('%s%s', name, suffix)];
    
    standardColor = [0, 0.4470, 0.7410];
    projectedColor = [0.8500, 0.3250, 0.0980];
    relaxedColor = [0.9290, 0.6940, 0.1250];
    
    sc = {'MarkerFaceColor', standardColor, 'MarkerEdgeColor', standardColor, 'Color', standardColor};
    pc = {'MarkerFaceColor', projectedColor, 'MarkerEdgeColor', projectedColor, 'Color', projectedColor};
    rc = {'MarkerFaceColor', relaxedColor, 'MarkerEdgeColor', relaxedColor, 'Color', relaxedColor};
    histMeanLine = {'LineWidth', 0.5, 'LineStyle', '-.'};
    finishLine = {'LineWidth', 1, 'LineStyle', '-.'};
    finishLineAlpha = 0.8;
    meanLine = {'LineWidth', 2, 'LineStyle', '-'};
    minmaxLine = {'LineWidth', 0.5, 'LineStyle', ':'};
    
    bigFont = true;

    % Plot solver times histograms
    nbins = 5000;
    % binLimits = [min(t(:)), max(t(:))];
    binLimits = [1e-4, 2e0];
    edges = 10.^linspace(log10(binLimits(1)),log10(binLimits(2)), 500);
    h = figure(1);
    clf;
    if bigFont
        set(h, "DefaultAxesFontSize", 12);
        set(h, "DefaultTextFontSize", 12);
    end

    hold on;
    % opts = {'Normalization', 'probability', 'EdgeColor', 'none'};
    opts = {'EdgeColor', 'none'};
    histogram(squeeze(tstandard), edges, opts{:});
    histogram(squeeze(tprojected), edges, opts{:});
    histogram(squeeze(trelaxed), edges, opts{:});
    ylimits = ylim;
    % plot a vertical line at mean
    plot([tstandardmean, tstandardmean], ylimits, sc{:}, histMeanLine{:});
    plot([tprojectedmean, tprojectedmean], ylimits, pc{:}, histMeanLine{:});
    plot([trelaxedmean, trelaxedmean], ylimits, rc{:}, histMeanLine{:});
    hold off;
    ylim tight;
    xLimits = [min([tstandard, tprojected, trelaxed], [], 'all'), binLimits(2)];
    xlim(xLimits);
    legend('Standard', 'Projected', 'Relaxed', 'Mean');
    xlabel('Solve Time (s)');
    ylabel('Count');
    set(gca,'xscale','log');
    setFigSize(gcf, 14, 4.5);
    benchmarksFigStylingAndSave(gcf, impath('solverTimesHistograms'));
    
    % Plot solver times, mean line along steps, max line along steps, min line
    h = figure(2);
    clf;
    if bigFont
        set(h, "DefaultAxesFontSize", 12);
        set(h, "DefaultTextFontSize", 12);
    end
    hold on;
    plot(simt, mean(tstandard, 1), sc{:}, meanLine{:});
    plot(simt, mean(tprojected, 1), pc{:}, meanLine{:});
    plot(simt, mean(trelaxed, 1), rc{:}, meanLine{:});
    plot(simt, max(tstandard, [], 1), sc{:}, minmaxLine{:});
    plot(simt, max(tprojected, [], 1), pc{:}, minmaxLine{:});
    plot(simt, max(trelaxed, [], 1), rc{:}, minmaxLine{:});
    plot(simt, min(tstandard, [], 1), sc{:}, minmaxLine{:});
    plot(simt, min(tprojected, [], 1), pc{:}, minmaxLine{:});
    plot(simt, min(trelaxed, [], 1), rc{:}, minmaxLine{:});
    set(gca, 'YScale', 'log');
    % vertical line at finish time for each method
    ylimits = ylim;
    fs = plot([fTimeStandardMean, fTimeStandardMean], ylimits, sc{:}, finishLine{:});
    fp = plot([fTimeProjectedMean, fTimeProjectedMean], ylimits, pc{:}, finishLine{:});
    fr = plot([fTimeRelaxedMean, fTimeRelaxedMean], ylimits, rc{:}, finishLine{:});
    fs.Color(4) = finishLineAlpha;
    fp.Color(4) = finishLineAlpha;
    fr.Color(4) = finishLineAlpha;
    xlim tight;
    hold off;
    
    legend('Standard', 'Projected', 'Relaxed', '', '', '', '', '', '', 'Finish', 'Location', 'northwest');
    xlabel('Simulation Time (s)');
    ylabel('Solve Time (s)');
    setFigSize(gcf, 14, 4.5);
    benchmarksFigStylingAndSave(gcf, impath('solverTimesAlongSteps'));
    
    % Plot error norm along steps (EprojectedNorm, ErelaxedNorm)
    h=figure(3);
    clf;
    if bigFont
        set(h, "DefaultAxesFontSize", 12);
        set(h, "DefaultTextFontSize", 12);
    end
    hold on;
    plot(simt, mean(EprojectedNorm, 1), pc{:}, meanLine{:});
    plot(simt, mean(ErelaxedNorm, 1), rc{:}, meanLine{:});
    set(gca, 'YScale', 'log');
    % vertical line at finish time for each method
    ylimits = ylim;
    fp = plot([fTimeProjectedMean, fTimeProjectedMean], ylimits, pc{:}, finishLine{:});
    fr = plot([fTimeRelaxedMean, fTimeRelaxedMean], ylimits, rc{:}, finishLine{:});
    fp.Color(4) = finishLineAlpha;
    fr.Color(4) = finishLineAlpha;
    hold off;
    xlim tight;
    
    legend('Projected', 'Relaxed', 'Location', 'southwest');
    xlabel('Simulation Time (s)');
    ylabel('Absolute Error (m)');
    setFigSize(gcf, 14, 4.5);
    benchmarksFigStylingAndSave(gcf, impath('errorNormAlongSteps'));
    
    % Plot control norm integral along steps (UprojectedNorm, UrelaxedNorm)
    h=figure(4);
    clf;
    if bigFont
        set(h, "DefaultAxesFontSize", 12);
        set(h, "DefaultTextFontSize", 12);
    end
    hold on;
    plot(simt, mean(UstandardNormAccumulated, 1), sc{:}, meanLine{:});
    plot(simt, mean(UprojectedNormAccumulated, 1), pc{:}, meanLine{:});
    plot(simt, mean(UrelaxedNormAccumulated, 1), rc{:}, meanLine{:});
%     plot(simt, max(UstandardNormAccumulated, [], 1), sc{:}, minmaxLine{:});
%     plot(simt, max(UprojectedNormAccumulated, [], 1), pc{:}, minmaxLine{:});
%     plot(simt, max(UrelaxedNormAccumulated, [], 1), rc{:}, minmaxLine{:});
    % vertical line at finish time for each method
    ylimits = ylim;
    fs = plot([fTimeStandardMean, fTimeStandardMean], ylimits, sc{:}, finishLine{:});
    fp = plot([fTimeProjectedMean, fTimeProjectedMean], ylimits, pc{:}, finishLine{:});
    fr = plot([fTimeRelaxedMean, fTimeRelaxedMean], ylimits, rc{:}, finishLine{:});
    fs.Color(4) = finishLineAlpha;
    fp.Color(4) = finishLineAlpha;
    fr.Color(4) = finishLineAlpha;
    hold off;
    xlim tight;
    
    if N > 7
        legend('Standard', 'Projected', 'Relaxed', 'Finish', 'Location', 'southeast');
    else
        legend('Standard', 'Projected', 'Relaxed', 'Finish', 'Location', 'northwest');
    end
    xlabel('Simulation Time (s)');
    ylabel('Cumulative Control Effort (s)');
    setFigSize(gcf, 14, 4.5);
    benchmarksFigStylingAndSave(gcf, impath('controlNormIntegralAlongSteps'));
    
    %% 
    for i = 1:length(N)
        fprintf("N = %d\n", N(i));
        for j = 1:length(method)
            meanIterTime = mean(t(i,:,:,:,j,:), 'all');
            iterTime95 = prctile(t(i,:,:,:,j,:), 95, 'all');
            iterTime99 = prctile(t(i,:,:,:,j,:), 99, 'all');
            fprintf("\\SI{%0.2f}{\\milli\\second} & \\SI{%0.2f}{\\milli\\second} & \\SI{%0.2f}{\\milli\\second} \\\\\n", 1000*meanIterTime, 1000*iterTime95, 1000*iterTime99);
        end
    end
end


