function results = massSimulate()
    % Creates a batch of cfg for the given parameters for (parallel) simulating
    
    N = [5, 10, 20, 50]; % MPC controller horizon
%     dt = [5, 10, 20]; % simulation step size
    dt = [5, 10, 20];
    method = {'standard', 'projected', 'relaxed'}; % MPC solver method
    rerunID = 0:10000;
    
    tmin = [1, 2];
    
    cfgDefault = params();
    cfgBatch = cell(1, 0);
    
    verbosity = 0;
    cfgDefault.simulation.verbosity = verbosity;
    
    for n = 1:length(rerunID)
        for i = 1:length(N)
            for k = 1:length(dt)
                for l = 1:length(method)
                    for m = 1:length(tmin)
                        cfg = cfgDefault;
                        cfg.controller.N = N(i);
                        cfg.controller.tmin = tmin(m);
                        cfg.simulation.dt = dt(k);
                        cfg.simulation.steps = ceil(cfg.simulation.T/dt(k));
                        cfg.simulation.method = method{l};
                        cfg.simulation.rerun = rerunID(n);
                        
                        cfgBatch{end+1} = cfg;
                    end
                end
            end
        end
    end
    
    % Simulate the batch of cfgs
    results = simulateBatch(cfgBatch, 1);
    disp('Batch simulation over');
end