function outputBatch = simulateBatch(cfgBatch, nThreads)
%SIMALL Simulates each and every cfg in the cfgBatch cell list
% Returns a cell list with the output of each corresponding simulation
% cfgBatch: cell list of cfgs
% outputBatch: cell list of outputs
% nThreads: number of threads to use (default: 1)

outputBatch = cell(size(cfgBatch));

if nargin < 2
    nThreads = 1;
end

N = length(cfgBatch);

if nThreads == 1
    w = waitbar(0, 'Simulating...');
    for i = 1:N
        outputBatch{i} = simulate(cfgBatch{i});
        waitbar(i/N, w, sprintf('Simulating... %d/%d (rerun=%d)', i, N, cfgBatch{i}.simulation.rerun));
    end
    close(w);
else
    parfor (i = 1:N, nThreads)
        outputBatch{i} = simulate(cfgBatch{i});
%         fprintf('Simulation progress: %d/%d\n', i, N);
    end
end
end

