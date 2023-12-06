function renew_mcf()
    dirname = 'cache';
    if ~exist(dirname, 'dir')
        mkdir(dirname);
    end
    master_cache_file = sprintf('%s/master_cache.mat', dirname);
    % delete master cache file
    if exist(master_cache_file, 'file')
        delete(master_cache_file);
    end
    
    % create new master cache file
    cached_simulations = containers.Map('KeyType', 'char', 'ValueType', 'any');

    % iterate through all the files in the cache directory
    files = dir(dirname);

    for i = 1:length(files)
        file = files(i);
        if ~file.isdir && ~strcmp(file.name, 'master_cache.mat')
            % load the cache file
            load(sprintf('%s/%s', dirname, file.name), 'cfg');
            % add the simulation to the master cache
            key = cfgRepr(cfg);
            cached_simulations(key) = struct('cfg', cfg, 'file', sprintf("%s/%s", dirname, file.name));
            disp(file.name);
        end
    end

    % save the master cache file
    save(master_cache_file, 'cached_simulations');
end
