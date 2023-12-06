function data = cfgData(cfg)
% CFGDATA returns the cfg item stripped of all non-data fields (function handles, etc.)
    data = cfg;
    data.system.dynamics = [];
    data.simulation.verbosity = 0;
end
