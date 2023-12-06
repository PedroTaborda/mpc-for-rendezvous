function name = cfgFileName(cfg)
%CFGFILENAME produces a valid filename from a given cfg, mostly but not
%guaranteed unique
startString = cfgRepr(cfg);
% get a seed by summing the ascii values of the characters
seed = sum(startString);
name = sprintf('%d', startString, seed);
end