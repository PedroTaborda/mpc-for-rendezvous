function h = ploteci(axes, cfg, X, opts)
if nargin < 4
    opts = {};
end
Xeci = lvlh2eci(cfg, X, 0); 
h = plot(axes, Xeci(1, :), Xeci(3, :), opts{:});
end
