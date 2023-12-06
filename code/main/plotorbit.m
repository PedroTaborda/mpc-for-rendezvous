function h = plotorbit(axes, cfg, X, opts)
    xOrbit = lvlh2eci(cfg, zeros(size(X)));
    h = plot(axes, xOrbit(1, :), xOrbit(3, :), opts{:});
end
