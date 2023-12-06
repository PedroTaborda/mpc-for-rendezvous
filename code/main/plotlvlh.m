function h = plotlvlh(axes, cfg, X, opts)
if nargin < 4
    opts = {};
end
h = plot(axes, X(1, :), X(3, :), opts{:});
end
