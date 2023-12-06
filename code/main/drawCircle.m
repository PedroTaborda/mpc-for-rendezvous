function line = drawCircle(ax, center, radius, plotOptions, N, fillCircle, fillOptions)
    if nargin < 4
        N=100;
    end
    if nargin < 3
        plotOptions = {};
    end
    t = linspace(0, 2*pi, N);
    
    x = radius*cos(t) + center(1);
    y = radius*sin(t) + center(2);

    line = plot(x, y, plotOptions{:});

    if nargin > 5 && fillCircle
        if nargin < 7
            fillOptions = {};
        end
        fillOptions = [fillOptions, 'FaceColor', get(line, 'Color')];
        fillOptions = [fillOptions, 'EdgeColor', 'none'];
        fillOptions = [fillOptions, 'FaceAlpha', 0.2];
        color = get(line, 'Color');
        fill(x, y, color, fillOptions{:});
    end
end

