function line = drawCircle(center, radius, plotOptions, N)
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
end

