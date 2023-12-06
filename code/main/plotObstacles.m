function circlesWereDrawn = plotObstacles(ax,cfg, plotOptions)
%PLOTOBSTACLES plots the obstacles contained in cfg
    circlesWereDrawn = false;
    if ~endsWith(cfg.simulation.method, '-o')
        return;
    end
    for cc = 1:numel(cfg.simulation.obstacles)
        obstacle = cfg.simulation.obstacles{cc};
        fill = true;
        drawCircle(ax, obstacle.center, obstacle.radius, plotOptions, 200, fill);
    end
    circlesWereDrawn = true;
end

