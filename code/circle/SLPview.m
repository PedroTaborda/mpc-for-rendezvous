

p = [0.25, .25]
r = 0.5


% points on a line from [0.1, 0.85] to [0.9, 0.15]
t = linspace(0, 1, 10)
trajectoryUnc = [0.1, 0.85] + t'*([0.9, 0.15] - [0.1, 0.85])


figure(1); clf; hold on; axis equal; axis([0, 1, 0, 1]); grid on; box on;

for i = 1:length(trajectoryUnc)
    if norm(trajectoryUnc(i, :) - p) < r
        [linePoints, pNearest, v] = getLinePoints(p, r, trajectoryUnc(i, :))
        plot(linePoints(:, 1), linePoints(:, 2), 'Color', '#7E2F8E', 'LineWidth', 0.5)
        plot(pNearest(1), pNearest(2), 'Color', '#EDB120', 'LineWidth', 1, 'Marker', 'o')
        plot(trajectoryUnc(i, 1), trajectoryUnc(i, 2), 'Color', '#D95319', 'LineWidth', 0.5, 'Marker', 'o')

        % plot the arrow on top of the line
        q = quiver(trajectoryUnc(i, 1), trajectoryUnc(i, 2), pNearest(1) - trajectoryUnc(i, 1), pNearest(2) - trajectoryUnc(i, 2), 'Color', '#D95319')
    else
        plot(trajectoryUnc(i, 1), trajectoryUnc(i, 2), 'Color', '#0072BD', 'LineWidth', 1, 'Marker', 'o')
    end
end

% plot the circle
drawCircle(p, r, {'k-'})

set(gca, 'XTickLabel', {}, 'YTickLabel', {})
print('slpExample.pdf', '-dpdf')

function [linePoints, pNearest, v] = getLinePoints(p, r, xInf)
%GETLINEPOINTS Get two points on the tangent line to a circle in the
%nearest point to a point inside the circle

% p: center of the circle
% r: radius of the circle
% xInf: point inside the circle

% linePoints: two points on the tangent line
% pNearest: nearest point on the circle to xInf
% v: direction of the tangent line

    v = (xInf - p)/norm(xInf - p)
    pNearest = p + r*v
    linePoints = [pNearest - [v(2), -v(1)]; pNearest + [v(2), -v(1)]]
end