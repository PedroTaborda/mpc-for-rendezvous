function Xeci = doPlots(cfg, X, S, figNumOffset)
if nargin < 4
    figNumOffset = 0;
end

% plot trajectory in LVLH frame
figure(figNumOffset + 1);
clf;
plot3(X(1, :), X(2, :), X(3, :));
xlabel("x");
ylabel("y");
zlabel("z");
title("Trajectory LVLH (m)");

% plot trajectory in ECI frame (circular orbit, keplerian)
Xeci = lvlh2eci(cfg, X);
xOrbit = lvlh2eci(cfg, zeros(size(X)));

p0Orbit = xOrbit(:, 1);
p0ECInewton = Xeci(:, 1);
pfOrbit = xOrbit(:, end);
pfECInewton = Xeci(:, end);

% plot trajectory in ECI frame, in orbital plane
figure(figNumOffset + 2);
clf;
hold on;
color2 = [0.8500, 0.3250, 0.0980];
color3 = [0.9290, 0.6940, 0.1250];
plot(Xeci(1, :), Xeci(3, :), "Color", color2);
plot(xOrbit(1, :), xOrbit(3, :), "Color", color3);
plot(p0ECInewton(1), p0ECInewton(3), "o", "Color", color2);
plot(pfECInewton(1), pfECInewton(3), "x", "Color", color2);
plot(p0Orbit(1), p0Orbit(3), "o", "Color", color3);
plot(pfOrbit(1), pfOrbit(3), "x", "Color", color3);
xlabel("x");
ylabel("z");
title("Trajectory ECI (m)");
legend("Trajectory", "Circular Orbit", "Trajectory Start", "Trajectory End", "Orbit Start", "Orbit End");
axis equal;
end