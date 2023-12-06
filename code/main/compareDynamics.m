
cfg = params();

cfg.controller.N = 20;
cfg.simulation.dt = 10;
cfg.controller.tmin = 1;
cfg.simulation.T = cfg.simulation.dt*2000;
cfg.simulation.steps = ceil(cfg.simulation.T/cfg.simulation.dt);

dt = cfg.simulation.dt;
T = cfg.simulation.T;
x0 = cfg.simulation.x0;
xref = cfg.simulation.xTarget;
method = cfg.simulation.method;


t = 0:dt:T;
XNewtonLVLH = zeros(length(x0), length(t) + 1);
XNewtonECI = zeros(length(x0), length(t) + 1);
Xcw = zeros(length(x0), length(t) + 1);
Xcwlin = zeros(length(x0), length(t) + 1);

iterationTime = zeros(1, length(t));
XNewtonLVLH(:,1) = x0;
XNewtonECI(:,1) = x0;
Xcw(:,1) = x0;
Xcwlin(:,1) = x0;

u = zeros(length(x0), 1);
fDynamicscwlin = @(x, tThrusters) cfg.system.dynamics.fLinT(x, tThrusters, cfg.controller.tlin);

for i = 2:length(t)+1
    % simulate for one time step
    [tR, yR] = ode45(@(tO, xO) newtonDynamics(tO, xO, t(i-1), u, cfg), [t(i-1), t(i-1)+dt], XNewtonLVLH(:, i-1));
    XNewtonLVLH(:, i) = yR(end, :)';
    
    [tR, yR] = ode45(@(tO, xO) newtonDynamicsECI(tO, xO, t(i-1), u, cfg), [t(i-1), t(i-1)+dt], lvlh2eci(cfg, XNewtonECI(:, i-1), t(i-1)));
    XNewtonECI(:, i) = eci2lvlh(cfg, yR(end, :)', t(i-1)+dt);
    
    Xcw(:, i) = cfg.system.dynamics.fCW(Xcw(:, i-1), u);
    
    Xcwlin(:, i) = fDynamicscwlin(Xcwlin(:, i-1), u);
end

figure(1); clf;
subplot(2, 2, 1);
hold on;
plot(t, XNewtonECI(1, 1:end-1), 'x');
% plot(t, XNewtonLVLH(1, 1:end-1), 'x');
plot(t, Xcw(1, 1:end-1), 'x');
plot(t, Xcwlin(1, 1:end-1), 'x');
% legend('NewtonECI', 'NewtonLVLH', 'CW', 'CW linearized');
legend('NewtonECI', 'CW', 'CW linearized');
subplot(2, 2, 3);
% plot(t, XNewtonECI(3, 1:end-1), 'x', t, XNewtonLVLH(3, 1:end-1), 'x', t, Xcw(3, 1:end-1), 'x', t, Xcwlin(3, 1:end-1), 'x');
hold on;
plot(t, XNewtonECI(3, 1:end-1), 'x');
% plot(t, XNewtonLVLH(3, 1:end-1), 'x');
plot(t, Xcw(3, 1:end-1), 'x');
plot(t, Xcwlin(3, 1:end-1), 'x');
% legend('NewtonECI', 'NewtonLVLH', 'CW', 'CW linearized');
legend('NewtonECI', 'CW', 'CW linearized');

subplot(2, 2, [2, 4]);
% plot(XNewtonECI(1, 1:end-1), XNewtonECI(3, 1:end-1), 'x', XNewtonLVLH(1, 1:end-1), XNewtonLVLH(3, 1:end-1), 'x', Xcw(1, 1:end-1), Xcw(3, 1:end-1), 'x', Xcwlin(1, 1:end-1), Xcwlin(3, 1:end-1), 'x');
hold on;
plot(XNewtonECI(1, 1:end-1), XNewtonECI(3, 1:end-1), 'x');
% plot(XNewtonLVLH(1, 1:end-1), XNewtonLVLH(3, 1:end-1), 'x');
plot(Xcw(1, 1:end-1), Xcw(3, 1:end-1), 'x');
plot(Xcwlin(1, 1:end-1), Xcwlin(3, 1:end-1), 'x');
% legend('NewtonECI', 'NewtonLVLH', 'CW', 'CW linearized');
axis equal;
legend('NewtonECI', 'CW', 'CW linearized');

