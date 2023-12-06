
G = 6.674e-11; % N m^2 / kg^2
M = 5.972e24;  % kg
R = 6371e3;    % m
mu = G*M;      % m^3 / s^2
mt = 3000;     % kg
mc = 2000;     % kg

altTarget = 800e3;            % m
rT = R+altTarget;             % m
w = sqrt(mu/(R+altTarget)^3); % rad/s
T = 2*pi/w;                   % s

% initial conditions
alt0 = 100e3; % m
dalt = altTarget-alt0; % m

x0 = [-dalt; 0; 0; 0; 0; 0];
dt = 1;
Tsim = dt*3500;

% CW equations (for controller)
Ac = [
    zeros(3, 3), eye(3);
    diag([0, -w*w, 3*w*w]), fliplr(diag([2*w, 0, -2*w]))
];

Ad = expm(Ac*dt);

G = @(s) integral(@(T) expm(Ac*T), 0, s, "ArrayValued", true);

B = [
    zeros(3, 3);
    eye(3)/mc
];

% Newton equations (for simulating dynamics)
A = [
    zeros(3, 3), eye(3);
    -mu/(R+alt0)^3, 0, 0, 0, 2*w, 0;
    0, 0, 0, -2*w, 0, 0;
    0, 0, 0, 0, 0, 0;
];

Adnewton = expm(A*dt);



inputVec = [0; 0; 0];

% simulate
t = 0:dt:Tsim;
xcw = zeros(6, length(t));
xnewton = zeros(6, length(t));
xcw(:, 1) = x0;
xnewton(:, 1) = x0;
for i = 2:length(t)
    xcw(:, i) = Ad*xcw(:, i-1); % + Ad*G(dt)*B*inputVec;
    xnewton(:, i) = Adnewton*xnewton(:, i-1) + B*inputVec;
end

% plot trajectory in LVLH frame
figure(1);
plot3(xcw(1, :), xcw(2, :), xcw(3, :));
hold on;
plot3(xnewton(1, :), xnewton(2, :), xnewton(3, :));
xlabel("x");
ylabel("y");
zlabel("z");
title("Trajectory LVLH (m)")
legend("CW", "Newton");

% plot trajectory in ECI frame (circular orbit, keplerian)
xECIcw = zeros(3, length(t));
xECInewton = zeros(3, length(t));

% orbit is on xz plane
xOrbit = zeros(3, length(t));

for i = 1:length(t)
    % apply rotation and shift
    rotMat = [cos(w*t(i)), 0, sin(w*t(i)); 0, 1, 0; -sin(w*t(i)), 0, cos(w*t(i))];
    posAlongOrbit = [rT*cos(w*t(i)); 0; rT*sin(w*t(i))];
    xOrbit(:, i) = posAlongOrbit;
    xECIcw(:, i) = rotMat*xcw(1:3, i) + posAlongOrbit;
    xECInewton(:, i) = rotMat*xnewton(1:3, i) + posAlongOrbit;
end
p0Orbit = xOrbit(:, 1);
p0ECIcw = xECIcw(:, 1);
p0ECInewton = xECInewton(:, 1);
pfOrbit = xOrbit(:, end);
pfECIcw = xECIcw(:, end);
pfECInewton = xECInewton(:, end);

% plot trajectory in ECI frame, in orbital plane
figure(2);
clf;
hold on;
color1 = [0, 0.4470, 0.7410];
color2 = [0.8500, 0.3250, 0.0980];
color3 = [0.9290, 0.6940, 0.1250];
plot(xECIcw(1, :), xECIcw(3, :), "Color", color1);
plot(xECInewton(1, :), xECInewton(3, :), "Color", color2);
plot(xOrbit(1, :), xOrbit(3, :), "Color", color3);
plot(p0ECIcw(1), p0ECIcw(3), "o", "Color", color1);
plot(pfECIcw(1), pfECIcw(3), "x", "Color", color1);
plot(p0ECInewton(1), p0ECInewton(3), "o", "Color", color2);
plot(pfECInewton(1), pfECInewton(3), "x", "Color", color2);
plot(p0Orbit(1), p0Orbit(3), "o", "Color", color3);
plot(pfOrbit(1), pfOrbit(3), "x", "Color", color3);
xlabel("x");
ylabel("z");
title("Trajectory ECI (m)");
legend("Trajectory CW", "Trajectory Newton", "Orbit", "Start CW", "End CW", "Start Newton", "End Newton", "Start Orbit", "End Orbit");
axis equal;



