function E = energySpent(S, cfg)
%ENERGYSPENT Calculates the energy spent by a given control sequence at
% each time step
%   S is the actuator opening times in seconds
    F = cfg.system.F; % max force of each actuator

    M = length(F); % number of actuators
    szS = size(S);
    N = szS(2); % number of time steps
    % M should also be equal to szS(1)

    E = zeros(1, N);
    for i = 1:N
        % E(i) = sum(F .* S(:, i));
        E(i) = sum(S(:, i));
    end
end
    