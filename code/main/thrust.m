function t = thrust(S, w, F)
%THRUST Get thrust from a given time, direction and force vectors
%
%   S: time actuators are on
%   w: direction of thrust
%   F: force of thrust
%
    szS = size(S);
    szw = size(w);
    szF = size(F);
    N = szS(2);

    t = zeros(3, N);
    for i = 1:N
        taux = [0, 0, 0];
        for j = 1:szw(1)
            taux = taux + S(j, i) * w(j, :) * F(j);
        end
        t(:, i) = taux;
    end


end

