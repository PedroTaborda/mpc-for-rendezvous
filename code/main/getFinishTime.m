function fTime = getFinishTime(X, dist)
%getFinishTime Retrieves the rendezvous finishing step fTime, such that the
% distance to the target dTarget(fTime)<dist for any idx>fTime
% fTime the index (not a time value) of the last simulation step where
% dist was exceeded
if nargin < 2
    dist = 1000;
end
sz = size(X);
M = min(3, sz(1));
dists = vecnorm(X(1:M, :));
notOver = find(dists>dist);
if isempty(notOver)
    fTime = 1;
    return
end
fTime = notOver(end);
end