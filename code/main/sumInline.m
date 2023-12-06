function sumVal = sumInline(f,arr)
%SUMINLINE Computes f(arr(1)) + ... + f(arr(end))
    sumVal = 0;
    for ii = 1:length(arr)
        sumVal = sumVal + f(arr(ii));
    end
end

