% returns the SSD between 2 intensity vectors
function d = getDistance(w1, w2)
    w1Len = size(w1);
    w2Len = size(w2);
    if w1Len < w2Len
        v1 = [w1 zeros(w2Len-w1Len)];
        v2 = w2;
    else
        v1 = w1;
        v2 = [w2 zeros(w1Len-w2Len)];
    end
    d = sum((v1 - v2).^2);
end
