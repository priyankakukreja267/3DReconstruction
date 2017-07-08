function R = findReprojectionError(P, p1, p2, M1, M2)
    % P = NX3
    % p1 = NX2
    % p2 = NX2

    nPts = size(p1, 1);

    p1 = p1';
    p1 = [p1; ones(1, nPts)];

    p2 = p2';
    p2 = [p2; ones(1, nPts)];

    P = P';
    P = [P; ones(1, nPts)];

    R = 0;
    for i = 1:nPts
        p1hat = M1*P(:,i); % p1hat = 3X1
        p2hat = M2*P(:,i); % p2hat = 3X1
        R = R + sqrtsum( ((p1(:,i) - p1hat).^2 + (p2(:,i) - p2hat).^2) );
    end
end