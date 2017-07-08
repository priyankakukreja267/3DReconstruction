function [ P, error ] = triangulate( M1, p1, M2, p2)
% triangulate:
%       M1 - 3x4 Camera Matrix 1
%       p1 - Nx2 set of points
%       M2 - 3x4 Camera Matrix 2
%       p2 - Nx2 set of points

% Multiply M1 with K1, and M2 with K2 before starting

% Cross Product Matrix of v1:
% [0    -v1z   v1y
%  v1z   0    -v1x
% -v1y   v1x    0]
    load('../data/intrinsics.mat');

    tempP1 = p1;
    tempP2 = p2;
    
    nPts = size(p1, 1);
    p1 = p1';
    p1 = [p1; ones(1, nPts)];

    p2 = p2';
    p2 = [p2; ones(1, nPts)];

    P = zeros(4, nPts);
    
    for i = 1:nPts
        pt1 = p1(:,i);
        pt2 = p2(:,i);
        u1 = [0        -pt1(3)  pt1(2); ...
              pt1(3)   0        -pt1(1); ...
              -pt1(2)  pt1(1)   0];

        u2 = [0       -pt2(3)  pt2(2); ...
             pt2(3)   0        -pt2(1); ...
             -pt2(2)  pt2(1)   0];
        A = [u1*M1; u2*M2]; % 6X4
        [W, S, V] = svd(A);
        P(:,i) = V(:,4)';
        P(:,i) = (P(:,i))./(P(4,i));
    end
P = P(1:3,:)';
% error = findReprojectionError(P, tempP1, tempP2, M1, M2);
end

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
%         R = R + sum( sqrt( ((p1(:,i) - p1hat).^2 + (p2(:,i) - p2hat).^2) ) );
        error = (sqrt(sum(p1(:,i) - p1hat).^2) + sqrt(sum(p2(:,i) - p2hat).^2)) / sqrt(sum(P(:,i).*2));
        R = R + error;
    end
end