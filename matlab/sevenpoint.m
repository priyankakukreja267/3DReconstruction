function [ F ] = sevenpoint( pts1, pts2, M )
% sevenpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% scale data by dividing all points by M
nPts = size(pts1, 1);
x = pts1(:,1)./M;
y = pts1(:,2)./M;
xp = pts2(:,1)./M;
yp = pts2(:,2)./M;
T = [1/M 0 0; 0 1/M 0; 0 0 1];

% find F
lastCol = ones(nPts, 1);
A = [x.*xp x.*yp x y.*xp y.*yp y xp yp lastCol];
[U, S, V] = svd(A);
f1 = V(:,8);
f2 = V(:,9);

f1 = reshape(f1, [3,3])';
f2 = reshape(f2, [3,3])';

% find the equation for Det(f)
syms k
f = (1-k)*f1 + k*f2;
kAns = solve(det(f) == 0, k);
kArray = real(double(kAns));

F = zeros(3,3,3);

for i = 1:3
    F(:,:,i) = (1-kArray(i))*f1 + kArray(i)*f2;
 
    % refine F
    F(:,:,i) = refineF(F(:,:,i), [xp, yp], [x, y]);

    % unscale the data
    F(:,:,i) = T'*F(:,:,i)*T;
end

save('../results/q2_2.mat', 'F', 'M', 'pts1', 'pts2');
end
