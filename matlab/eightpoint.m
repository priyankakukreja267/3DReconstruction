function [ F ] = eightpoint( pts1, pts2, M )
% eightpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

nPts = size(pts1, 1);
x = pts1(:,1)./M;
y = pts1(:,2)./M;
xp = pts2(:,1)./M;
yp = pts2(:,2)./M;
T = [1/M 0 0; 0 1/M 0; 0 0 1];

% find F
lastCol = ones(nPts, 1);
% A = [xp.*x xp.*y xp yp.*x yp.*y yp x y lastCol];
A = [x.*xp, x.*yp, x, y.*xp, y.*yp, y, xp, yp, lastCol];

B = A'*A;
[V, D] = eig(B);
eigVector = V(:,1); % get the eigVector of smallest eigValue
F = reshape(eigVector, [3,3]); % reshape to get F
F = F';

% [U, S, V] = svd(A);
% f = V(:,9);
% F = reshape(f, [3,3]);
% F = F';

% enforce singularity
[W, D, V] = svd(F);
D(3, 3) = 0;
F = W*D*(V');

% refine F
% F = refineF(F, [xp, yp], [x, y]);
F = refineF(F, [x, y], [xp, yp]); 
% unscale the data
F = T'*F*T;

save('../results/q2_1.mat', 'F', 'M', 'pts1', 'pts2');
end
