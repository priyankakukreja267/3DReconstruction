
i1 = imread('../data/im1.png');
i1 = im2double(i1);
i1 = rgb2gray(i1);

i2 = imread('../data/im2.png');
i2 = im2double(i2);
i2 = rgb2gray(i2);

M = max(size(i1, 1), size(i1, 2))

% Check eightpoint algo
load('../data/some_corresp.mat');
eightF = eightpoint(pts1, pts2, M);
epipolarMatchGUI(i1, i2, eightF);
% displayEpipolarF(i1, i2, eightF);

% Check sevenpoint algo
% load('../results/manualPoints.mat');
% sevenF = sevenpoint(fixedPoints, movingPoints, M);
% displayEpipolarF(i1, i2, sevenF);
% displayEpipolarF(i1,i2,sevenF(:,:,1));
% displayEpipolarF(i1,i2,sevenF(:,:,2));
% displayEpipolarF(i1,i2,sevenF(:,:,3));

% Check RANSAC algo
load('../data/some_corresp_noisy.mat');
ranF = ransacF(pts1, pts2, M);
% displayEpipolarF(i1, i2, ranF);

% Get Essential Matrix E
load('../data/intrinsics.mat');
F = eightF;
% E = K2' * F * K1
E = essentialMatrix(F, K1, K2);

% Find the correct M2 by triangulating a set of 3D points
M1 = K1*[eye(3), zeros(3,1)];
M2s = camera2(E);

load('../data/some_corresp.mat');
M2 = findM2(M2s, pts1, pts2);
%     M1 = K1*M1;

M2 = K2*M2;

[P, ] = triangulate(M1, pts1, M2, pts2);

% K1, K2 - internal camera matrix 3X3
% M1, M2 = total calibration matrix (4X1 world to 3X1 image)
% M1 = K1 * [I|0] * [R|t];

% Find the corresponding points for the temple
load('../data/templeCoords.mat');
[x2, y2] = epipolarCorrespondence(i1, i2, F, x1, y1);
% epipolarMatchGUI(i1, i2, F);
p1 = [x1, y1];
p2 = [x2; y2]';

% Find the corresponding 3D points
[P, ] = triangulate(M1, p1, M2, p2);
% Visualize them
scatter3(P(:,1), P(:,2), P(:,3), 'red', 'filled');
