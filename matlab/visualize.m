% Todo:
% Integrating everything together.
% Loads necessary files from ../data/ and visualizes 3D reconstruction
% using scatter3

function [] = visualize()
    %  Load the images
    i1 = imread('../data/im1.png');
    i1 = im2double(i1);
    i1 = rgb2gray(i1);

    i2 = imread('../data/im2.png');
    i2 = im2double(i2);
    i2 = rgb2gray(i2);

    M = max(size(i1, 1), size(i1, 2));

    % Find F using eightpoint algo
    load('../data/some_corresp.mat');
    F = eightpoint(pts1, pts2, M);

    % Get Essential Matrix E
    load('../data/intrinsics.mat');
    % E = K2' * F * K1
    E = essentialMatrix(F, K1, K2);
    
    % Find the correct M2 by triangulating a set of 3D points
    % K1, K2 - internal camera matrix 3X3
    % M1, M2 = total calibration matrix (4X1 world to 3X1 image)
    % M1 = K1 * [I|0] * [R|t];

    load('../data/some_corresp.mat');
    M2s = camera2(E);
    M2 = findM2(M2s, pts1, pts2);
    M1 = [eye(3), zeros(3,1)];
    [P, ] = triangulate(K1*M1, pts1, K2*M2, pts2);

    % Find the corresponding points for the temple
    load('../data/templeCoords.mat');
    [x2, y2] = epipolarCorrespondence(i1, i2, F, x1, y1);
    p1 = [x1, y1];
    p2 = [x2; y2]';

    % Find the corresponding 3D points
    [P, ] = triangulate(K1*M1, p1, K2*M2, p2);

    % Visualize them
    scatter3(P(:,1), P(:,2), P(:,3), 'red', 'filled'); 
    % 3D Point Cloud
    % 3D Interpolated Surface
%     mesh(P(:,1), P(:,2), P(:,3))
    save('../results/q2_7.mat', 'F', 'M1', 'M2');
end
    