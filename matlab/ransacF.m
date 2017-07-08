function [ F ] = ransacF( pts1, pts2, M)
    % ransacF:
    %   pts1 - Nx2 matrix of (x,y) coordinates
    %   pts2 - Nx2 matrix of (x,y) coordinates
    %   M    - max (imwidth, imheight)

    k = 250; % number of retries for RANSAC
    nPts = size(pts1, 1);
    homoP1 = [pts1(:,1)'; pts1(:,2)'; ones(1, nPts)];
    homoP2 = [pts2(:,1)'; pts2(:,2)'; ones(1, nPts)];

    nIter = 0;
    curInliers = 0;
    maxInliers = -Inf;
    th = 0.0001;
    while nIter < k
        % Take a random sample of 7 points from pts1, pts2
        r = randi([1 nPts],7,1);
        p1 = pts1(r, :);
        p2 = pts2(r, :);

        % Find F using sevenpoint algo
        tempF = sevenpoint(p1, p2, M);

        % Find the F with maximum number of inliers (those with p1'tempFp2 < 0.01)
        curInliers = -Inf;
        for i = 1:3
            nInliers = 0;
            for j = 1:nPts
                predicted = abs( homoP1(:,j)'*tempF(:,:,i)*homoP2(:,j) );
                nInliers = nInliers + (predicted < th);
            end
            if nInliers > curInliers
                selectedF = tempF(:,:,i);
                curInliers = nInliers;
            end
        end

        % If the selected F has more inliers than maximum inliers, update
        if curInliers > maxInliers
            maxF = selectedF;
            maxInliers = curInliers;
        end

        nIter = nIter + 1;
    end
    F = maxF
end