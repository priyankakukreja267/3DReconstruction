function M2 = findM2(M2s, p1, p2)
%     for i = 1:4
%         [P, ] = triangulate(M1, p1, M2, p2);
%         triangulate using M(:,:,i);
%         check for some metric
%         if optimised, update
%         end
%     end

    load('../data/intrinsics.mat');
    M1 = K1*[eye(3), zeros(3,1)];
    maxPosZ = -Inf;
    
    for i = 1:4
        [P, ] = triangulate(M1, p1, K2*M2s(:,:,i), p2);
        P = P';

        % find the number of points with a positive Z value. The one with the 
        % maximum value is chosen
        nPosZ = sum(P(3,:) > 0);
        
        if nPosZ > maxPosZ
            maxPosZ = nPosZ;
            selectedM = M2s(:,:,i);
        end
    end
    
    M2 = selectedM;
    [P, ] = triangulate(M1, p1, K2*M2, p2);
    save('../results/q2_5.mat', 'M2', 'p1', 'p2', 'P');
    M2 = M2s(:,:,3);
end
