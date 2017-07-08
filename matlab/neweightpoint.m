function [ F ] = neweightpoint( X, Y, M )
    scale = [1/M 0 0; 0 1/M 0; 0 0 1];
    xscale = X ./ M;
    yscale = Y ./ M;
    
    % Nx1 dimensional coordinate points.
    x = xscale(:,1);
    y = xscale(:,2);
    xp = yscale(:,1);
    yp = yscale(:,2);

    numPts = size(xp, 1);
    
    % Generate the Nx9 A matrix.
    A = [x.*xp, ...
         x.*yp, ...
         x, ...
         y.*xp, ...
         y.*yp,  ...
         y,  ...
         xp,  ...
         yp,  ...
         ones(numPts, 1)];
    
    % Compute the SVD of A
    [U, S, V] = svd(A);
    F = reshape(V(:,9), 3, 3)';
    [FU, FS, FV] = svd(F);
    FS(3,3) = 0;
    F = FU*FS*FV';
    
    % Rescale the Fundamental Matrix
    F = scale'*F*scale;
    
end