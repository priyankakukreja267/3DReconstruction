function [ x2, y2 ] = epipolarCorrespondence( im1, im2, F, x1, y1 )
    i1 = im1;
    i2 = im2;
    nPts = size(x1);
    h = size(i2, 1);
    w = size(i2, 2);

    for i = 1:nPts
        p = [x1(i); y1(i); 1];
        l = F*p;
        y = [1:h]';
        a = l(1); b = l(2); c = l(3);
        epipolarLine = [round((-c-(b.*y))/a), y];
        
        offset = 30;
        loY = int32(max(1, y1(i) - 2*offset));
        hiY = int32(min(y1(i) + offset, length(epipolarLine)));

        minPt = epipolarLine(loY, :);
        minDist = Inf;
        
        for j = loY:hiY
            w1 = getWindowVector(i1, [p(1); p(2)], w, h);
            w2 = getWindowVector(i2, epipolarLine(j, :), w, h);
            d = getDistance(w1, w2);
            if d < minDist
                minDist = d;
                minPt = epipolarLine(j, :);
            end
        end
        x2(i) = minPt(1);
        y2(i) = minPt(2);
    end
    pts1 = [x1';y1']';
    pts2 = [x2;y2]';
    save('../results/q2_6.mat', 'F', 'pts1', 'pts2');
end

% returns the SSD between 2 intensity vectors
function d = getDistance(w1, w2)
    w1Len = size(w1, 1);
    w2Len = size(w2, 1);
    if w1Len < w2Len
        v1 = [w1; zeros(w2Len-w1Len, 1)];
        v2 = w2;
    else
        v1 = w1;
        v2 = [w2; zeros(w1Len-w2Len, 1)];
    end
    d = sum((v1 - v2).^2);
end

% Finds the intensity value vector in a 17X17 vector around the point p
function v = getWindowVector(im, p, w, h)
% P: point around which the window vector is supposed to be found
% w: width of image
% h = height of image
% Assume size of window = 17X17
    x = p(1);
    y = p(2);
    wsize = 5;
    loX = max(1,x-wsize);
    hiX = min(x+wsize,w);
    loY = max(1,y-wsize);
    hiY = min(y+wsize,h);
    v = im(loY:hiY, loX:hiX);


    filter1 = fspecial('gaussian', 1 + (2*wsize), 1.0);
    v = imfilter(v, filter1, 'replicate', 'conv');


    nEls = size(v, 1) * size(v, 2);
    v = reshape(v, [nEls, 1]);
end
