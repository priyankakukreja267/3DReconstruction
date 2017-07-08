% Finds the intensity value vector in a 17X17 vector around the point p
function v = getWindowVector(im, p, w, h)
% P: point around which the window vector is supposed to be found
% w: width of image
% h = height of image
% Assume size of window = 17X17
    x = p(1);
    y = p(2);
    w = im(max(1,x-8):min(x+8,w), max(1,y-8):min(y+8,h));
    nEls = size(w, 1) * size(w, 2);
    v = reshape(w, [nEls, 1]);
end