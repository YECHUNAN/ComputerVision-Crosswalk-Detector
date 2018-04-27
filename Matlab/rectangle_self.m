function [ rect_center, rect_angle, W, L ] = rectangle_self( region, G, im )
%RECTANGLE_SELF
%   input: 2xN each column as a pixel in the region
%   output: rect_center 2x1 coordinate
%           rect_angle  2x1 eigenvector of the structure tensor
%           W: Width of the rectangle
%           L: Length of the rectangle
%   | y out of M
%   |
%   |
%   |----------- x out of N
%   for pixel [r,c],
%   x = c;
%   y = M-c+1;
    [M, N] = size(G);
    cx = 0;
    cy = 0;
    norm_term = 0;
    for i = 1:length(region(1,:))
        r = region(1,i);
        c = region(2,i);
        x = c;
        y = M-r+1;
        cx = cx + G(r,c)*x;
        cy = cy + G(r,c)*y;
        norm_term = norm_term + G(r,c);
    end
    cx = cx / norm_term;
    cy = cy / norm_term;
    rect_center = [cx; cy];
    mxx = 0;
    mxy = 0;
    myy = 0;
    for i = 1:length(region(1,:))
        r = region(1,i);
        c = region(2,i);
        x = c;
        y = M-r+1;
        mxx = mxx + G(r,c) * (x - cx)^2;
        mxy = mxy + G(r,c) * (x - cx)*(y - cy);
        myy = myy + G(r,c) * (y - cy)^2;
    end
    Matix = [mxx mxy; mxy myy];
    Matix = Matix ./ norm_term;
    [V, D] = eig(Matix);
    v = V(:,1);
    rect_angle = atan2(v(2), v(1));
    
    % normal vector
    n = [cos(rect_angle); sin(rect_angle)];
    n_perp = [0 1; -1 0] * n;
    pos_W_max = 0;
    neg_W_max = 0;
    pos_L_max = 0;
    neg_L_max = 0;
    for i = 1:length(region(1,:))
        r = region(1,i);
        c = region(2,i);
        x = c;
        y = M-r+1;
        dw = ([x; y] - rect_center)' * n;
        dl = ([x; y] - rect_center)' * n_perp;
        if dw > pos_W_max
            pos_W_max = dw;
        elseif dw < neg_W_max
            neg_W_max = dw;
        end
        if dl > pos_L_max
            pos_L_max = dl;
        elseif dl < neg_L_max
            neg_L_max = dl;
        end
    end

    W = 2*max(pos_W_max, -neg_W_max);
    L = 2*max(pos_L_max, -neg_L_max);

end