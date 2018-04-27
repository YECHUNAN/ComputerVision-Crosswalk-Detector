function [ rst ] = check_color( pos_seg, neg_seg, im )
%CHECK_COLOR Summary of this function goes here
%       
%       pos_neg and neg_seg
%           
%       pos_seg/neg_seg: [rho; theta; xmin; xmax; polarity]

%     (x2,y2) ------------------------- (x4,y4)   neg_seg
%    /  | ->                               \
%   /   | ->                                \
%   (x1,y1) ------------------------------- (x3,y3)   pos_seg
    black_threshold = 65;
    rst = 0;
    r1 = pos_seg(1);
    r2 = neg_seg(1);
    t1 = pos_seg(2);
    t2 = neg_seg(2);
    x1 = pos_seg(3);
    x2 = neg_seg(3);
    y1 = (r1 - x1*cos(t1)) / sin(t1);
    y2 = (r2 - x2*cos(t2)) / sin(t2);
    
    x3 = pos_seg(4);
    x4 = neg_seg(4);
    y3 = (r1 - x3*cos(t1)) / sin(t1);
    y4 = (r2 - x4*cos(t2)) / sin(t2);

    [M, N] = size(im);
    r = [round(M-y1+1), round(M-y2+1), round(M-y4+1), round(M-y3+1)];
    c = [round(x1)+1, round(x2)+1, round(x4)+1, round(x3)+1];
    mask = roipoly(im, c, r);
    total_pix_num = 0;
    black_pix_num = 0;
    for i = round(min([x1, x2])):round(max([x3, x4]))
        for j = round(min([y1, y3])):round(max([y2, y4])) 
            r = max(1, min(M - j + 1, M));            
            c = max(1, min(i, N));
            if mask(r,c) == 1
                total_pix_num = total_pix_num + 1;
                if im(r,c) < black_threshold
                    black_pix_num = black_pix_num + 1;
                end
            end
        end
    end
    if black_pix_num / total_pix_num > 0.6
        rst = 1;
    end
end
