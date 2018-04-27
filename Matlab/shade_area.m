function [ rst ] = shade_area( pos_seg, neg_seg, im )
%SHADE_AREA 
%           shade the area in the quadrilateral defined by line segment
%           pos_neg and neg_seg
%           
%           pos_seg/neg_seg: [rho; theta; xmin; xmax; polarity]

%     (x2,y2) ------------------------- (x4,y4)   neg_seg
%    /  | ->                               \
%   /   | ->                                \
%   (x1,y1) ------------------------------- (x3,y3)   pos_seg

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

    x = [x1,x2,x4,x3];
    y = [y1,y2,y4,y3];
    fill(x,y,'b');
    
    rst = 0;
end

