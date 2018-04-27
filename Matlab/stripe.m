function [ sp ] = stripe( pos_seg, neg_seg )
%STRIPE 
%   Form a stripe representation given pos line segment and neg line
%   segment

%   (x2,y2) ------------------------- (x4,y4)   neg
%   |                                   |
%   |                                   |
%   (x1,y1) ------------------------- (x3,y3)   pos

%   input:  [rho; theta; xmin; xmax; polar]
%   output: [centroid_x; centroid_y; vertical_width; rho1; rho2; theta1; theta2; xmin; xmax; L]
    
    r1 = pos_seg(1);
    t1 = pos_seg(2);
    x1 = pos_seg(3);
    y1 = (r1 - x1*cos(t1))/sin(t1);
    x3 = pos_seg(4);
    y3 = (r1 - x3*cos(t1))/sin(t1);
    
    r2 = neg_seg(1);
    t2 = neg_seg(2);
    x2 = neg_seg(3);
    y2 = (r2 - x2*cos(t2))/sin(t2);
    x4 = neg_seg(4);
    y4 = (r2 - x4*cos(t2))/sin(t2);
    
    centroid_x = (x1+x2+x3+x4)/4;
    centroid_y = (y1+y2+y3+y4)/4;
    vertical_width = abs(centroid_x*cos(t1)+centroid_y*sin(t1)-r1) + abs(centroid_x*cos(t2)+centroid_y*sin(t2)-r2);
    rho1 = r1;
    rho2 = r2;

    xmin = min(x1, x2);
    xmax = max(x3, x4);
    L = (((x4-x2)/abs(sin(t2)))*((x3-x1)/abs(sin(t1))))^(0.25);
    sp = [centroid_x; centroid_y; vertical_width; rho1; rho2; t1; t2; xmin; xmax; L];

end

