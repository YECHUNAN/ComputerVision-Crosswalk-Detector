function [ rst ] = line_similarity( seg1, seg2, eps_theta, eps_dist)
%LINE_SIMILARITY 
%   determine whether seg1 and seg2 belongs to the same line, are of same polarity
%   seg1, seg2:     [rho; theta; xmin; xmax; polarity]
%   rst:            1: YES      0: NO    
    rst = 1;
    if seg1(5) ~= seg2(5)
        rst = 0;
        return;
    end

    diff = angle_diff(seg1(2), seg2(2));
    if diff > eps_theta
        rst = 0;
        return;
    end
    rho1 = seg1(1);
    theta1 = seg1(2);

    rho2 = seg2(1);
    theta2 = seg2(2);
    
    x = seg2(3);
    y = (rho2 - x*cos(theta2))/ sin(theta2);
    dist2 = abs(x*cos(theta1)+y*sin(theta1)-rho1);    
    x = seg2(4);
    y = (rho2 - x*cos(theta2))/ sin(theta2);
    dist2 = (abs(x*cos(theta1)+y*sin(theta1)-rho1)+dist2)/2;
    
    dist = (dist2+dist2)/2;
    if dist > eps_dist
        rst = 0;
        return;
    end

end

