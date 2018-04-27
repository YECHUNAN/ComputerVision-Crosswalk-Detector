function [ score ] = proximity( seg1, seg2 )
%PROXIMITY 
%   proximity score about matching seg1 and seg2
%   higher score means higher likelihood of being a good match
%   [rho; theta; xmin; xmax; polar]
%   by default, seg1 is pos, seg2 is neg

%   (x2,y2) ------------------------- (x4,y4)
%   |                                   |
%   |                                   |
%   (x1,y1) ------------------------- (x3,y3)

    % check up down relationship
    score = 1;
    r1 = seg1(1);
    r2 = seg2(1);
    t1 = seg1(2);
    t2 = seg2(2);
    x1 = seg1(3);
    x2 = seg2(3);
    y1 = (r1 - x1*cos(t1)) / sin(t1);
    y2 = (r2 - x2*cos(t2)) / sin(t2);
    % check up-down relationship
    if y1 > y2
        score = 0;
        return;
    end
    % check vertical width
    v_width = abs([x1-x2; y1-y2]' * [cos(t1); sin(t1)]);
    if v_width < 5 || v_width > 40
        score = 0;
        return;
    end
    x3 = seg1(4);
    x4 = seg2(4);

    c = 2*pi;
    dover = max(0, min(x3, x4) - max(x1, x2));
    dnorm = min(x3 - x1, x4 - x2);
    score = score * dover/dnorm;
    score = score * exp(-abs(t1 - t2)/c);
    return;
end

