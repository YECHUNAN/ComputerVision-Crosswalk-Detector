function [ rst ] = connectivity( stripe_up, stripe_down )
%CONNECTIVITY 
%   compute connectivity between stripe_up and stripe_down
%   using metric:   1. distance between stripe centroids, 
%                   2. cross ratio error measure R
%                   3. monotonicity requirement, higher stripe have less
%                   vertical width than lower strip
    % Hyper parameter
    y_thresh = 5;
    d_thresh = 50;
    R_thresh = 0.15;
    
    rst = 0;
    if abs((stripe_up(2)-stripe_down(2))) <= y_thresh
        rst = 0.3*max(stripe_up(10),stripe_down(10))/min(stripe_up(10),stripe_down(10));
        return;
    end
    if (stripe_up(3)-stripe_down(3))*(stripe_up(2)-stripe_up(2)) > 0
        return;
    end
    d = abs(stripe_up(2) - stripe_down(2));
    if d > d_thresh
        return;
    end
    % how to compute cross ratio?
    v1 = stripe_up(1);
    A = (stripe_up(5) - v1*cos(stripe_up(7) ))/sin(stripe_up(7));
    B = (stripe_up(4) - v1*cos(stripe_up(6) ))/sin(stripe_up(6));
    C = (stripe_down(5) - v1*cos(stripe_down(7) ))/sin(stripe_down(7));
    D = (stripe_down(4) - v1*cos(stripe_down(6) ))/sin(stripe_down(6));
    r1 = abs((A-B)*(C-D)/((C-B)*(A-D)));
    v2 = stripe_down(1);
    A = (stripe_up(5) - v2*cos(stripe_up(7) ))/sin(stripe_up(7));
    B = (stripe_up(4) - v2*cos(stripe_up(6) ))/sin(stripe_up(6));
    C = (stripe_down(5) - v2*cos(stripe_down(7) ))/sin(stripe_down(7));
    D = (stripe_down(4) - v2*cos(stripe_down(6) ))/sin(stripe_down(6));
    r2 = abs((A-B)*(C-D)/((C-B)*(A-D)));
    R = (abs(r1-0.25)+abs(r2-0.25))/2+2*abs(r1-r2);
    if R >= R_thresh
        return;
    end
    rst = R;
end

