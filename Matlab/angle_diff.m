function [ diff ] = angle_diff( t1, t2 )
%ANGLE_DIFF 
%   input: two angles t1, t2 in [-pi, pi]
%   output: smallest difference to rotate one to match the other
    diff = abs(t1 - t2);
    if diff > pi
        diff = 2*pi-diff;
    end
    return;
end

