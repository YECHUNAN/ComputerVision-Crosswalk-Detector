function [ p ] = polarity( region, gy )
%POLARITY 
%   compute the polarity of a line segment region
%   input:  region      pixel coordinates in rc space
%           gy          vertical gradient map
    
    sum = 0;
    for i = 1:5
        k = randi(length(region(1,:)));
        r = region(1,k);
        c = region(2,k);
        sum = sum + gy(r,c);
    end
    if sum > 0
        p = 1;
    else
        p = -1;
    end
end

