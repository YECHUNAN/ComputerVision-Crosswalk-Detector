function [ rst ] = isValidPixel( p, M, N )
%ISVALIDPIXEL 
%   Given MxN image, check whether p = [r; c] is a valid pixel index
    rst = 1;
    r = p(1);
    c = p(2);
    if r<1 || c<1 || r>M || c>N
        rst = 0;
    end
    return;
end

