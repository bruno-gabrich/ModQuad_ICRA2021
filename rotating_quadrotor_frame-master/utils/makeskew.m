function [s] = makeskew(v)
%MAKESKEW 
%   Make a skew symmetric matrix from a vector v
%   v must be a vector of length 3
if length(v) == 3
    s = [ 0    -v(3) v(2);
         v(3)   0       -v(1);
        -v(2)   v(1)    0];
else
    error('input arg must be of size 3');
end

