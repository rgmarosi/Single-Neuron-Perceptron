function [ a ] = hardlim( n )
%hardlim Hard Limit transfer function
%   Accepts column vector. Returns column vector.
%   behaves like unit step.
%   a=0 for n<0
%   a=1 for n>=0
    dims = size(n,1);
    a = zeros(dims,1);
    for i = 1:dims
        if n(i,1) < 0
            a(i,1) = 0;
        else
            a(i,1) = 1;
        end
    end
end

