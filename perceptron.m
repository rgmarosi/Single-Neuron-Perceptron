function [ a ] = perceptron( W,p,b )
%perceptron architecture
%   a (output): binary classification of input vector [Sx1] (0 or 1)
%   W (input): weight matrix [SxR]
%   p (input): inputs of neuron [Rx1]
%   b (input): bias [Sx1]

a = hardlim(W*p+b);

end

