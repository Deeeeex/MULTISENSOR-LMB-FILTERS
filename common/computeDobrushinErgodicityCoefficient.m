function [coefficient, details] = ...
    computeDobrushinErgodicityCoefficient(matrix)
% COMPUTEDOBRUSHINERGODICITYCOEFFICIENT Row-stochastic contraction.
%
% For a row-stochastic matrix P,
%
%   delta(P) = 1 - min_{i,j} sum_k min(P(i,k), P(j,k))
%            = 0.5 max_{i,j} ||P(i,:) - P(j,:)||_1.
%
% It satisfies diam(Px) <= delta(P) diam(x) for every real vector x.

if ~isnumeric(matrix) || ~isreal(matrix) || ...
        ndims(matrix) ~= 2 || size(matrix, 1) ~= size(matrix, 2) || ...
        size(matrix, 1) < 1 || any(~isfinite(matrix(:))) || ...
        any(matrix(:) < -1e-12) || ...
        any(abs(sum(matrix, 2) - 1) > 1e-12)
    error('KlaWindowContraction:InvalidStochasticMatrix', ...
        'The Dobrushin coefficient requires a row-stochastic matrix.');
end
matrix = max(matrix, 0);
nodeCount = size(matrix, 1);
minimumOverlap = 1;
maximumHalfL1 = 0;
minimumOverlapPair = [1, 1];
maximumHalfL1Pair = [1, 1];
for leftIdx = 1:nodeCount
    for rightIdx = leftIdx:nodeCount
        overlap = sum(min(matrix(leftIdx, :), ...
            matrix(rightIdx, :)));
        halfL1 = 0.5 * sum(abs( ...
            matrix(leftIdx, :) - matrix(rightIdx, :)));
        if overlap < minimumOverlap
            minimumOverlap = overlap;
            minimumOverlapPair = [leftIdx, rightIdx];
        end
        if halfL1 > maximumHalfL1
            maximumHalfL1 = halfL1;
            maximumHalfL1Pair = [leftIdx, rightIdx];
        end
    end
end
coefficient = min(max(1 - minimumOverlap, 0), 1);
if abs(coefficient - maximumHalfL1) > 1e-10
    error('KlaWindowContraction:InternalCoefficientMismatch', ...
        'The two Dobrushin coefficient formulas disagree.');
end
details = struct( ...
    'contractVersion', 'dobrushin-ergodicity-coefficient-v1', ...
    'coefficient', coefficient, ...
    'minimumRowOverlap', minimumOverlap, ...
    'minimumRowOverlapPair', minimumOverlapPair, ...
    'maximumHalfL1Distance', maximumHalfL1, ...
    'maximumHalfL1Pair', maximumHalfL1Pair, ...
    'diameterContractionCertified', true);
end
