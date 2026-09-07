function [distance, components] = gospa(X, Y, c, p, alpha)
% GOSPA Generalized optimal sub-pattern assignment distance.
%   DISTANCE = GOSPA(X, Y, C, P, ALPHA) compares two finite sets whose
%   points are stored as columns of X and Y. Pairwise Euclidean distances
%   are cut off at C, raised to P, and optimally assigned. Unmatched points
%   contribute C^P / ALPHA each. The standard interpretable setting used by
%   the paper is C=5, P=2, ALPHA=2.
%
%   [DISTANCE, COMPONENTS] also returns the unrooted localization, missed,
%   false, and total costs. Keeping these costs unrooted makes the additive
%   GOSPA decomposition explicit.

if nargin < 5
    alpha = 2;
end
if nargin < 4
    p = 2;
end
if nargin < 3
    c = 5;
end

validateattributes(X, {'numeric'}, {'2d', 'real', 'finite'});
validateattributes(Y, {'numeric'}, {'2d', 'real', 'finite'});
validateattributes(c, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
validateattributes(p, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
validateattributes(alpha, {'numeric'}, {'scalar', 'real', 'finite', 'positive', '<=', 2});

if size(X, 1) ~= size(Y, 1) && ~isempty(X) && ~isempty(Y)
    error('gospa:DimensionMismatch', 'X and Y must have the same point dimension.');
end

n = size(X, 2);
m = size(Y, 2);
localizationCost = 0;

if n > 0 && m > 0
    pairCosts = zeros(n, m);
    for i = 1:n
        for j = 1:m
            pairCosts(i, j) = min(c, norm(X(:, i) - Y(:, j))) ^ p;
        end
    end
    [~, localizationCost] = Hungarian(pairCosts);
end

missedCost = (c ^ p / alpha) * max(n - m, 0);
falseCost = (c ^ p / alpha) * max(m - n, 0);
totalCost = localizationCost + missedCost + falseCost;
distance = totalCost ^ (1 / p);

components = struct( ...
    'localizationCost', localizationCost, ...
    'missedCost', missedCost, ...
    'falseCost', falseCost, ...
    'totalCost', totalCost, ...
    'cardinalityDifference', abs(n - m), ...
    'cutoff', c, ...
    'order', p, ...
    'alpha', alpha);
end

%!test
%! addpath(fileparts(mfilename('fullpath')));
%! [d, parts] = gospa(zeros(2, 0), zeros(2, 0), 5, 2, 2);
%! assert(d, 0, 1e-12);
%! assert(parts.totalCost, 0, 1e-12);

%!test
%! X = [0, 1; 0, 1];
%! [d, parts] = gospa(X, zeros(2, 0), 5, 2, 2);
%! assert(d, 5, 1e-12);
%! assert(parts.missedCost, 25, 1e-12);
%! assert(parts.falseCost, 0, 1e-12);

%!test
%! X = [0; 0];
%! Y = [3; 0];
%! assert(gospa(X, Y, 5, 2, 2), 3, 1e-12);
%! assert(gospa(Y, X, 5, 2, 2), 3, 1e-12);

%!test
%! X = [0, 10; 0, 0];
%! Y = [0; 0];
%! expected = sqrt(25 / 2);
%! assert(gospa(X, Y, 5, 2, 2), expected, 1e-12);
%! assert(gospa(Y, X, 5, 2, 2), expected, 1e-12);

%!test
%! X = [0; 0];
%! Y = [100; 0];
%! assert(gospa(X, Y, 5, 2, 2), 5, 1e-12);
