function indices = lmbConditionalMapIndices(r, cardinality)
% LMBCONDITIONALMAPINDICES Conditional-MAP labels at fixed cardinality.
%
% For an LMB with existence probabilities r, the probability of a label
% subset I conditional on |I| = k is proportional to the product of the
% odds r_i/(1-r_i) over i in I.  The maximizing subset therefore contains
% the k largest existence probabilities.

r = reshape(r, 1, []);
if any(~isfinite(r)) || any(r < 0) || any(r > 1) || ...
        ~isscalar(cardinality) || ~isfinite(cardinality) || ...
        cardinality ~= round(cardinality) || cardinality < 0 || ...
        cardinality > numel(r)
    error('LmbConditionalMap:InvalidInput', ...
        ['Existence probabilities must lie in [0,1], and cardinality ', ...
         'must be an integer between zero and their count.']);
end
[~, order] = sort(-r);
indices = order(1:cardinality);
end
