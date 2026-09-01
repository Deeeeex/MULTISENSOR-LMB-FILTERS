function [selectedFraction, selectedIndex] = ...
        selectMaximumAllowedTransferFractionV226(fractions, allowedMask)
% SELECTMAXIMUMALLOWEDTRANSFERFRACTIONV226 Deterministic grid projection.
%
% Every candidate must already have been evaluated.  Selection uses the
% numerical maximum among allowed fractions and therefore does not assume
% that the safety decision is monotone in the transfer fraction.

fractions = reshape(fractions, 1, []);
allowedMask = reshape(allowedMask, 1, []);
if isempty(fractions) || numel(fractions) ~= numel(allowedMask) || ...
        any(~isfinite(fractions)) || any(fractions <= 0) || ...
        any(fractions > 1) || ...
        numel(unique(fractions)) ~= numel(fractions) || ...
        any(~ismember(allowedMask, [false, true]))
    error('BoundedDominantEdgeTransferV226:InvalidSelection', ...
        'The bounded-transfer selection request is malformed.');
end

allowedIndices = find(logical(allowedMask));
if isempty(allowedIndices)
    selectedFraction = 0;
    selectedIndex = 0;
    return;
end
[selectedFraction, localIndex] = max(fractions(allowedIndices));
selectedIndex = allowedIndices(localIndex);
end
