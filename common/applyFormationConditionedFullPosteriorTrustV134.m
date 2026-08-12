function weights = applyFormationConditionedFullPosteriorTrustV134( ...
        weights, fusionDetails, missingNeighborWeightMode)
% APPLYFORMATIONCONDITIONEDFULLPOSTERIORTRUSTV134 Attenuate cross inputs.
%
% The input posterior itself is unchanged.  The message planner attaches a
% factor only to selected cross-residual inputs.  This function multiplies
% those KLA weights and leaves normalization to the caller.  Under the
% runtime's `self` missing-input semantics, removed weight is transferred
% to self so alpha=0 is continuous with payload abstention.

weights = reshape(weights, 1, []);
if ~isstruct(fusionDetails) || ...
        ~isfield(fusionDetails, 'formationTrustFactor') || ...
        numel(fusionDetails.formationTrustFactor) ~= numel(weights)
    error('V134FullPosteriorTrust:InvalidInput', ...
        'Every fusion input must carry one formation trust factor.');
end
formationTrustFactors = reshape( ...
    fusionDetails.formationTrustFactor, 1, []);
if any(~isfinite(formationTrustFactors)) || ...
        any(formationTrustFactors < 0) || ...
        any(formationTrustFactors > 1) || ...
        ~any(strcmpi(missingNeighborWeightMode, {'renormalize', 'self'}))
    error('V134FullPosteriorTrust:InvalidInput', ...
        'Trust factors must lie in [0,1] and use known missing semantics.');
end
for inputIdx = 2:numel(weights)
    trustFactor = formationTrustFactors(inputIdx);
    removedWeight = weights(inputIdx) * (1 - trustFactor);
    weights(inputIdx) = weights(inputIdx) * trustFactor;
    if strcmpi(missingNeighborWeightMode, 'self')
        weights(1) = weights(1) + removedWeight;
    end
end
end
