function [adjacency, details] = ...
    selectDiverseLabelSetMessagePassingRoutingPolicy( ...
        context, model, proposalIndex, options)
% SELECTDIVERSELABELSETMESSAGEPASSINGROUTINGPOLICY Select safe bank rank.

if nargin < 4 || isempty(options)
    options = struct();
end
proposalIndex = round(proposalIndex);
if ~isscalar(proposalIndex) || ~isfinite(proposalIndex) || ...
        proposalIndex < 1
    error('Diverse label-set proposalIndex must be a positive integer.');
end
options.proposalCount = max(proposalIndex, ...
    getField(options, 'proposalCount', proposalIndex));
[candidateAdjacency, candidateDetails, metadata] = ...
    buildDiverseLabelSetMessagePassingProposalBank( ...
        context, model, options);
adjacency = candidateAdjacency(:, :, proposalIndex);
details = candidateDetails{proposalIndex};
details.diverseProposalBankMetadata = metadata;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
