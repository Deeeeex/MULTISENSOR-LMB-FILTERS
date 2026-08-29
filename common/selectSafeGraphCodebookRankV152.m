function [adjacency, details] = ...
    selectSafeGraphCodebookRankV152(context, model, rankIndex, options)
% SELECTSAFEGRAPHCODEBOOKRANKV152 Select one complete feasible graph rank.

if nargin < 4 || isempty(options)
    options = struct();
end
protocol = getField(options, 'protocol', ...
    getSafeGraphCodebookOracleV152Protocol());
labelProtocol = getField(options, 'labelProtocol', ...
    getLabelSetSimulatorPolicyProtocol());
rankIndex = round(rankIndex);
if ~isscalar(rankIndex) || ~isfinite(rankIndex) || ...
        ~ismember(rankIndex, protocol.proposalRanks)
    error('V152 graph rank is outside the frozen codebook.');
end
if ~strcmp(labelProtocol.id, protocol.labelSetProtocolId) || ...
        abs(labelProtocol.dominantWeight - ...
            protocol.dominantWeight) > 1e-12 || ...
        abs(labelProtocol.localResidualWeight - ...
            protocol.residualWeight) > 1e-12
    error('V152 label-set generator differs from the frozen protocol.');
end

[candidateAdjacency, candidateDetails, metadata] = ...
    buildDiverseLabelSetMessagePassingProposalBank( ...
        context, model, struct( ...
            'protocol', labelProtocol, ...
            'proposalCount', rankIndex, ...
            'fixedResidualWeight', protocol.residualWeight));
if size(candidateAdjacency, 3) ~= rankIndex || ...
        numel(candidateDetails) ~= rankIndex || ...
        ~metadata.allCandidatesDistinct || ...
        ~metadata.allCandidatesTruthFree || ...
        ~metadata.allCandidatesSafe
    error('V152 graph codebook failed its complete safety contract.');
end

adjacency = candidateAdjacency(:, :, rankIndex);
details = candidateDetails{rankIndex};
details.mode = 'safe-graph-codebook-rank-v152';
details.v152ProtocolId = protocol.id;
details.v152ContractVersion = protocol.contractVersion;
details.v152Rank = rankIndex;
details.v152ProposalCount = protocol.proposalCount;
details.v152GeneratedPrefixCount = rankIndex;
details.v152CodebookGraphSha256 = metadata.graphSha256;
details.v152MaximumPairwiseCrossEdgeJaccard = ...
    metadata.maximumPairwiseCrossEdgeJaccard;
details.v152AllCandidatesDistinct = metadata.allCandidatesDistinct;
details.v152AllCandidatesTruthFree = metadata.allCandidatesTruthFree;
details.v152AllCandidatesSafe = metadata.allCandidatesSafe;
details.v152ValuePredictionUsed = false;
details.truthUsed = false;
details.groundTruthUsed = false;
details.futureOutcomeUsed = false;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
