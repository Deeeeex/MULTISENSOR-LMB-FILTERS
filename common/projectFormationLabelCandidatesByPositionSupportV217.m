function [projectedCandidates, details] = ...
        projectFormationLabelCandidatesByPositionSupportV217( ...
            candidates, options)
% PROJECTFORMATIONLABELCANDIDATESBYPOSITIONSUPPORTV217 Reject disjoint labels.
%
% The V188/V190 synopsis models each two-dimensional position density as an
% isotropic Gaussian with the advertised covariance trace.  The resulting
% compatibility is exp(-d^2/2).  V217 keeps a candidate only when every
% receiver lies inside a frozen chi-square support contour.  This semantic
% guard uses no truth or future outcome and is not a tracking-value claim.

if nargin < 2 || isempty(options)
    options = struct();
end
chiSquareThreshold = getField( ...
    options, 'chiSquareThreshold', 9.21034037197618);
if ~isscalar(chiSquareThreshold) || ~isnumeric(chiSquareThreshold) || ...
        ~isfinite(chiSquareThreshold) || chiSquareThreshold <= 0
    error('FormationPositionSupportV217:InvalidThreshold', ...
        'The position-support chi-square threshold must be positive.');
end
if ~isstruct(candidates) || ...
        (~isempty(candidates) && ...
         ~all(isfield(candidates, { ...
            'candidateIndex', 'receiverCompatibilityMinimum'})))
    error('FormationPositionSupportV217:InvalidCandidates', ...
        'V190 candidates with minimum receiver compatibility are required.');
end
candidates = reshape(candidates, 1, []);
compatibilityFloor = exp(-0.5 * chiSquareThreshold);
if isempty(candidates)
    minimumCompatibility = zeros(1, 0);
else
    minimumCompatibility = reshape( ...
        [candidates.receiverCompatibilityMinimum], 1, []);
end
if any(~isfinite(minimumCompatibility)) || ...
        any(minimumCompatibility < 0) || any(minimumCompatibility > 1)
    error('FormationPositionSupportV217:InvalidCompatibility', ...
        'Candidate compatibility values must lie in [0,1].');
end
eligibleMask = minimumCompatibility >= compatibilityFloor - 1e-15;
projectedCandidates = candidates(eligibleMask);

details = struct();
details.contractVersion = ...
    'formation-label-position-support-projector-v217-v1';
details.chiSquareDimension = 2;
details.chiSquareCoverage = 0.99;
details.chiSquareThreshold = chiSquareThreshold;
details.minimumCompatibilityFloor = compatibilityFloor;
details.inputCandidateCount = numel(candidates);
details.eligibleMask = eligibleMask;
details.rejectedMask = ~eligibleMask;
details.eligibleCandidateCount = nnz(eligibleMask);
details.rejectedCandidateCount = nnz(~eligibleMask);
details.minimumReceiverCompatibility = minimumCompatibility;
details.projectedCandidateIndices = reshape( ...
    [projectedCandidates.candidateIndex], 1, []);
details.thresholdFittedToTrackingOutcomes = false;
details.trackingValueCertified = false;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
