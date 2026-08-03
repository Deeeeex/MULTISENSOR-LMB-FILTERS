function profile = ...
    computeStaticReliableKlaMeanSquareHorizonProfile( ...
        adjacency, fusionWeights, receiverLinkReliability, ...
        maximumHorizon, options)
% COMPUTESTATICRELIABLEKLAMEANSQUAREHORIZONPROFILE
% Exact all-prefix profile for a repeated current mixing page.
%
% One maximum-window backward recursion contains the exact suffix factor
% for every shorter repeated-page horizon, avoiding separate recomputation.

if nargin < 5 || isempty(options)
    options = struct();
end
allowedFields = { ...
    'missingNeighborWeightMode', 'maximumIncomingCount', ...
    'targetSquaredFactors'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowedFields))
    error('KlaMeanSquareProfile:InvalidOptions', ...
        'The static horizon-profile options are malformed.');
end
targets = reshape(getField(options, ...
    'targetSquaredFactors', [0.99, 0.90, 0.75, 0.50]), 1, []);
if ndims(adjacency) ~= 2 || ndims(fusionWeights) ~= 2 || ...
        ndims(receiverLinkReliability) ~= 2 || ...
        ~isscalar(maximumHorizon) || ~isfinite(maximumHorizon) || ...
        maximumHorizon < 1 || ...
        maximumHorizon ~= round(maximumHorizon) || ...
        ~isnumeric(targets) || ~isreal(targets) || isempty(targets) || ...
        any(~isfinite(targets)) || any(targets <= 0) || ...
        any(targets >= 1) || ...
        any(diff(targets) >= 0)
    error('KlaMeanSquareProfile:InvalidOptions', ...
        'The static horizon-profile inputs are invalid.');
end
nodeCount = size(adjacency, 1);
adjacencySequence = repmat(adjacency, ...
    1, 1, maximumHorizon);
weightSequence = repmat(fusionWeights, 1, 1, maximumHorizon);
reliabilitySequence = repmat(receiverLinkReliability, ...
    1, 1, maximumHorizon);
certificateOptions = rmfieldIfPresent(options, ...
    'targetSquaredFactors');
certificate = ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacencySequence, weightSequence, reliabilitySequence, ...
        certificateOptions);

suffix = certificate. ...
    suffixWorstCaseExpectedSquaredContractionFactors;
factorByHorizon = [1, fliplr(suffix(1:end-1))];
horizons = 0:maximumHorizon;
firstStrict = find(factorByHorizon(2:end) < 1 - 1e-12, ...
    1, 'first');
if isempty(firstStrict)
    firstStrict = nan;
end
firstTargetHorizon = nan(size(targets));
for targetIdx = 1:numel(targets)
    found = find(factorByHorizon(2:end) <= targets(targetIdx), ...
        1, 'first');
    if ~isempty(found)
        firstTargetHorizon(targetIdx) = found;
    end
end

profile = struct();
profile.contractVersion = ...
    'static-reliable-kla-mean-square-horizon-profile-v1';
profile.nodeCount = nodeCount;
profile.maximumHorizon = maximumHorizon;
profile.horizons = horizons;
profile.worstCaseExpectedSquaredFactorByHorizon = ...
    factorByHorizon;
profile.worstCaseExpectedRmsFactorByHorizon = ...
    sqrt(factorByHorizon);
profile.firstStrictContractionHorizon = firstStrict;
profile.targetSquaredFactors = targets;
profile.firstTargetHorizon = firstTargetHorizon;
profile.maximumWindowCertificate = certificate;
profile.repeatedCurrentPage = true;
profile.posteriorUsed = false;
profile.truthUsed = false;
profile.futureOutcomeUsed = false;
end

function structure = rmfieldIfPresent(structure, name)
if isfield(structure, name)
    structure = rmfield(structure, name);
end
end

function value = getField(structure, name, defaultValue)
if isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
