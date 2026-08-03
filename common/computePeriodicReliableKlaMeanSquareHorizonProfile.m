function profile = ...
    computePeriodicReliableKlaMeanSquareHorizonProfile( ...
        adjacencyPeriod, fusionWeightPeriod, reliabilityPeriod, ...
        maximumHorizon, options)
% COMPUTEPERIODICRELIABLEKLAMEANSQUAREHORIZONPROFILE
% Exact all-prefix factors for every possible phase of a periodic route.

if nargin < 5 || isempty(options)
    options = struct();
end
allowedFields = {'missingNeighborWeightMode', ...
    'maximumIncomingCount', 'targetSquaredFactors'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowedFields))
    error('PeriodicKlaMeanSquareProfile:InvalidOptions', ...
        'The periodic horizon-profile options are malformed.');
end
targets = reshape(getField(options, ...
    'targetSquaredFactors', [0.99, 0.90, 0.75, 0.50]), 1, []);
if ndims(adjacencyPeriod) > 3 || ndims(fusionWeightPeriod) > 3 || ...
        ndims(reliabilityPeriod) > 3 || ...
        ~isscalar(maximumHorizon) || ~isfinite(maximumHorizon) || ...
        maximumHorizon < 1 || maximumHorizon ~= round(maximumHorizon) || ...
        ~isnumeric(targets) || ~isreal(targets) || isempty(targets) || ...
        any(~isfinite(targets)) || any(targets <= 0) || ...
        any(targets >= 1) || any(diff(targets) >= 0)
    error('PeriodicKlaMeanSquareProfile:InvalidOptions', ...
        'The periodic horizon-profile inputs are invalid.');
end
if ndims(adjacencyPeriod) == 2
    adjacencyPeriod = reshape(adjacencyPeriod, ...
        size(adjacencyPeriod, 1), size(adjacencyPeriod, 2), 1);
end
if ndims(fusionWeightPeriod) == 2
    fusionWeightPeriod = reshape(fusionWeightPeriod, ...
        size(fusionWeightPeriod, 1), size(fusionWeightPeriod, 2), 1);
end
period = size(adjacencyPeriod, 3);
nodeCount = size(adjacencyPeriod, 1);
if ndims(reliabilityPeriod) == 2
    reliabilityPeriod = repmat(reliabilityPeriod, 1, 1, period);
end
if nodeCount < 2 || size(adjacencyPeriod, 2) ~= nodeCount || ...
        ~isequal(size(fusionWeightPeriod), size(adjacencyPeriod)) || ...
        ~isequal(size(reliabilityPeriod), size(adjacencyPeriod))
    error('PeriodicKlaMeanSquareProfile:InvalidSequence', ...
        'The periodic route matrices have inconsistent dimensions.');
end

factorByStartPhaseAndHorizon = nan(period, maximumHorizon + 1);
factorByStartPhaseAndHorizon(:, 1) = 1;
certificates = cell(1, period);
certificateOptions = rmfieldIfPresent(options, 'targetSquaredFactors');
for constructionOffset = 0:(period - 1)
    phaseIndices = mod(constructionOffset + (0:maximumHorizon-1), ...
        period) + 1;
    certificate = ...
        computeReliableKlaWindowMeanSquareContractionCertificate( ...
            adjacencyPeriod(:, :, phaseIndices), ...
            fusionWeightPeriod(:, :, phaseIndices), ...
            reliabilityPeriod(:, :, phaseIndices), ...
            certificateOptions);
    certificates{constructionOffset + 1} = certificate;
    suffix = certificate. ...
        suffixWorstCaseExpectedSquaredContractionFactors;
    for horizon = 1:maximumHorizon
        suffixIndex = maximumHorizon - horizon + 1;
        startPhase = phaseIndices(suffixIndex);
        factorByStartPhaseAndHorizon(startPhase, horizon + 1) = ...
            suffix(suffixIndex);
    end
end
if any(isnan(factorByStartPhaseAndHorizon(:)))
    error('PeriodicKlaMeanSquareProfile:InternalMappingError', ...
        'The phase/horizon factor table is incomplete.');
end

messageCountsByPhase = reshape(sum(sum(adjacencyPeriod, 1), 2), 1, []);
cumulativeMessagesByStartPhaseAndHorizon = ...
    zeros(period, maximumHorizon + 1);
for startPhase = 1:period
    for horizon = 1:maximumHorizon
        phases = mod(startPhase - 1 + (0:horizon-1), period) + 1;
        cumulativeMessagesByStartPhaseAndHorizon( ...
            startPhase, horizon + 1) = sum(messageCountsByPhase(phases));
    end
end

worstFactor = max(factorByStartPhaseAndHorizon, [], 1);
bestFactor = min(factorByStartPhaseAndHorizon, [], 1);
firstTargetByStartPhase = nan(period, numel(targets));
firstTargetWorstPhase = nan(1, numel(targets));
for targetIdx = 1:numel(targets)
    for startPhase = 1:period
        found = find(factorByStartPhaseAndHorizon( ...
            startPhase, 2:end) <= targets(targetIdx), 1, 'first');
        if ~isempty(found)
            firstTargetByStartPhase(startPhase, targetIdx) = found;
        end
    end
    found = find(worstFactor(2:end) <= targets(targetIdx), 1, 'first');
    if ~isempty(found)
        firstTargetWorstPhase(targetIdx) = found;
    end
end

profile = struct();
profile.contractVersion = ...
    'periodic-reliable-kla-mean-square-horizon-profile-v1';
profile.nodeCount = nodeCount;
profile.period = period;
profile.maximumHorizon = maximumHorizon;
profile.horizons = 0:maximumHorizon;
profile.factorByStartPhaseAndHorizon = ...
    factorByStartPhaseAndHorizon;
profile.worstStartPhaseFactorByHorizon = worstFactor;
profile.bestStartPhaseFactorByHorizon = bestFactor;
profile.firstTargetHorizonByStartPhase = firstTargetByStartPhase;
profile.firstTargetHorizonWorstPhase = firstTargetWorstPhase;
profile.targetSquaredFactors = targets;
profile.messageCountsByPhase = messageCountsByPhase;
profile.cumulativeMessagesByStartPhaseAndHorizon = ...
    cumulativeMessagesByStartPhaseAndHorizon;
profile.maximumCertificatesByConstructionOffset = certificates;
profile.allStartPhasesEnumeratedExactly = true;
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
