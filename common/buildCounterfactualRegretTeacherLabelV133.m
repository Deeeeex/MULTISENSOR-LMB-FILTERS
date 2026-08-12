function label = buildCounterfactualRegretTeacherLabelV133( ...
        referenceOutcome, candidateOutcome, groupIds, ...
        protectedFormationId, referenceAdjacency, protocol)
% BUILDCOUNTERFACTUALREGRETTEACHERLABELV133 Paired multistep harm label.
%
% Positive regret means the one-page abstention action is worse than the
% full-posterior continuation.  A peer formation is evaluated only from the
% first page at which information from the protected receiver can reach it
% on the directed formation graph.

required = { ...
    'eospaBySensorTime', 'posteriorConsensusByTime', ...
    'attemptedBytesByTime'};
if ~isstruct(referenceOutcome) || ~isscalar(referenceOutcome) || ...
        ~isstruct(candidateOutcome) || ~isscalar(candidateOutcome) || ...
        ~all(isfield(referenceOutcome, required)) || ...
        ~all(isfield(candidateOutcome, required)) || ...
        ~isequal(size(referenceOutcome.eospaBySensorTime), ...
            size(candidateOutcome.eospaBySensorTime)) || ...
        ~isequal(size(referenceOutcome.posteriorConsensusByTime), ...
            size(candidateOutcome.posteriorConsensusByTime)) || ...
        ~isequal(size(referenceOutcome.attemptedBytesByTime), ...
            size(candidateOutcome.attemptedBytesByTime)) || ...
        any(~isfinite(referenceOutcome.eospaBySensorTime(:))) || ...
        any(~isfinite(candidateOutcome.eospaBySensorTime(:))) || ...
        any(~isfinite(referenceOutcome.posteriorConsensusByTime(:))) || ...
        any(~isfinite(candidateOutcome.posteriorConsensusByTime(:))) || ...
        ~isstruct(protocol) || ~isscalar(protocol) || ...
        ~strcmp(protocol.id, 'counterfactual-action-regret-gate-v133-v1')
    error('CounterfactualRegretV133:InvalidTeacherOutcome', ...
        'The paired teacher outcomes are malformed or non-finite.');
end
eospaReference = double(referenceOutcome.eospaBySensorTime);
eospaCandidate = double(candidateOutcome.eospaBySensorTime);
[nodeCount, horizonSteps] = size(eospaReference);
groupIds = reshape(groupIds, 1, []);
referenceAdjacency = logical(referenceAdjacency);
formationIds = unique(groupIds, 'stable');
if numel(groupIds) ~= nodeCount || horizonSteps < 2 || ...
        ~ismember(protectedFormationId, formationIds) || ...
        ~isequal(size(referenceAdjacency), [nodeCount, nodeCount])
    error('CounterfactualRegretV133:InvalidTeacherOutcome', ...
        'The teacher route or formation membership is malformed.');
end

delta = eospaCandidate - eospaReference;
receiverRegret = mean(delta, 2);
formationRegret = zeros(1, numel(formationIds));
for formationIdx = 1:numel(formationIds)
    members = groupIds == formationIds(formationIdx);
    formationDelta = delta(members, :);
    formationRegret(formationIdx) = mean(formationDelta(:));
end
[formationAdjacency, protectedIndex] = collapseFormationRoute( ...
    referenceAdjacency, groupIds, formationIds, protectedFormationId);
distances = directedDistances(formationAdjacency, protectedIndex);
peerRegret = -inf(1, numel(formationIds));
for formationIdx = 1:numel(formationIds)
    if formationIdx == protectedIndex || ~isfinite(distances(formationIdx))
        continue;
    end
    firstCausalColumn = min(horizonSteps, distances(formationIdx) + 1);
    members = groupIds == formationIds(formationIdx);
    peerWindow = delta(members, firstCausalColumn:horizonSteps);
    peerRegret(formationIdx) = mean(peerWindow(:));
end
finitePeer = isfinite(peerRegret);
if any(finitePeer)
    downstreamPeerRegret = max(peerRegret(finitePeer));
else
    downstreamPeerRegret = 0;
end

postActionDelta = delta(:, 2:end);
components = [ ...
    mean(delta(:)), ...
    mean(postActionDelta(:)), ...
    max(receiverRegret), ...
    max(formationRegret), ...
    downstreamPeerRegret, ...
    candidateOutcome.posteriorConsensusByTime(end) - ...
        referenceOutcome.posteriorConsensusByTime(end)];
componentNames = protocol.teacherLabelContract.components;
if numel(componentNames) ~= numel(components)
    error('CounterfactualRegretV133:TeacherContractDrift', ...
        'The registered teacher component list changed.');
end
tolerance = protocol.teacherLabelContract.numericalTolerance;

label = struct();
label.contractVersion = ...
    'counterfactual-regret-gate-v133-teacher-label-v1';
label.componentNames = componentNames;
label.componentRegret = components;
label.maximumRegret = max(components);
label.harmful = any(components > tolerance);
label.safeAction = ~label.harmful;
label.numericalTolerance = tolerance;
label.receiverRegret = reshape(receiverRegret, 1, []);
label.formationIds = formationIds;
label.formationRegret = formationRegret;
label.downstreamDistance = distances;
label.downstreamPeerRegretByFormation = peerRegret;
label.referenceMeanEospa = mean(eospaReference(:));
label.candidateMeanEospa = mean(eospaCandidate(:));
label.actionGainPercent = 100 * ( ...
    label.referenceMeanEospa - label.candidateMeanEospa) / ...
    max(label.referenceMeanEospa, eps);
% The current action saves bytes only on its intervention page; all later
% pages use the same frozen full-posterior carrier.  Report that direct
% action-page saving rather than diluting it over the propagation horizon.
referenceBytes = referenceOutcome.attemptedBytesByTime(1);
candidateBytes = candidateOutcome.attemptedBytesByTime(1);
label.referenceAttemptedBytes = referenceBytes;
label.candidateAttemptedBytes = candidateBytes;
label.attemptedByteSavingPercent = 100 * ...
    (referenceBytes - candidateBytes) / max(referenceBytes, 1);
label.pairedStateRequired = true;
label.pairedMeasurementsRequired = true;
label.pairedDeliveryUniformsRequired = true;
label.pairedFilterRngRequired = true;
label.truthUsedOffline = true;
label.futureOutcomeUsedOffline = true;
label.teacherLabelAllowedAsOnlineFeature = false;
end

function [formation, protectedIndex] = collapseFormationRoute( ...
        adjacency, groupIds, formationIds, protectedFormationId)
formationCount = numel(formationIds);
formation = false(formationCount);
for receiverIdx = 1:formationCount
    receiverMembers = groupIds == formationIds(receiverIdx);
    for senderIdx = 1:formationCount
        if receiverIdx == senderIdx
            continue;
        end
        senderMembers = groupIds == formationIds(senderIdx);
        formation(receiverIdx, senderIdx) = any(any( ...
            adjacency(receiverMembers, senderMembers)));
    end
end
protectedIndex = find(formationIds == protectedFormationId, 1);
end

function distances = directedDistances(adjacency, sourceIdx)
nodeCount = size(adjacency, 1);
distances = inf(1, nodeCount);
distances(sourceIdx) = 0;
queue = sourceIdx;
while ~isempty(queue)
    current = queue(1);
    queue(1) = [];
    receivers = reshape(find(adjacency(:, current)), 1, []);
    for receiverIdx = receivers
        if isfinite(distances(receiverIdx))
            continue;
        end
        distances(receiverIdx) = distances(current) + 1;
        queue(end + 1) = receiverIdx; %#ok<AGROW>
    end
end
end
