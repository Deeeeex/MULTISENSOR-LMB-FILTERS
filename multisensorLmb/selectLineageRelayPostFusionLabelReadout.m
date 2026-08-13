function [selectedObjects, details] = ...
        selectLineageRelayPostFusionLabelReadout( ...
            workingObjects, relayObjects, localObjects, labelEvidence)
% SELECTLINEAGERELAYPOSTFUSIONLABELREADOUT Evidence-conditioned readout.
%
% The hidden working posterior is not changed.  For labels with current
% positive or credible-negative local evidence, this function selects the
% post-fusion W or exact-R Bernoulli that is closer to the receiver's current
% local update in reference-to-candidate Bernoulli-GM KLD.  Ties retain W.

selectedObjects = reshape(workingObjects, 1, []);
details = struct( ...
    'contractVersion', 'lineage-relay-post-fusion-label-readout-v1', ...
    'evaluatedLabelCount', 0, ...
    'relaySelectedLabelCount', 0, ...
    'evaluatedLabels', zeros(2, 0), ...
    'relaySelectedLabels', zeros(2, 0), ...
    'workingDivergence', zeros(1, 0), ...
    'relayDivergence', zeros(1, 0), ...
    'stateMutationAllowed', false, ...
    'usesTargetTruth', false, ...
    'usesFutureMeasurements', false);
if isempty(labelEvidence)
    return;
end
if ~isstruct(labelEvidence)
    error('PostFusionLabelReadoutV138:InvalidEvidence', ...
        'Per-label local evidence must be a structure array.');
end

for evidenceIdx = 1:numel(labelEvidence)
    evidence = labelEvidence(evidenceIdx);
    if ~isfield(evidence, 'isAdmissibleToSafeReference') || ...
            ~logical(evidence.isAdmissibleToSafeReference)
        continue;
    end
    label = [evidence.birthTime; evidence.birthLocation];
    localObject = findObject(localObjects, label);
    workingObject = findObject(workingObjects, label);
    relayObject = findObject(relayObjects, label);
    workingDivergence = localToCandidateDivergence( ...
        localObject, workingObject);
    relayDivergence = localToCandidateDivergence( ...
        localObject, relayObject);
    details.evaluatedLabelCount = details.evaluatedLabelCount + 1;
    details.evaluatedLabels(:, end + 1) = label; %#ok<AGROW>
    details.workingDivergence(end + 1) = workingDivergence; %#ok<AGROW>
    details.relayDivergence(end + 1) = relayDivergence; %#ok<AGROW>
    if ~(relayDivergence + 1e-12 < workingDivergence)
        continue;
    end
    selectedObjects = replaceLabel( ...
        selectedObjects, relayObject, label);
    details.relaySelectedLabelCount = ...
        details.relaySelectedLabelCount + 1;
    details.relaySelectedLabels(:, end + 1) = label; %#ok<AGROW>
end
end

function divergence = localToCandidateDivergence(localObject, candidate)
if isempty(localObject)
    localExistence = 0;
    spatialKld = 0;
elseif isempty(candidate)
    localExistence = localObject.r;
    spatialKld = 0;
else
    localExistence = localObject.r;
    spatialKld = approximateLmbSpatialKldCubature( ...
        localObject, candidate);
end
if isempty(candidate)
    candidateExistence = 0;
else
    candidateExistence = candidate.r;
end
terms = computeLmbBernoulliKldTerms( ...
    localExistence, candidateExistence, spatialKld);
divergence = terms.total;
end

function selected = replaceLabel(selected, replacement, label)
selectedIdx = findObjectIndex(selected, label);
if isempty(replacement)
    if selectedIdx > 0
        selected(selectedIdx) = [];
    end
elseif selectedIdx > 0
    selected(selectedIdx) = replacement;
else
    selected(end + 1) = replacement;
end
end

function object = findObject(objects, label)
idx = findObjectIndex(objects, label);
if idx > 0
    object = objects(idx);
else
    object = [];
end
end

function idx = findObjectIndex(objects, label)
idx = 0;
for objectIdx = 1:numel(objects)
    if objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        idx = objectIdx;
        return;
    end
end
end
