function metrics = computeFormationH3ObservableEventScore( ...
        context, groupIds, options)
% COMPUTEFORMATIONH3OBSERVABLEEVENTSCORE Truth-free routing opportunity.
%
% The score is high when sensor posteriors are coherent within formations
% but disagree across formations, especially when the currently selected
% cross-formation route is under link stress.  It uses only predecision LMB
% posteriors, the current link-probability page, and the previous selected
% topology.  Sensor indices, formation identifiers, truth, future
% measurements, and future link outcomes do not enter the score.

if nargin < 3 || isempty(options)
    options = struct();
end
tailFraction = getField(options, 'tailFraction', 0.34);
tailWeight = getField(options, 'tailWeight', 0.5);
groupIds = reshape(groupIds, 1, []);
nodeCount = numel(groupIds);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
required = { ...
    'localPosteriorBySensor', 'model', 'commConfig', ...
    'previousAdjacency'};
if ~isstruct(context) || ~all(isfield(context, required)) || ...
        nodeCount < 2 || formationCount < 2 || ...
        numel(context.localPosteriorBySensor) ~= nodeCount || ...
        ~isequal(size(context.previousAdjacency), ...
            [nodeCount, nodeCount]) || ...
        any(~isfinite(groupIds)) || ...
        any(groupIds ~= round(groupIds)) || any(groupIds < 1) || ...
        ~isscalar(tailFraction) || ~isfinite(tailFraction) || ...
        tailFraction <= 0 || tailFraction > 1 || ...
        ~isscalar(tailWeight) || ~isfinite(tailWeight) || ...
        tailWeight < 0 || tailWeight > 1 || ...
        isfield(context.commConfig, 'linkUniforms')
    error('Formation H=3 observable event-score input is invalid.');
end

summaries = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    summaries{sensorIdx} = summarizeLmbPosteriorForDisagreement( ...
        context.localPosteriorBySensor{sensorIdx}, context.model);
end
pairCombined = zeros(nodeCount);
withinValues = zeros(1, 0);
for leftIdx = 1:(nodeCount - 1)
    for rightIdx = (leftIdx + 1):nodeCount
        pair = computeLmbPosteriorSummaryDisagreement( ...
            summaries{leftIdx}, summaries{rightIdx});
        pairCombined(leftIdx, rightIdx) = pair.combined;
        pairCombined(rightIdx, leftIdx) = pair.combined;
        if groupIds(leftIdx) == groupIds(rightIdx)
            withinValues(end + 1) = pair.combined; %#ok<AGROW>
        end
    end
end

formationPairMeans = zeros(1, ...
    formationCount * (formationCount - 1) / 2);
pairCursor = 0;
for leftFormation = 1:(formationCount - 1)
    leftMembers = find(groupIds == groups(leftFormation));
    for rightFormation = (leftFormation + 1):formationCount
        rightMembers = find(groupIds == groups(rightFormation));
        pairCursor = pairCursor + 1;
        crossBlock = pairCombined(leftMembers, rightMembers);
        formationPairMeans(pairCursor) = mean(crossBlock(:));
    end
end
if pairCursor ~= numel(formationPairMeans) || ...
        isempty(formationPairMeans)
    error('Formation H=3 event-score pair aggregation failed.');
end

withinMean = meanOrZero(withinValues);
crossMean = mean(formationPairMeans);
crossTail = upperTailMean(formationPairMeans, tailFraction);
robustCross = ...
    (1 - tailWeight) * crossMean + tailWeight * crossTail;
posteriorContrast = max(robustCross - withinMean, 0);

[linkStressMean, linkStressTail, selectedCrossEdgeCount] = ...
    currentSelectedCrossFormationLinkStress( ...
        context, groupIds, tailFraction);
robustLinkStress = ...
    (1 - tailWeight) * linkStressMean + ...
    tailWeight * linkStressTail;
score = posteriorContrast * (1 + robustLinkStress);
if ~isfinite(score) || score < 0
    error('Formation H=3 observable event score is invalid.');
end

metrics = struct();
metrics.contractVersion = ...
    'formation-h3-observable-event-score-v1';
metrics.score = score;
metrics.posteriorContrast = posteriorContrast;
metrics.withinFormationMeanDisagreement = withinMean;
metrics.crossFormationMeanDisagreement = crossMean;
metrics.crossFormationTailDisagreement = crossTail;
metrics.robustCrossFormationDisagreement = robustCross;
metrics.selectedCrossLinkDropMean = linkStressMean;
metrics.selectedCrossLinkDropTail = linkStressTail;
metrics.robustSelectedCrossLinkStress = robustLinkStress;
metrics.selectedCrossEdgeCount = selectedCrossEdgeCount;
metrics.sensorCount = nodeCount;
metrics.formationCount = formationCount;
metrics.formationPairCount = numel(formationPairMeans);
metrics.tailFraction = tailFraction;
metrics.tailWeight = tailWeight;
metrics.sensorPermutationInvariant = true;
metrics.formationLabelPermutationInvariant = true;
metrics.truthUsed = false;
metrics.futureMeasurementsUsed = false;
metrics.futureLinkOutcomesUsed = false;
end

function [meanStress, tailStress, edgeCount] = ...
        currentSelectedCrossFormationLinkStress( ...
            context, groupIds, tailFraction)
nodeCount = numel(groupIds);
if isfield(context.commConfig, 'pDropByEdge') && ...
        ~isempty(context.commConfig.pDropByEdge)
    pDrop = context.commConfig.pDropByEdge;
    if ndims(pDrop) >= 3 || ...
            ~isequal(size(pDrop), [nodeCount, nodeCount])
        error('Formation H=3 event score requires one current drop page.');
    end
    pDrop = min(max(double(pDrop), 0), 1);
else
    pDrop = zeros(nodeCount);
end
dropByReceiverSender = pDrop';
selected = logical(context.previousAdjacency);
selected(1:(nodeCount + 1):end) = false;
crossMask = groupIds(:) ~= groupIds(:)';
values = dropByReceiverSender(selected & crossMask);
edgeCount = numel(values);
if isempty(values)
    meanStress = 0;
    tailStress = 0;
else
    meanStress = mean(values);
    tailStress = upperTailMean(values, tailFraction);
end
end

function value = upperTailMean(values, fraction)
values = reshape(values, 1, []);
if isempty(values)
    value = 0;
    return;
end
count = max(1, ceil(fraction * numel(values)));
values = sort(values, 'descend');
value = mean(values(1:count));
end

function value = meanOrZero(values)
if isempty(values)
    value = 0;
else
    value = mean(values);
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
