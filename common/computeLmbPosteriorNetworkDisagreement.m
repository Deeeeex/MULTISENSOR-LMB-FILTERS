function metrics = computeLmbPosteriorNetworkDisagreement( ...
    posteriorsBySensor, model)
% COMPUTELMBPOSTERIORNETWORKDISAGREEMENT Label-wise posterior dispersion.
%
% The metric separates existence disagreement from normalized spatial
% moment disagreement. It operates on the Bernoulli posteriors rather than
% MAP set estimates, avoiding cardinality-extraction artifacts.

pairExistence = [];
pairSpatial = [];
pairCombined = [];
sensorCount = numel(posteriorsBySensor);
for leftIdx = 1:sensorCount-1
    for rightIdx = leftIdx+1:sensorCount
        pair = computeLmbPosteriorPairDisagreement( ...
            posteriorsBySensor{leftIdx}, ...
            posteriorsBySensor{rightIdx}, model);
        pairExistence(end+1) = pair.existence; %#ok<AGROW>
        pairSpatial(end+1) = pair.spatial; %#ok<AGROW>
        pairCombined(end+1) = pair.combined; %#ok<AGROW>
    end
end
metrics = struct( ...
    'existence', meanOrZero(pairExistence), ...
    'spatial', meanOrZero(pairSpatial), ...
    'combined', meanOrZero(pairCombined), ...
    'pairCount', numel(pairCombined));
end

function value = meanOrZero(values)
if isempty(values)
    value = 0;
else
    value = mean(values);
end
end
