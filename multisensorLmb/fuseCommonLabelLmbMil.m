function [fusedObjects, diagnostics] = fuseCommonLabelLmbMil( ...
    posteriorDistributions, weights, model, maximumComponents)
% FUSECOMMONLABELLMBMIL LMB-constrained minimum-information-loss baseline.
% Gao et al., arXiv:1911.01083v1, Proposition 3, equations (26)--(28).
% r = sum_i w_i r_i; p = sum_i w_i r_i p_i / r.
% Absent labels are zero-existence Bernoullis, not removed source weights.
% Shared labels are preserved. This is not label matching or a FoV partition.
% GM concatenation is exact before the explicitly diagnosed output truncation.

sourceCount = numel(posteriorDistributions);
weights = reshape(weights, 1, []);
diagnostics = struct('fusedLabelCount', 0, 'missingSourceCount', 0, ...
    'labelsWithMissingSourceCount', 0, 'truncatedLabelCount', 0, ...
    'discardedSpatialMassSum', 0, 'maximumDiscardedSpatialMass', 0);
if sourceCount == 0
    fusedObjects = model.object;
    return;
end
assert(numel(weights) == sourceCount && all(isfinite(weights)) && ...
    all(weights >= 0) && sum(weights) > 0, ...
    'LmbMil:InvalidWeights', 'MIL needs nonnegative finite source weights.');
assert(isscalar(maximumComponents) && maximumComponents >= 1 && ...
    maximumComponents == round(maximumComponents));
weights = weights / sum(weights);
fusedObjects = posteriorDistributions{1}([]);
labels = zeros(2, 0);
for s = find(weights > 0)
    objects = posteriorDistributions{s};
    for k = 1:numel(objects)
        if objects(k).numberOfGmComponents < 1, continue; end
        label = [objects(k).birthTime; objects(k).birthLocation];
        if ~any(all(labels == label, 1)), labels(:, end + 1) = label; end %#ok<AGROW>
    end
end
if ~isempty(labels), labels = sortrows(labels')'; end
for ell = 1:size(labels, 2)
    mixtureMass = []; means = {}; covariances = {};
    fusedExistence = 0; template = []; missingCount = 0;
    for s = find(weights > 0)
        objects = posteriorDistributions{s};
        idx = find([objects.birthTime] == labels(1, ell) & ...
            [objects.birthLocation] == labels(2, ell) & ...
            [objects.numberOfGmComponents] > 0);
        assert(numel(idx) <= 1, 'LmbMil:DuplicateLabel', ...
            'Each input may contain at most one Bernoulli per label.');
        if isempty(idx), missingCount = missingCount + 1; continue; end
        object = objects(idx);
        r = object.r;
        assert(isscalar(r) && isfinite(r) && r >= 0 && r <= 1);
        if isempty(template), template = object; end
        if r == 0, continue; end
        localWeights = reshape(object.w, 1, []);
        assert(numel(localWeights) == object.numberOfGmComponents && ...
            numel(object.mu) == numel(localWeights) && ...
            numel(object.Sigma) == numel(localWeights) && ...
            all(isfinite(localWeights)) && all(localWeights >= 0) && ...
            sum(localWeights) > 0, 'LmbMil:InvalidMixture', ...
            'MIL requires a normalized, nonempty represented spatial density.');
        localWeights = localWeights / sum(localWeights);
        fusedExistence = fusedExistence + weights(s) * r;
        mixtureMass = [mixtureMass, weights(s) * r * localWeights]; %#ok<AGROW>
        means = [means, reshape(object.mu, 1, [])]; %#ok<AGROW>
        covariances = [covariances, reshape(object.Sigma, 1, [])]; %#ok<AGROW>
    end
    diagnostics.missingSourceCount = diagnostics.missingSourceCount + missingCount;
    diagnostics.labelsWithMissingSourceCount = ...
        diagnostics.labelsWithMissingSourceCount + (missingCount > 0);
    if fusedExistence <= 0, continue; end
    template.r = fusedExistence;
    template.w = mixtureMass / fusedExistence;
    template.mu = means;
    template.Sigma = covariances;
    template.numberOfGmComponents = numel(mixtureMass);
    % Reuse exact-copy coalescence: duplicate kernels must not consume the
    % cap or make an otherwise identical arithmetic input lose a mode.
    [template, reduction] = canonicalizeLmbGaussianMixtureRepresentation( ...
        template, struct('weightThreshold', 0, ...
            'maximumComponentCount', maximumComponents));
    discardedMass = reduction.discardedNormalizedWeightMass;
    diagnostics.truncatedLabelCount = ...
        diagnostics.truncatedLabelCount + (discardedMass > 0);
    diagnostics.discardedSpatialMassSum = ...
        diagnostics.discardedSpatialMassSum + discardedMass;
    diagnostics.maximumDiscardedSpatialMass = ...
        max(diagnostics.maximumDiscardedSpatialMass, discardedMass);
    fusedObjects(end + 1) = template; %#ok<AGROW>
    diagnostics.fusedLabelCount = diagnostics.fusedLabelCount + 1;
end
end
