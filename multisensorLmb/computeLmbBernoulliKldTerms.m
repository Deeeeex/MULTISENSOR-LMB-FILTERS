function terms = computeLmbBernoulliKldTerms( ...
        referenceExistence, candidateExistence, spatialKld)
% COMPUTELMBBERNOULLIKLDTERMS Reference-to-candidate LMB label terms.
%
% For matched label spaces, the LMB KLD is the sum of these Bernoulli
% terms. spatialKld contains D_KL(p_reference || p_candidate) per label.

referenceExistence = reshape(referenceExistence, 1, []);
candidateExistence = reshape(candidateExistence, 1, []);
spatialKld = reshape(spatialKld, 1, []);
if isempty(referenceExistence) || ...
        numel(candidateExistence) ~= numel(referenceExistence) || ...
        numel(spatialKld) ~= numel(referenceExistence) || ...
        any(~isfinite(referenceExistence)) || ...
        any(~isfinite(candidateExistence)) || ...
        any(referenceExistence < 0 | referenceExistence > 1) || ...
        any(candidateExistence < 0 | candidateExistence > 1) || ...
        any(isnan(spatialKld)) || any(spatialKld < 0)
    error('LmbBernoulliKld:InvalidInput', ...
        'Existence probabilities and nonnegative spatial KLDs must match.');
end

absent = zeros(size(referenceExistence));
present = zeros(size(referenceExistence));
spatial = zeros(size(referenceExistence));
for labelIdx = 1:numel(referenceExistence)
    rReference = referenceExistence(labelIdx);
    rCandidate = candidateExistence(labelIdx);
    absent(labelIdx) = relativeEntropyMass( ...
        1 - rReference, 1 - rCandidate);
    present(labelIdx) = relativeEntropyMass( ...
        rReference, rCandidate);
    if rReference > 0
        spatial(labelIdx) = rReference * spatialKld(labelIdx);
    end
end

perLabel = absent + present + spatial;
terms = struct();
terms.contractVersion = 'lmb-bernoulli-kld-terms-v1';
terms.absentExistence = absent;
terms.presentExistence = present;
terms.spatial = spatial;
terms.perLabel = perLabel;
terms.total = sum(perLabel);
terms.referenceToCandidate = true;
terms.additiveAcrossLabels = true;
terms.additiveAcrossSenderEdges = false;
end

function value = relativeEntropyMass(referenceMass, candidateMass)
if referenceMass == 0
    value = 0;
elseif candidateMass == 0
    value = Inf;
else
    value = referenceMass * log(referenceMass / candidateMass);
end
end
