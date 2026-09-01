function test_canonical_lmb_gm_representation()
% TEST_CANONICAL_LMB_GM_REPRESENTATION Core representation invariants.

setPath;

fprintf('Test 1: exact component splitting does not change canonical output\n');
base = buildObject([0.6, 0.4], [0, 10]);
split = buildObject([0.2, 0.4, 0.4], [0, 0, 10]);
[baseCanonical, baseDiagnostics] = ...
    canonicalizeLmbGaussianMixtureRepresentation(base, struct());
[splitCanonical, splitDiagnostics] = ...
    canonicalizeLmbGaussianMixtureRepresentation(split, struct());
assert(baseCanonical.numberOfGmComponents == 2);
assert(splitCanonical.numberOfGmComponents == 2);
assert(max(abs(baseCanonical.w - splitCanonical.w)) < 1e-15);
assert(isequal(baseCanonical.mu, splitCanonical.mu));
assert(isequal(baseCanonical.Sigma, splitCanonical.Sigma));
assert(baseDiagnostics.exactDensityPreserved);
assert(splitDiagnostics.exactDensityPreserved);
assert(splitDiagnostics.identicalCopyCount == 1);

fprintf('Test 2: exact copies merge before the component cap\n');
objects = buildObject(1, 0);
posteriorParameters = struct( ...
    'w', [0.40; 0.35; 0.25], ...
    'mu', {{stateAt(0), stateAt(0), stateAt(10)}}, ...
    'Sigma', {{eye(4), eye(4), eye(4)}});
model = struct( ...
    'gmWeightThreshold', 0, ...
    'maximumNumberOfGmComponents', 2);
posterior = computePosteriorLmbSpatialDistributions( ...
    objects, 0.9, ones(1, 3), posteriorParameters, model);
assert(posterior.numberOfGmComponents == 2);
assert(max(abs(posterior.w - [0.75, 0.25])) < 1e-15);
assert(posterior.mu{1}(1) == 0);
assert(posterior.mu{2}(1) == 10);

fprintf('Canonical LMB-GM representation tests passed.\n');
end

function object = buildObject(weights, positions)
componentCount = numel(weights);
means = cell(1, componentCount);
covariances = cell(1, componentCount);
for componentIdx = 1:componentCount
    means{componentIdx} = stateAt(positions(componentIdx));
    covariances{componentIdx} = eye(4);
end
object = struct( ...
    'r', 0.9, ...
    'numberOfGmComponents', componentCount, ...
    'w', reshape(weights, 1, []), ...
    'mu', {means}, ...
    'Sigma', {covariances});
end

function state = stateAt(position)
state = [position; 0; 0; 0];
end
