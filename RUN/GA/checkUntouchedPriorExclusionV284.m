function checkUntouchedPriorExclusionV284()
% A few semantic self-checks; this is not a performance or novelty test.
plain = struct('birthTime', 1, 'birthLocation', 1, 'r', 0.2, ...
    'numberOfGmComponents', 1, 'w', 1, 'mu', {{[40; 0; 0; 0]}}, ...
    'Sigma', {{eye(4)}});
empty = initializeLmbObservationLineageV284(plain([]));
assert(isempty(empty) && isfield(empty, 'hasObservationLineage'));
prior = initializeLmbObservationLineageV284(plain);
assert(~prior.hasObservationLineage);
model = struct('object', empty, 'xDimension', 4, ...
    'existenceThreshold', 0.01, 'detectionProbability', 0.8, ...
    'Q', {{eye(2)}}, 'sensorInitialStates', {{[0; 0; 1; 0]}}, ...
    'sensorMotionEnabled', true, 'sensorFovEnabled', true, ...
    'sensorFovHalfAngleDeg', 60, 'sensorFovRange', 100);
informed = markLmbObservationLineageV284(prior, prior, model, 1, 1, true);
assert(informed.hasObservationLineage); % opportunity, not detection success
unscheduled = markLmbObservationLineageV284(prior, prior, model, 1, 1, false);
assert(~unscheduled.hasObservationLineage);
outside = prior; outside.mu = {[-40; 0; 0; 0]};
outside = markLmbObservationLineageV284(outside, outside, model, 1, 1, true);
assert(~outside.hasObservationLineage);
historical = outside; historical.hasObservationLineage = true;
historical = markLmbObservationLineageV284( ...
    historical, historical, model, 1, 1, true);
assert(historical.hasObservationLineage);

informed.r = 0.8;
[sp, ep, flag, excluded] = restrictUntouchedLmbPriorsV284( ...
    {prior, informed}, [true, true], [true, true], [0.5, 0.5], [NaN, NaN]);
assert(isequal(sp, [false, true]) && isequal(ep, sp) && flag && excluded == 1);
[sp, ep, flag, excluded] = restrictUntouchedLmbPriorsV284( ...
    {prior, informed}, [true, true], [true, true], [1, 0], [NaN, NaN]);
assert(all(sp) && all(ep) && ~flag && excluded == 0);
[sp, ep, flag, excluded] = restrictUntouchedLmbPriorsV284( ...
    {prior, empty}, [true, false], [true, true], [0.5, 0.5], [NaN, 0.01]);
assert(~any(sp) && isequal(ep, [false, true]) && flag && excluded == 1);

enabled = struct('untouchedPriorExclusionEnabled', true);
[selected, diagnostic] = fuseLmbPosteriorsByLabel( ...
    {prior, informed}, [0.5, 0.5], model, [], [], enabled);
assert(numel(selected) == 1 && selected.hasObservationLineage && ...
    abs(selected.r - informed.r) < 1e-12 && ...
    diagnostic.lineageExcludedSourceCount == 1);
plainInformed = rmfield(informed, 'hasObservationLineage');
ordinary = fuseLmbPosteriorsByLabel( ...
    {plain, plainInformed}, [0.5, 0.5], model);
disabled = fuseLmbPosteriorsByLabel( ...
    {prior, informed}, [0.5, 0.5], model);
assert(abs(ordinary.r - disabled.r) < 1e-12 && ...
    isequal(ordinary.mu, disabled.mu) && isequal(ordinary.Sigma, disabled.Sigma));
uninformed = informed; uninformed.hasObservationLineage = false;
fallback = fuseLmbPosteriorsByLabel( ...
    {prior, uninformed}, [0.5, 0.5], model, [], [], enabled);
assert(~fallback.hasObservationLineage && abs(fallback.r - ordinary.r) < 1e-12);
baseBytes = estimateLmbPayloadSize(plain, model, 2);
newBytes = estimateLmbPayloadSize(prior, model, 2);
assert(newBytes.estimatedBytes == baseBytes.estimatedBytes + 8 && ...
    newBytes.lineageMetadataBytes == 8);
fprintf('V284 semantic self-check PASS: opportunity history, participation, fallback and byte accounting.\n');
end
