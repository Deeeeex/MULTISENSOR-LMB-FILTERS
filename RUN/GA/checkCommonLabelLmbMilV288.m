function checkCommonLabelLmbMilV288()
% Focused formula/dispatch checks, not a tracking-performance claim.
a = struct('birthTime', 1, 'birthLocation', 1, 'r', 0.2, ...
    'numberOfGmComponents', 1, 'w', 1, 'mu', {{[0; 0; 0; 0]}}, ...
    'Sigma', {{eye(4)}});
b = a; b.r = 0.8; b.mu = {[10; 0; 0; 0]};
model = struct('object', a([]), 'xDimension', 4);
config = struct('lmbFusionRule', 'mil-common-label', ...
    'mixtureAwareMaxFusedComponents', 8);
[f, d] = fuseLmbPosteriorsByLabel({a, b}, [0.25, 0.75], model, [], [], config);
assert(abs(f.r - 0.65) < 1e-12 && f.numberOfGmComponents == 2);
assert(norm(f.w - [0.6, 0.05] / 0.65) < 1e-12 && ...
    isequal(f.mu{1}, b.mu{1}) && d.milTruncatedLabelCount == 0);
% Missing Bernoulli is zero existence: retain source weight, do not veto.
[f, d] = fuseLmbPosteriorsByLabel({a, a([])}, [0.25, 0.75], model, [], [], config);
assert(abs(f.r - 0.05) < 1e-12 && d.missingSourceCount == 1);
other = b; other.birthLocation = 2;
f = fuseLmbPosteriorsByLabel({a, other}, [0.25, 0.75], model, [], [], config);
assert(numel(f) == 2 && norm([f.r] - [0.05, 0.6]) < 1e-12);
% Known rule is exact before a reported, not hidden, truncation.
config.mixtureAwareMaxFusedComponents = 1;
[f, d] = fuseLmbPosteriorsByLabel({a, b}, [0.25, 0.75], model, [], [], config);
assert(f.w == 1 && abs(f.r - 0.65) < 1e-12 && ...
    abs(d.milDiscardedSpatialMassSum - 0.05 / 0.65) < 1e-12);
f = fuseLmbPosteriorsByLabel({a, a}, [0.25, 0.75], model, [], [], config);
assert(f.numberOfGmComponents == 1 && abs(f.r - a.r) < 1e-12);
threw = false;
try
    fuseLmbPosteriorsByLabel({a, b}, [0.25, 0.75], model, [0.5, 0.5], [], config);
catch err
    threw = strcmp(err.identifier, 'LmbMil:IncompatibleKlaOverride');
end
assert(threw);
% Default receiver remains the existing KLA path.
default = fuseLmbPosteriorsByLabel({a, b}, [0.25, 0.75], model);
explicit = fuseLmbPosteriorsByLabel( ...
    {a, b}, [0.25, 0.75], model, [], [], struct('lmbFusionRule', 'kla'));
assert(isequal(default, explicit));
fprintf('V288 formula self-check PASS: weighted existence/GM, absent labels, reduction, override guard, default KLA.\n');
end
