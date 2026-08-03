function [adjacency, details] = ...
    selectTemporalFormationBackboneConditionalInputBundleSuspensionPolicy( ...
        context, suspendedFormationIds, options)
% SELECTTEMPORALFORMATIONBACKBONECONDITIONALINPUTBUNDLESUSPENSIONPOLICY
% V40 conditional-preserving one-step input-bundle intervention.
%
% The frozen v38 route constructor determines the physical edge removal and
% rolling-B3 feasibility.  V40 changes only the fusion counterfactual.  For
% every removed residual input, the remaining reference row is divided by
% its remaining mass.  Hence the candidate row equals the reference row
% used by the runtime when that residual input is missing under the
% registered `renormalize` missing-message rule.

if nargin < 2 || isempty(suspendedFormationIds)
    suspendedFormationIds = zeros(1, 0);
end
if nargin < 3 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), {'policyConfig'}))
    error('FormationConditionalBundle:InvalidOptions', ...
        'Only the frozen v40 policyConfig may be supplied.');
end
policy = getFormationBackboneConditionalBundlePolicyConfig();
if isfield(options, 'policyConfig') && ...
        (~isstruct(options.policyConfig) || ...
         ~isscalar(options.policyConfig) || ...
         ~isequaln(options.policyConfig, policy))
    error('FormationConditionalBundle:InvalidPolicyConfig', ...
        'The supplied v40 policy config is not registered.');
end
basePolicy = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
baseOptions = struct('policyConfig', basePolicy);
[adjacency, baseDetails] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, suspendedFormationIds, baseOptions);

referenceWeights = baseDetails.referenceFusionWeights;
fusionWeights = referenceWeights;
nodeCount = size(referenceWeights, 1);
renormalizationScaleByReceiver = ones(1, nodeCount);
remainingReferenceMassByReceiver = ones(1, nodeCount);
for edgeIdx = 1:numel(baseDetails.suspendedReceivers)
    receiver = baseDetails.suspendedReceivers(edgeIdx);
    sender = baseDetails.suspendedSenders(edgeIdx);
    removedWeight = referenceWeights(receiver, sender);
    if removedWeight <= 0 || ...
            fusionWeights(receiver, sender) ~= removedWeight
        error('FormationConditionalBundle:InvalidReference', ...
            'A registered residual input cannot be isolated exactly.');
    end
    fusionWeights(receiver, sender) = 0;
    remainingMass = sum(fusionWeights(receiver, :));
    if ~isfinite(remainingMass) || remainingMass <= 0 || ...
            abs(remainingMass - (1 - removedWeight)) > 1e-12
        error('FormationConditionalBundle:InvalidReference', ...
            'The remaining reference row has invalid mass.');
    end
    fusionWeights(receiver, :) = ...
        fusionWeights(receiver, :) / remainingMass;
    remainingReferenceMassByReceiver(receiver) = remainingMass;
    renormalizationScaleByReceiver(receiver) = 1 / remainingMass;
end

weightSupport = adjacency | logical(eye(nodeCount));
if any(~isfinite(fusionWeights(:))) || ...
        any(fusionWeights(:) < -1e-12) || ...
        any(fusionWeights(:) > 1e-12 & ~weightSupport(:)) || ...
        any(abs(sum(fusionWeights, 2) - 1) > 1e-12)
    error('FormationConditionalBundle:InvalidCandidate', ...
        'The conditional-preserving fusion rows are invalid.');
end

details = baseDetails;
details.contractVersion = ...
    'temporal-formation-conditional-input-bundle-suspension-v1';
details.mode = ...
    'temporal-formation-conditional-input-bundle-suspension';
details.actionName = buildActionName(suspendedFormationIds);
details.policyConfig = policy;
details.policyConfigSha256 = policy.canonicalSha256;
details.basePolicyConfigSha256 = basePolicy.canonicalSha256;
details.fusionWeightMatrix = fusionWeights;
details.removedWeightRedistributionMode = ...
    policy.suspensionReweightingMode;
details.remainingReferenceMassByReceiver = ...
    remainingReferenceMassByReceiver;
details.renormalizationScaleByReceiver = ...
    renormalizationScaleByReceiver;
details.referenceMissingInputEquivalent = true;
details.referenceMissingInputEquivalenceMode = ...
    'same-row-conditional-on-removed-input-not-delivered';
details.relativeSelfTrustChangedBeyondConditioning = false;
details.entryDisagreementGateApplied = false;
details.entryDisagreementDiagnosticOnly = true;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end

function name = buildActionName(formationIds)
if isempty(formationIds)
    name = 'reference';
    return;
end
text = sprintf('f%d-', reshape(formationIds, 1, []));
name = ['conditional-suspend-input-bundle-', text(1:end-1)];
end
