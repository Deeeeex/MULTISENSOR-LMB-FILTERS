function test_observation_supported_lmb_set_entry_risk()
% TEST_OBSERVATION_SUPPORTED_LMB_SET_ENTRY_RISK Focused support split.

reference = receiverDistribution([1, 1; 1, 2], [0.90, 0.10]);
candidate = receiverDistribution([1, 1; 1, 2], [0.90, 0.70]);
supportedPosterior = {posteriorObjects(0.80)};
unsupportedPosterior = {posteriorObjects(0.00)};
[supportedRisk, supported] = ...
    computeObservationSupportedLmbSetEntryRisk( ...
        {reference}, {candidate}, supportedPosterior, 1);
[unsupportedRisk, unsupported] = ...
    computeObservationSupportedLmbSetEntryRisk( ...
        {reference}, {candidate}, unsupportedPosterior, 1);
assert(supported.entryCount == 1);
assert(supported.unsupportedEntryCount == 0);
assert(supportedRisk == 0);
assert(unsupported.unsupportedEntryCount == 1);
assert(unsupportedRisk > 0);

networkPosterior = {posteriorObjects(0.00), posteriorObjects(0.90)};
[networkRisk, network] = ...
    computeObservationSupportedLmbSetEntryRisk( ...
        {reference, reference}, {candidate, reference}, ...
        networkPosterior, [1, 2], struct( ...
            'supportMode', 'receiver-or-cross-sender', ...
            'fusionWeights', [0.5, 0.5; 0.5, 0.5]));
assert(network.unsupportedEntryCount(1) == 0);
assert(networkRisk == 0);
fprintf('test_observation_supported_lmb_set_entry_risk passed.\n');
end

function distribution = receiverDistribution(labels, existence)
distribution = struct();
distribution.probability = 1;
distribution.summary = {struct( ...
    'labels', labels, 'existence', existence)};
end

function objects = posteriorObjects(secondSupport)
template = struct( ...
    'birthTime', 1, 'birthLocation', 1, ...
    'numberOfGmComponents', 1, 'detectionAssociationMass', 0.9);
objects = repmat(template, 1, 2);
objects(2).birthLocation = 2;
objects(2).detectionAssociationMass = secondSupport;
end
