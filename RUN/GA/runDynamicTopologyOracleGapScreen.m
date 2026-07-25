function [reportPath, summary] = ...
    runDynamicTopologyOracleGapScreen(presetName, seedList, options)
% RUNDYNAMICTOPOLOGYORACLEGAPSCREEN Paired topology-only evidence gate.
%
% Example:
%   opts = struct('maxTimeSteps', 20);
%   runDynamicTopologyOracleGapScreen('d12-handover', 7, opts);
%
% The same measurements, trajectories and link uniforms are reused across
% arms. Payload compression and event triggering are disabled; only the
% selected topology changes.

if nargin < 1 || isempty(presetName)
    presetName = 'd12-handover';
end
if nargin < 2 || isempty(seedList)
    seedList = 1:3;
end
if nargin < 3 || isempty(options)
    options = struct();
end
seedList = reshape(seedList, 1, []);
options = resolveOptions(options);
armDefinitions = buildArms(options.armNames);

fprintf(['Dynamic topology screen: %s, %d seed(s), ', ...
    '%d arm(s), maxTimeSteps=%s\n'], ...
    presetName, numel(seedList), numel(armDefinitions), ...
    numericOrAll(options.maxTimeSteps));

records = repmat(emptyRecord(), ...
    numel(seedList), numel(armDefinitions));
validationBySeed = cell(1, numel(seedList));
for seedIdx = 1:numel(seedList)
    seed = seedList(seedIdx);
    fprintf('Generating paired input for seed %d...\n', seed);
    inputs = generateDynamicTopologyScenarioInputs( ...
        presetName, seed, options.scenarioOverrides);
    inputs = cropInputs(inputs, options.maxTimeSteps);
    validationBySeed{seedIdx} = inputs.validation;
    for armIdx = 1:numel(armDefinitions)
        arm = armDefinitions(armIdx);
        fprintf('  seed %d arm %s...\n', seed, arm.name);
        triggerConfig = buildArmTriggerConfig( ...
            arm, inputs.config, options.fusionOverrides);
        rng(seed + options.filterSeedOffset);
        timerId = tic;
        [stateEstimates, diagnostics] = ...
            runEventTriggeredDistributedLmbFilter( ...
                inputs.model, inputs.measurements, ...
                inputs.sensorTrajectories, inputs.neighborMap, ...
                inputs.commConfig, triggerConfig);
        elapsedSeconds = toc(timerId);
        record = scoreRun( ...
            seed, arm, stateEstimates, diagnostics, ...
            inputs.model, inputs.groundTruthRfs, elapsedSeconds);
        records(seedIdx, armIdx) = record;
        fprintf(['    E-OSPA %.4f, consensus OSPA %.4f, ', ...
            'attempted %.0f B, %.2f s\n'], ...
            record.meanEospa, record.consensusOspa, ...
            record.attemptedBytes, record.elapsedSeconds);
    end
end

summary = aggregateRecords( ...
    presetName, seedList, armDefinitions, records, ...
    validationBySeed, options);
reportPath = '';
if options.writeReport
    outputDirectory = options.outputDirectory;
    if ~exist(outputDirectory, 'dir')
        mkdir(outputDirectory);
    end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    stem = sprintf('DYNAMIC_TOPOLOGY_ORACLE_GAP_%s_N%d_%s', ...
        upper(strrep(char(presetName), '-', '_')), ...
        numel(seedList), timestamp);
    reportPath = fullfile(outputDirectory, [stem, '.md']);
    matPath = fullfile(outputDirectory, [stem, '.mat']);
    writeReport(reportPath, summary);
    save(matPath, 'summary');
    summary.reportPath = reportPath;
    summary.matPath = matPath;
    fprintf('Report: %s\n', reportPath);
end
end

function options = resolveOptions(options)
options.armNames = getField(options, 'armNames', { ...
    'local', 'robust-static', 'reliability', 'discrepancy', ...
    'oracle-consensus', 'oracle-truth'});
options.maxTimeSteps = getField(options, 'maxTimeSteps', inf);
options.writeReport = getField(options, 'writeReport', true);
options.scenarioOverrides = getField( ...
    options, 'scenarioOverrides', struct());
options.fusionOverrides = getField( ...
    options, 'fusionOverrides', struct());
options.outputDirectory = getField(options, 'outputDirectory', ...
    fullfile('RUN', 'GA', 'dynamic_topology'));
options.filterSeedOffset = getField(options, 'filterSeedOffset', 100000);
end

function arms = buildArms(names)
if ischar(names)
    names = {names};
end
arms = repmat(struct('name', '', 'mode', ''), 1, numel(names));
for armIdx = 1:numel(names)
    mode = lower(strrep(char(names{armIdx}), '_', '-'));
    switch mode
        case 'local'
            displayName = 'Local only';
        case 'robust-static'
            displayName = 'Robust geometry static';
        case 'reliability'
            displayName = 'Reliability dynamic';
        case 'discrepancy'
            displayName = 'Posterior discrepancy dynamic';
        case 'oracle-consensus'
            displayName = 'Exact one-step consensus oracle';
        case 'oracle-truth'
            displayName = 'Exact one-step truth diagnostic oracle';
        otherwise
            error('Unknown topology screen arm: %s', names{armIdx});
    end
    arms(armIdx) = struct('name', displayName, 'mode', mode);
end
end

function config = buildArmTriggerConfig(arm, scenarioConfig, overrides)
config = buildMixtureAwareKlaReferenceConfig(overrides);
config.dynamicTopologyEnabled = false;
config.dynamicTopologyEdgeBudget = scenarioConfig.edgeBudget;
config.topologyMaxEdgeChangeFraction = inf;
switch arm.mode
    case 'local'
        config.eventPolicy = 'none';
    case 'robust-static'
        config.eventPolicy = 'alwaysHeavy';
    case {'reliability', 'discrepancy', ...
            'oracle-consensus', 'oracle-truth'}
        config.eventPolicy = 'alwaysHeavy';
        config.dynamicTopologyEnabled = true;
        config.topologyPolicyName = arm.mode;
        mode = arm.mode;
        config.topologyPolicyFcn = ...
            @(context) selectD12TopologyPolicy(context, mode);
end
end

function inputs = cropInputs(inputs, maxTimeSteps)
availableTime = size(inputs.measurements, 2);
if ~isfinite(maxTimeSteps) || maxTimeSteps >= availableTime
    return;
end
timeCount = max(1, round(maxTimeSteps));
inputs.measurements = inputs.measurements(:, 1:timeCount);
for sensorIdx = 1:numel(inputs.sensorTrajectories)
    inputs.sensorTrajectories{sensorIdx} = ...
        inputs.sensorTrajectories{sensorIdx}(:, 1:timeCount);
end
fields = {'x', 'mu', 'Sigma'};
for fieldIdx = 1:numel(fields)
    fieldName = fields{fieldIdx};
    inputs.groundTruthRfs.(fieldName) = ...
        inputs.groundTruthRfs.(fieldName)(1:timeCount);
end
inputs.groundTruthRfs.cardinality = ...
    inputs.groundTruthRfs.cardinality(1:timeCount);
if ndims(inputs.commConfig.pDropByEdge) >= 3
    inputs.commConfig.pDropByEdge = ...
        inputs.commConfig.pDropByEdge(:, :, 1:timeCount);
end
inputs.commConfig.linkUniforms = ...
    inputs.commConfig.linkUniforms(:, :, 1:timeCount);
inputs.model.simulationLength = timeCount;
end

function record = scoreRun( ...
    seed, arm, estimates, diagnostics, model, truth, elapsedSeconds)
sensorCount = numel(estimates);
timeCount = numel(truth.x);
eospa = zeros(sensorCount, timeCount);
cardinalityError = zeros(sensorCount, timeCount);
for sensorIdx = 1:sensorCount
    [sensorEospa, ~, cardinality] = ...
        computeSimulationOspa(model, truth, estimates{sensorIdx});
    eospa(sensorIdx, :) = sensorEospa;
    cardinalityError(sensorIdx, :) = abs( ...
        cardinality - truth.cardinality);
end
[positionDisagreement, cardinalityDispersion, ospaDisagreement] = ...
    computeDistributedConsensusMetrics(estimates, model);
sensorMeanEospa = mean(eospa, 2);

record = emptyRecord();
record.seed = seed;
record.armName = arm.name;
record.armMode = arm.mode;
record.meanEospa = mean(eospa(:));
record.worstSensorEospa = max(sensorMeanEospa);
record.meanCardinalityError = mean(cardinalityError(:));
record.consensusOspa = finiteMean(ospaDisagreement);
record.consensusPosition = finiteMean(positionDisagreement);
record.cardinalityDispersion = finiteMean(cardinalityDispersion);
record.posteriorConsensus = ...
    diagnostics.summary.posteriorConsensusCombined;
record.posteriorExistenceConsensus = ...
    diagnostics.summary.posteriorConsensusExistence;
record.posteriorSpatialConsensus = ...
    diagnostics.summary.posteriorConsensusSpatial;
record.attemptedBytes = diagnostics.summary.attemptedPayloadBytes;
record.deliveredBytes = diagnostics.summary.payloadBytes;
record.attemptCount = diagnostics.summary.attemptCount;
record.deliveryCount = diagnostics.summary.deliveryCount;
record.meanEdgeCount = diagnostics.summary.meanUndirectedEdgeCount;
record.topologyChurnRate = diagnostics.summary.topologyChurnRate;
record.topologyInfeasibleRate = ...
    diagnostics.summary.topologyInfeasibleRate;
record.topologyPolicySeconds = ...
    diagnostics.summary.topologyPolicySeconds;
record.elapsedSeconds = elapsedSeconds;
candidateIndices = diagnostics.topologyPolicyCandidateIndex;
candidateIndices = candidateIndices(isfinite(candidateIndices));
record.distinctCandidateCount = numel(unique(candidateIndices));
end

function record = emptyRecord()
record = struct( ...
    'seed', 0, ...
    'armName', '', ...
    'armMode', '', ...
    'meanEospa', NaN, ...
    'worstSensorEospa', NaN, ...
    'meanCardinalityError', NaN, ...
    'consensusOspa', NaN, ...
    'consensusPosition', NaN, ...
    'cardinalityDispersion', NaN, ...
    'posteriorConsensus', NaN, ...
    'posteriorExistenceConsensus', NaN, ...
    'posteriorSpatialConsensus', NaN, ...
    'attemptedBytes', NaN, ...
    'deliveredBytes', NaN, ...
    'attemptCount', NaN, ...
    'deliveryCount', NaN, ...
    'meanEdgeCount', NaN, ...
    'topologyChurnRate', NaN, ...
    'topologyInfeasibleRate', NaN, ...
    'topologyPolicySeconds', NaN, ...
    'elapsedSeconds', NaN, ...
    'distinctCandidateCount', 0);
end

function summary = aggregateRecords( ...
    presetName, seeds, arms, records, validations, options)
metricNames = { ...
    'meanEospa', 'worstSensorEospa', 'meanCardinalityError', ...
    'consensusOspa', 'consensusPosition', 'cardinalityDispersion', ...
    'posteriorConsensus', 'posteriorExistenceConsensus', ...
    'posteriorSpatialConsensus', ...
    'attemptedBytes', 'deliveredBytes', 'attemptCount', ...
    'deliveryCount', 'meanEdgeCount', 'topologyChurnRate', ...
    'topologyInfeasibleRate', 'topologyPolicySeconds', ...
    'elapsedSeconds', 'distinctCandidateCount'};
aggregates = repmat(struct(), 1, numel(arms));
for armIdx = 1:numel(arms)
    aggregates(armIdx).name = arms(armIdx).name;
    aggregates(armIdx).mode = arms(armIdx).mode;
    for metricIdx = 1:numel(metricNames)
        metricName = metricNames{metricIdx};
        values = reshape([records(:, armIdx).(metricName)], 1, []);
        aggregates(armIdx).(metricName) = mean(values);
        aggregates(armIdx).([metricName, 'BySeed']) = values;
    end
end

decision = computeScreenDecision(aggregates, numel(seeds));
summary = struct();
summary.generatedAt = datestr(now, 31);
summary.presetName = presetName;
summary.seeds = seeds;
summary.options = options;
summary.armNames = {arms.name};
summary.records = records;
summary.aggregate = aggregates;
summary.validationBySeed = validations;
summary.decision = decision;
summary.referenceBoundary = [ ...
    'Mixture-aware results use the repository componentwise ', ...
    'powered-GM KLA approximation. This preserves multiple modes but ', ...
    'is not an exact arbitrary-mixture density power.'];
end

function decision = computeScreenDecision(aggregates, seedCount)
staticIdx = findMode(aggregates, 'robust-static');
analyticIdx = [findMode(aggregates, 'reliability'), ...
    findMode(aggregates, 'discrepancy')];
oracleIdx = findMode(aggregates, 'oracle-consensus');
truthOracleIdx = findMode(aggregates, 'oracle-truth');
decision = struct( ...
    'status', 'insufficient-arms', ...
    'oracleConsensusGainPercent', NaN, ...
    'oracleTrackingGainPercent', NaN, ...
    'analyticOracleGainCapture', NaN, ...
    'attemptedByteMismatchPercent', NaN, ...
    'communicationMatchedWithinTwoPercent', false, ...
    'recommendation', 'Run the complete registered arm set.');
if isempty(staticIdx) || isempty(analyticIdx) || isempty(oracleIdx)
    return;
end

candidateIdx = [staticIdx, analyticIdx];
[bestConsensus, bestConsensusCursor] = min( ...
    [aggregates(candidateIdx).posteriorConsensus]);
bestConsensusIdx = candidateIdx(bestConsensusCursor);
[bestTracking, ~] = min([aggregates(candidateIdx).meanEospa]);
oracleConsensus = aggregates(oracleIdx).posteriorConsensus;
if isempty(truthOracleIdx)
    oracleTracking = aggregates(oracleIdx).meanEospa;
else
    oracleTracking = aggregates(truthOracleIdx).meanEospa;
end
decision.oracleConsensusGainPercent = percentImprovement( ...
    bestConsensus, oracleConsensus);
decision.oracleTrackingGainPercent = percentImprovement( ...
    bestTracking, oracleTracking);
decision.attemptedByteMismatchPercent = 100 * abs( ...
    aggregates(bestConsensusIdx).attemptedBytes - ...
    aggregates(oracleIdx).attemptedBytes) / max( ...
    aggregates(bestConsensusIdx).attemptedBytes, 1);
decision.communicationMatchedWithinTwoPercent = ...
    decision.attemptedByteMismatchPercent <= 2;

bestAnalytic = min([aggregates(analyticIdx).posteriorConsensus]);
staticValue = aggregates(staticIdx).posteriorConsensus;
oracleDenominator = staticValue - oracleConsensus;
if oracleDenominator > 0
    decision.analyticOracleGainCapture = ...
        (staticValue - bestAnalytic) / oracleDenominator;
end

if seedCount < 10
    decision.status = 'screening-only';
    decision.recommendation = [ ...
        'Treat this as a runtime and signal check; at least 10 paired ', ...
        'screening seeds are required before a research stop decision.'];
elseif ~decision.communicationMatchedWithinTwoPercent
    decision.status = 'communication-mismatch';
    decision.recommendation = [ ...
        'Compare performance on an attempted-byte-matched curve before ', ...
        'interpreting the oracle gap.'];
elseif decision.oracleConsensusGainPercent < 10 && ...
        decision.oracleTrackingGainPercent < 5
    decision.status = 'stop-no-oracle-gap';
    decision.recommendation = [ ...
        'The registered practical gap is absent; do not train a GNN on ', ...
        'this scene without redesigning the information handover.'];
elseif isfinite(decision.analyticOracleGainCapture) && ...
        decision.analyticOracleGainCapture >= 0.90
    decision.status = 'stop-analytic-sufficient';
    decision.recommendation = [ ...
        'The analytic posterior heuristic captures at least 90% of the ', ...
        'oracle consensus gain; prefer the simpler method.'];
else
    decision.status = 'continue-learned-policy';
    decision.recommendation = [ ...
        'A residual oracle gap remains after strong analytic controls; ', ...
        'proceed to learned edge-value prediction with safe projection.'];
end
end

function writeReport(path, summary)
fid = fopen(path, 'w');
if fid < 0
    error('Could not open report path: %s', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '# Dynamic-topology oracle-gap screen\n\n');
fprintf(fid, '- Preset: `%s`\n', summary.presetName);
fprintf(fid, '- Seeds: `%s`\n', mat2str(summary.seeds));
fprintf(fid, '- Generated: %s\n', summary.generatedAt);
fprintf(fid, '- Decision status: `%s`\n\n', summary.decision.status);
fprintf(fid, '%s\n\n', summary.referenceBoundary);
fprintf(fid, ['| Arm | E-OSPA | Worst node | ', ...
    'Posterior disagreement | MAP-set disagreement | Card. error | ', ...
    'Attempted bytes | Delivered bytes | ', ...
    'Edges | Churn | Infeasible | Policy s | Total s |\n']);
fprintf(fid, ['|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|', ...
    '--:|--:|--:|--:|\n']);
for armIdx = 1:numel(summary.aggregate)
    arm = summary.aggregate(armIdx);
    fprintf(fid, ['| %s | %.4f | %.4f | %.4f | %.4f | %.4f | ', ...
        '%.0f | %.0f | %.2f | %.4f | %.4f | %.2f | %.2f |\n'], ...
        arm.name, arm.meanEospa, arm.worstSensorEospa, ...
        arm.posteriorConsensus, arm.consensusOspa, ...
        arm.meanCardinalityError, ...
        arm.attemptedBytes, arm.deliveredBytes, arm.meanEdgeCount, ...
        arm.topologyChurnRate, arm.topologyInfeasibleRate, ...
        arm.topologyPolicySeconds, arm.elapsedSeconds);
end

fprintf(fid, '\n## Registered gate readout\n\n');
fprintf(fid, '- Oracle consensus gain: %.2f%%\n', ...
    summary.decision.oracleConsensusGainPercent);
fprintf(fid, '- Oracle tracking gain: %.2f%%\n', ...
    summary.decision.oracleTrackingGainPercent);
fprintf(fid, '- Analytic share of static-to-oracle gain: %.3f\n', ...
    summary.decision.analyticOracleGainCapture);
fprintf(fid, '- Attempted-byte mismatch: %.2f%%\n', ...
    summary.decision.attemptedByteMismatchPercent);
fprintf(fid, '- Recommendation: %s\n', ...
    summary.decision.recommendation);

fprintf(fid, '\n## Evidence limits\n\n');
fprintf(fid, ['- This runner isolates topology: every active edge sends the ', ...
    'same heavy posterior every step; event triggering and payload ', ...
    'compression are disabled.\n']);
fprintf(fid, ['- A one- or three-seed run is a software/runtime screen, not ', ...
    'a paper-level effect estimate. The registered screening gate needs ', ...
    'at least 10 paired seeds; the held-out claim needs 30.\n']);
fprintf(fid, ['- Equal edge budgets do not guarantee equal bytes. The table ', ...
    'therefore reports attempted payload bytes explicitly.\n']);
end

function idx = findMode(aggregates, mode)
idx = find(strcmp({aggregates.mode}, mode));
end

function value = finiteMean(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = mean(values);
end
end

function value = percentImprovement(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), eps);
end

function value = numericOrAll(number)
if isfinite(number)
    value = num2str(number);
else
    value = 'all';
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
