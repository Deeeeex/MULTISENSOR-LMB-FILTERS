function test_dynamic_topology_screen_analysis()
% TEST_DYNAMIC_TOPOLOGY_SCREEN_ANALYSIS Deterministic paired audit smoke.

modes = {'robust-static', 'discrepancy', ...
    'oracle-consensus', 'oracle-truth'};
names = {'Static', 'Discrepancy', 'Consensus oracle', 'Truth oracle'};
records = repmat(makeRecord(), 3, 4);
for seedIdx = 1:3
    for armIdx = 1:4
        records(seedIdx, armIdx).armMode = modes{armIdx};
        records(seedIdx, armIdx).armName = names{armIdx};
        records(seedIdx, armIdx).focusWindowName = 'handover';
        records(seedIdx, armIdx).focusWindow = [35, 95];
        records(seedIdx, armIdx).attemptedBytes = 1e6 * ...
            (1 + 0.005 * (armIdx - 1));
        records(seedIdx, armIdx).topologyInfeasibleRate = 0;
        records(seedIdx, armIdx).topologyChurnRate = ...
            0.01 * (armIdx - 1);
        records(seedIdx, armIdx).distinctCandidateCount = ...
            3 * (armIdx - 1);
    end
    records(seedIdx, 1).focusEospa = 10;
    records(seedIdx, 2).focusEospa = 9.4;
    records(seedIdx, 3).focusEospa = 9.2;
    records(seedIdx, 4).focusEospa = 8.8;
    records(seedIdx, 1).focusPosteriorConsensus = 1.0;
    records(seedIdx, 2).focusPosteriorConsensus = 0.88;
    records(seedIdx, 3).focusPosteriorConsensus = 0.75;
    records(seedIdx, 4).focusPosteriorConsensus = 0.82;
end

aggregate = repmat(struct(), 1, 4);
for armIdx = 1:4
    aggregate(armIdx).mode = modes{armIdx};
    aggregate(armIdx).name = names{armIdx};
    aggregate(armIdx).focusEospa = mean( ...
        [records(:, armIdx).focusEospa]);
    aggregate(armIdx).focusPosteriorConsensus = mean( ...
        [records(:, armIdx).focusPosteriorConsensus]);
    aggregate(armIdx).attemptedBytes = mean( ...
        [records(:, armIdx).attemptedBytes]);
    aggregate(armIdx).topologyInfeasibleRate = 0;
    aggregate(armIdx).elapsedSeconds = armIdx;
end
summary = struct();
summary.seeds = [1, 2, 3];
summary.records = records;
summary.aggregate = aggregate;

matPath = [tempname(), '.mat'];
reportPath = [tempname(), '.md'];
cleanup = onCleanup(@() cleanupFiles(matPath, reportPath)); %#ok<NASGU>
save(matPath, 'summary');
[returnedPath, findings] = ...
    analyzeDynamicTopologyScreen(matPath, reportPath);
assert(strcmp(returnedPath, reportPath));
assert(strcmp(findings.classification.status, 'residual-oracle-gap'));
assert(findings.discrepancyVsStatic.trackingWinCount == 3);
assert(findings.discrepancyVsStatic.byteMatchedCount == 3);
assert(exist(reportPath, 'file') == 2);

% A myopic diagnostic that is dominated by the deployable arm must not be
% interpreted as evidence that the analytic policy is sufficient.
for seedIdx = 1:3
    records(seedIdx, 3).focusEospa = 10.2;
    records(seedIdx, 4).focusEospa = 9.8;
    records(seedIdx, 3).focusPosteriorConsensus = 1.05;
    records(seedIdx, 4).focusPosteriorConsensus = 0.95;
end
for armIdx = 1:4
    aggregate(armIdx).focusEospa = mean( ...
        [records(:, armIdx).focusEospa]);
    aggregate(armIdx).focusPosteriorConsensus = mean( ...
        [records(:, armIdx).focusPosteriorConsensus]);
end
summary.records = records;
summary.aggregate = aggregate;
save(matPath, 'summary');
[~, dominatedFindings] = ...
    analyzeDynamicTopologyScreen(matPath, reportPath);
assert(strcmp( ...
    dominatedFindings.classification.status, ...
    'diagnostic-oracle-dominated'));
assert(dominatedFindings.consensusOracleVsDiscrepancy.trackingWinCount == 0);
assert(dominatedFindings.truthOracleVsDiscrepancy.consensusWinCount == 0);
fprintf('test_dynamic_topology_screen_analysis passed\n');
end

function record = makeRecord()
record = struct( ...
    'armMode', '', ...
    'armName', '', ...
    'focusWindowName', '', ...
    'focusWindow', [NaN, NaN], ...
    'focusEospa', NaN, ...
    'focusPosteriorConsensus', NaN, ...
    'attemptedBytes', NaN, ...
    'topologyInfeasibleRate', NaN, ...
    'topologyChurnRate', NaN, ...
    'distinctCandidateCount', NaN);
end

function cleanupFiles(matPath, reportPath)
if exist(matPath, 'file')
    delete(matPath);
end
if exist(reportPath, 'file')
    delete(reportPath);
end
end
