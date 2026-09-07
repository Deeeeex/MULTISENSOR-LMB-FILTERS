function buildGospaMainTable(corePath, pdPath, outputDir, enforceLegacyValues, paperRowsPath)
% BUILDGOSPAMAINTABLE Combine the four-arm core and PD-weighted GOSPA runs.
% The function refuses to write a paper-facing table unless paired seeds,
% communication realizations, arm identities, and legacy OSPA values agree.
% PAPERROWSPATH is optional so synthetic tests cannot overwrite manuscript
% fragments; the formal validation pipeline supplies it explicitly.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(scriptDir));
if nargin < 3 || isempty(outputDir)
    outputDir = fullfile(projectRoot, 'RUN', 'GA', 'gospa_validation');
end
if nargin < 1 || isempty(corePath)
    corePath = fullfile(outputDir, 'gospa_core_n50_summary.mat');
end
if nargin < 2 || isempty(pdPath)
    pdPath = fullfile(outputDir, 'gospa_pd_weighted_n50_summary.mat');
end
if nargin < 4 || isempty(enforceLegacyValues)
    enforceLegacyValues = true;
end
if nargin < 5
    paperRowsPath = '';
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

coreData = load(corePath, 'summary');
pdData = load(pdPath, 'summary');
core = coreData.summary;
pdArm = pdData.summary;

expectedCore = {'fixed weights', 'Cao-Zhao FID-FIA baseline', ...
    '+structure-aware decoupled KLA', '+FID-FIA existence refinement'};
expectedPd = {'PD-weighted GA'};
assert(isequal(core.armNames, expectedCore), 'Unexpected core arm order.');
assert(isequal(pdArm.armNames, expectedPd), 'Unexpected PD arm selection.');
assert(isequal(core.trialSeeds, pdArm.trialSeeds), 'Trial seeds do not match.');
assert(isfield(core, 'commConfig') && isfield(pdArm, 'commConfig') && ...
    isequaln(core.commConfig, pdArm.commConfig), ...
    'Core and PD communication configurations do not match.');
validateArmConfigurations(core.arms, pdArm.arms);
assert(max(abs(core.pDropBySensorTrials(:) - pdArm.pDropBySensorTrials(:))) < 1e-12, ...
    'Communication realizations do not match.');

combined.armNames = {'Fixed Metropolis', 'PD-weighted GA', ...
    'FID-FIA-weighted GA', 'Balanced mode', 'Cardinality-critical mode'};
combined.trialSeeds = core.trialSeeds;
combined.pDropBySensorTrials = core.pDropBySensorTrials;
combined.consensusTrials.ospa = joinMetric(core.consensusTrials.ospa, ...
    pdArm.consensusTrials.ospa);
combined.consensusTrials.gospa = joinMetric(core.consensusTrials.gospa, ...
    pdArm.consensusTrials.gospa);
combined.consensusTrials.pos = joinMetric(core.consensusTrials.pos, ...
    pdArm.consensusTrials.pos);
combined.consensusTrials.card = joinMetric(core.consensusTrials.card, ...
    pdArm.consensusTrials.card);
combined.runtime.filterSeconds = joinMetric(core.runtime.filterSeconds, ...
    pdArm.runtime.filterSeconds);

if enforceLegacyValues
    validateLegacyValues(combined);
end

combinedPath = fullfile(outputDir, 'gospa_main_table_n50_combined.mat');
summaryCsvPath = fullfile(outputDir, 'gospa_main_table_n50.csv');
trialCsvPath = fullfile(outputDir, 'gospa_main_table_n50_per_trial.csv');
reportPath = fullfile(outputDir, 'gospa_main_table_n50.md');
latexRowsPath = fullfile(outputDir, 'gospa_main_table_n50_rows.tex');
save('-mat7-binary', combinedPath, 'combined');
writeSummaryCsv(summaryCsvPath, combined);
writeTrialCsv(trialCsvPath, combined);
writeReport(reportPath, combined, corePath, pdPath);
writeLatexRows(latexRowsPath, combined);
if ~isempty(paperRowsPath)
    ensureParentDirectory(paperRowsPath);
    writeLatexRows(paperRowsPath, combined);
end

fprintf('GOSPA_MAIN_TABLE_COMPLETE\n');
fprintf('Combined: %s\n', combinedPath);
fprintf('Summary CSV: %s\n', summaryCsvPath);
fprintf('Per-trial CSV: %s\n', trialCsvPath);
fprintf('Report: %s\n', reportPath);
fprintf('LaTeX rows: %s\n', latexRowsPath);
if ~isempty(paperRowsPath)
    fprintf('Paper LaTeX rows: %s\n', paperRowsPath);
end
end

function ensureParentDirectory(path)
parentDir = fileparts(path);
if ~isempty(parentDir) && ~exist(parentDir, 'dir')
    mkdir(parentDir);
end
end

function validateArmConfigurations(coreArms, pdArms)
assert(numel(coreArms) == 4 && numel(pdArms) == 1, ...
    'Unexpected number of saved arm configurations.');
assert(~coreArms(1).adaptiveFusion.enabled, ...
    'Fixed Metropolis configuration is not fixed.');
assert(strcmpi(coreArms(2).adaptiveFusion.method, 'fidFia') && ...
    ~coreArms(2).adaptiveFusion.fidFiaUseEma && ...
    coreArms(2).adaptiveFusion.fidFiaMinWeight == 0, ...
    'Unexpected FID-FIA baseline configuration.');
balanced = coreArms(3).adaptiveFusion;
assert(strcmpi(balanced.method, 'factorized') && balanced.useDecoupledKla && ...
    balanced.useStructureAwareKla && balanced.useCovariance && ...
    balanced.useLinkQuality && balanced.useExistenceConfidence && ...
    ~balanced.useFidFiaExistence, ...
    'Unexpected Balanced-mode configuration.');
cardinality = coreArms(4).adaptiveFusion;
assert(cardinality.useFidFiaExistence && ...
    cardinality.fidFiaExistenceStrength == 4 && ...
    cardinality.fidFiaExistenceMinScore == 0 && ...
    cardinality.existenceEmaAlpha == 0 && ...
    cardinality.existenceMinWeight == 0, ...
    'Unexpected Cardinality-critical configuration.');
assert(strcmpi(pdArms(1).adaptiveFusion.method, 'pdWeightedGa') && ...
    pdArms(1).adaptiveFusion.enabled, ...
    'Unexpected PD-weighted configuration.');
end

function joined = joinMetric(coreValues, pdValues)
assert(size(coreValues, 1) == size(pdValues, 1), 'Trial counts do not match.');
assert(size(coreValues, 2) == 4, 'Core metric must contain four arms.');
assert(size(pdValues, 2) == 1, 'PD metric must contain one arm.');
joined = [coreValues(:, 1), pdValues(:, 1), coreValues(:, 2:4)];
end

function validateLegacyValues(combined)
legacyOspa = [2.469, 2.177, 1.818, 1.779, 1.678];
legacyPos = [2.326, 1.995, 1.643, 1.522, 1.546];
legacyCard = [0.716, 0.588, 0.123, 0.188, 0.063];
assert(max(abs(mean(combined.consensusTrials.ospa, 1) - legacyOspa)) < 5e-4, ...
    'OSPA means do not reproduce the manuscript values at three decimals.');
assert(max(abs(mean(combined.consensusTrials.pos, 1, 'omitnan') - legacyPos)) < 5e-4, ...
    'Localization means do not reproduce the manuscript values at three decimals.');
assert(max(abs(mean(combined.consensusTrials.card, 1) - legacyCard)) < 5e-4, ...
    'Cardinality means do not reproduce the manuscript values at three decimals.');
end

function writeSummaryCsv(path, combined)
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, ['arm,ospa_mean,ospa_std,gospa_mean,gospa_std,' ...
    'localization_mean,localization_std,cardinality_mean,cardinality_std,' ...
    'gospa_improved_vs_fixed,gospa_sign_test_p\n']);
for armIdx = 1:numel(combined.armNames)
    ospa = combined.consensusTrials.ospa(:, armIdx);
    gospa = combined.consensusTrials.gospa(:, armIdx);
    pos = combined.consensusTrials.pos(:, armIdx);
    card = combined.consensusTrials.card(:, armIdx);
    if armIdx == 1
        improved = 0;
        signP = 1;
    else
        delta = gospa - combined.consensusTrials.gospa(:, 1);
        improved = sum(delta < 0);
        signP = exactTwoSidedSignP(delta);
    end
    fprintf(fid, '%s,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%.12g\n', ...
        combined.armNames{armIdx}, mean(ospa), std(ospa), mean(gospa), std(gospa), ...
        mean(pos, 'omitnan'), std(pos), mean(card), std(card), improved, signP);
end
end

function writeTrialCsv(path, combined)
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, 'trial,seed,arm,ospa,gospa,localization,cardinality\n');
for trialIdx = 1:numel(combined.trialSeeds)
    for armIdx = 1:numel(combined.armNames)
        fprintf(fid, '%d,%d,%s,%.9f,%.9f,%.9f,%.9f\n', ...
            trialIdx, combined.trialSeeds(trialIdx), combined.armNames{armIdx}, ...
            combined.consensusTrials.ospa(trialIdx, armIdx), ...
            combined.consensusTrials.gospa(trialIdx, armIdx), ...
            combined.consensusTrials.pos(trialIdx, armIdx), ...
            combined.consensusTrials.card(trialIdx, armIdx));
    end
end
end

function writeReport(path, combined, corePath, pdPath)
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, '# Main-scenario GOSPA validation\n\n');
fprintf(fid, '- Seeds: 2--51 (50 deterministic paired trials)\n');
fprintf(fid, '- OSPA/GOSPA: c=5, p=2; GOSPA alpha=2\n');
fprintf(fid, '- Ground space: complete extracted kinematic state vector\n');
fprintf(fid, '- Core source: `%s`\n', corePath);
fprintf(fid, '- PD source: `%s`\n\n', pdPath);
fprintf(fid, '| Arm | OSPA | GOSPA | Loc. disag. | Card. disp. | GOSPA improved | Sign-test p |\n');
fprintf(fid, '|:----|-----:|------:|------------:|------------:|---------------:|------------:|\n');
for armIdx = 1:numel(combined.armNames)
    ospa = combined.consensusTrials.ospa(:, armIdx);
    gospa = combined.consensusTrials.gospa(:, armIdx);
    pos = combined.consensusTrials.pos(:, armIdx);
    card = combined.consensusTrials.card(:, armIdx);
    if armIdx == 1
        improved = 0;
        signP = 1;
    else
        delta = gospa - combined.consensusTrials.gospa(:, 1);
        improved = sum(delta < 0);
        signP = exactTwoSidedSignP(delta);
    end
    fprintf(fid, '| %s | %.3f +/- %.3f | %.3f +/- %.3f | %.3f +/- %.3f | %.3f +/- %.3f | %d/50 | %.3g |\n', ...
        combined.armNames{armIdx}, mean(ospa), std(ospa), mean(gospa), std(gospa), ...
        mean(pos, 'omitnan'), std(pos), mean(card), std(card), improved, signP);
end
end

function writeLatexRows(path, combined)
% Emit the paper-facing values from the same validated aggregate used by
% the CSV and Markdown reports.  Keeping this as a row fragment prevents a
% second hand-transcription path while leaving table layout in the paper.
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, '%% AUTO-GENERATED by buildGospaMainTable.m; do not edit.\n');
fprintf(fid, '%% Seeds 2--51; values are mean \\pm standard deviation.\n');
fprintf(fid, '\\newcommand{\\GospaMainTableRows}{%%\n');
metricMeans = [ ...
    mean(combined.consensusTrials.ospa, 1); ...
    mean(combined.consensusTrials.gospa, 1); ...
    mean(combined.consensusTrials.pos, 1, 'omitnan'); ...
    mean(combined.consensusTrials.card, 1)];
bestMeans = min(metricMeans, [], 2);
for armIdx = 1:numel(combined.armNames)
    ospa = combined.consensusTrials.ospa(:, armIdx);
    gospa = combined.consensusTrials.gospa(:, armIdx);
    pos = combined.consensusTrials.pos(:, armIdx);
    card = combined.consensusTrials.card(:, armIdx);
    values = {ospa, gospa, pos, card};
    cells = cell(1, numel(values));
    for metricIdx = 1:numel(values)
        cells{metricIdx} = formatLatexMetric(values{metricIdx}, ...
            abs(metricMeans(metricIdx, armIdx) - bestMeans(metricIdx)) < 1e-12);
    end
    fprintf(fid, '%s & %s & %s & %s & %s \\\\\n', ...
        combined.armNames{armIdx}, cells{1}, cells{2}, cells{3}, cells{4});
end
fprintf(fid, '}\n');
end

function text = formatLatexMetric(values, isBest)
values = values(isfinite(values));
metricMean = mean(values);
metricStd = std(values);
if isBest
    text = sprintf('$\\mathbf{%.3f} \\pm %.3f$', metricMean, metricStd);
else
    text = sprintf('$%.3f \\pm %.3f$', metricMean, metricStd);
end
end

function pValue = exactTwoSidedSignP(delta)
delta = delta(isfinite(delta) & delta ~= 0);
n = numel(delta);
if n == 0
    pValue = 1;
    return;
end
k = min(sum(delta < 0), sum(delta > 0));
tail = 0;
for idx = 0:k
    tail = tail + nchoosek(n, idx) * 0.5^n;
end
pValue = min(1, 2 * tail);
end
