function audit = validatePdWeightedAgainstLegacy(summary, legacyReportPath)
% VALIDATEPDWEIGHTEDAGAINSTLEGACY Check the split PD arm seed by seed.
% The canonical direct-weighting report predates GOSPA.  A resumable rerun
% may add GOSPA, but its OSPA, matched localization, and cardinality values
% must reproduce the saved paired rows at their six-decimal precision.

if nargin < 2 || isempty(legacyReportPath)
    scriptDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(fileparts(scriptDir));
    legacyReportPath = fullfile(projectRoot, 'RUN', 'GA', ...
        'Del_GA_TIERED_LINK_ABLATION_N50_SEED1_20260528_092545.md');
end
assert(exist(legacyReportPath, 'file') == 2, ...
    'Legacy PD-weighted report not found: %s', legacyReportPath);
assert(isequal(summary.armNames, {'PD-weighted GA'}), ...
    'Unexpected PD-weighted arm identity.');
assert(size(summary.consensusTrials.ospa, 1) == numel(summary.trialSeeds), ...
    'PD-weighted trial count does not match trialSeeds.');

legacy = parseLegacyRows(legacyReportPath);
tolerance = 5.01e-7;
maxOspaDelta = 0;
maxLocalizationDelta = 0;
maxCardinalityDelta = 0;

for trialIdx = 1:numel(summary.trialSeeds)
    seed = summary.trialSeeds(trialIdx);
    legacyTrialIdx = find(legacy.seeds == seed, 1, 'first');
    assert(~isempty(legacyTrialIdx), ...
        'Seed %d is absent from the legacy PD-weighted report.', seed);
    maxOspaDelta = max(maxOspaDelta, abs( ...
        summary.consensusTrials.ospa(trialIdx, 1) - legacy.ospa(legacyTrialIdx)));
    maxLocalizationDelta = max(maxLocalizationDelta, abs( ...
        summary.consensusTrials.pos(trialIdx, 1) - legacy.localization(legacyTrialIdx)));
    maxCardinalityDelta = max(maxCardinalityDelta, abs( ...
        summary.consensusTrials.card(trialIdx, 1) - legacy.cardinality(legacyTrialIdx)));
end

assert(maxOspaDelta <= tolerance, ...
    'Per-seed PD-weighted OSPA differs from the legacy report (max %.12g).', ...
    maxOspaDelta);
assert(maxLocalizationDelta <= tolerance, ...
    ['Per-seed PD-weighted localization differs from the legacy report ' ...
    '(max %.12g).'], maxLocalizationDelta);
assert(maxCardinalityDelta <= tolerance, ...
    ['Per-seed PD-weighted cardinality differs from the legacy report ' ...
    '(max %.12g).'], maxCardinalityDelta);

localAggregateChecked = isequal(reshape(summary.trialSeeds, 1, []), 2:51);
maxLocalAggregateDelta = NaN;
if localAggregateChecked
    assert(isfield(summary, 'localTrials'), ...
        'PD-weighted full summary is missing local tracking metrics.');
    actualLocal = [ ...
        mean(summary.localTrials.eOspa(:)), ...
        mean(summary.localTrials.rmse(:), 'omitnan'), ...
        mean(summary.localTrials.cardErr(:))];
    expectedLocal = [2.735723, 1.562807, 1.254925];
    maxLocalAggregateDelta = max(abs(actualLocal - expectedLocal));
    assert(maxLocalAggregateDelta <= tolerance, ...
        ['PD-weighted local metrics differ from the canonical N=50 report ' ...
        '(max %.12g).'], maxLocalAggregateDelta);
end

audit.legacyReportPath = legacyReportPath;
audit.checkedSeeds = reshape(summary.trialSeeds, 1, []);
audit.tolerance = tolerance;
audit.maxOspaDelta = maxOspaDelta;
audit.maxLocalizationDelta = maxLocalizationDelta;
audit.maxCardinalityDelta = maxCardinalityDelta;
audit.localAggregateChecked = localAggregateChecked;
audit.maxLocalAggregateDelta = maxLocalAggregateDelta;
end

function legacy = parseLegacyRows(path)
content = fileread(path);
heading = '## Per-Trial Network Disagreement Metrics';
startIndices = strfind(content, heading);
assert(numel(startIndices) == 1, ...
    'Unable to isolate the legacy PD-weighted per-trial table.');
sectionStart = startIndices(1) + numel(heading);
remaining = content(sectionStart:end);
nextHeading = strfind(remaining, sprintf('\n## '));
assert(~isempty(nextHeading), ...
    'Legacy PD-weighted per-trial table has no closing heading.');
section = remaining(1:nextHeading(1) - 1);

pattern = ['\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*PD-weighted GA\s*\|\s*' ...
    '([-+0-9.eE]+)\s*\|\s*([-+0-9.eE]+)\s*\|\s*' ...
    '([-+0-9.eE]+)\s*\|'];
tokens = regexp(section, pattern, 'tokens');
assert(numel(tokens) == 50, ...
    'Expected 50 legacy PD-weighted rows, found %d.', numel(tokens));

legacy.seeds = zeros(1, 50);
legacy.ospa = zeros(1, 50);
legacy.localization = zeros(1, 50);
legacy.cardinality = zeros(1, 50);
seen = false(1, 50);
for rowIdx = 1:numel(tokens)
    row = tokens{rowIdx};
    trialIdx = str2double(row{1});
    assert(trialIdx >= 1 && trialIdx <= 50, ...
        'Invalid legacy PD-weighted trial index.');
    assert(~seen(trialIdx), 'Duplicate legacy PD-weighted trial row.');
    legacy.seeds(trialIdx) = str2double(row{2});
    legacy.ospa(trialIdx) = str2double(row{3});
    legacy.localization(trialIdx) = str2double(row{4});
    legacy.cardinality(trialIdx) = str2double(row{5});
    seen(trialIdx) = true;
end
assert(all(seen), 'Legacy PD-weighted per-trial table is incomplete.');
end
