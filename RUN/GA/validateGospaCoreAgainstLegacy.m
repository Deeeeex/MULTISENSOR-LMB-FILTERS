function audit = validateGospaCoreAgainstLegacy(summary, legacyReportPath)
% VALIDATEGOSPACOREAGAINSTLEGACY Check split-run pairing seed by seed.
% The legacy report contains the paper's pre-GOSPA OSPA, matched
% localization, and cardinality values.  A resumable run may add GOSPA but
% must reproduce every legacy value at the report's six-decimal precision.

if nargin < 2 || isempty(legacyReportPath)
    scriptDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(fileparts(scriptDir));
    legacyReportPath = fullfile(projectRoot, 'RUN', 'GA', ...
        'GA_TIERED_LINK_ABLATION_N50_SEED1_20260528_001743.md');
end
assert(exist(legacyReportPath, 'file') == 2, ...
    'Legacy paired report not found: %s', legacyReportPath);

expectedArmNames = {'fixed weights', 'Cao-Zhao FID-FIA baseline', ...
    '+structure-aware decoupled KLA', '+FID-FIA existence refinement'};
assert(isequal(summary.armNames, expectedArmNames), ...
    'Unexpected core arm order.');
assert(size(summary.consensusTrials.ospa, 1) == numel(summary.trialSeeds), ...
    'OSPA trial count does not match trialSeeds.');

legacy = parseLegacyRows(legacyReportPath, expectedArmNames);
tolerance = 5.01e-7;
maxOspaDelta = 0;
maxLocalizationDelta = 0;
maxCardinalityDelta = 0;

for trialIdx = 1:numel(summary.trialSeeds)
    seed = summary.trialSeeds(trialIdx);
    legacyTrialIdx = find(legacy.seeds == seed, 1, 'first');
    assert(~isempty(legacyTrialIdx), 'Seed %d is absent from the legacy report.', seed);
    ospaDelta = abs(summary.consensusTrials.ospa(trialIdx, :) - ...
        legacy.ospa(legacyTrialIdx, :));
    localizationDelta = abs(summary.consensusTrials.pos(trialIdx, :) - ...
        legacy.localization(legacyTrialIdx, :));
    cardinalityDelta = abs(summary.consensusTrials.card(trialIdx, :) - ...
        legacy.cardinality(legacyTrialIdx, :));
    maxOspaDelta = max(maxOspaDelta, max(ospaDelta));
    maxLocalizationDelta = max(maxLocalizationDelta, max(localizationDelta));
    maxCardinalityDelta = max(maxCardinalityDelta, max(cardinalityDelta));
end

assert(maxOspaDelta <= tolerance, ...
    'Per-seed OSPA differs from the legacy report (max %.12g).', maxOspaDelta);
assert(maxLocalizationDelta <= tolerance, ...
    'Per-seed localization differs from the legacy report (max %.12g).', ...
    maxLocalizationDelta);
assert(maxCardinalityDelta <= tolerance, ...
    'Per-seed cardinality differs from the legacy report (max %.12g).', ...
    maxCardinalityDelta);

localAggregateChecked = isequal(reshape(summary.trialSeeds, 1, []), 2:51);
maxLocalAggregateDelta = NaN;
if localAggregateChecked
    assert(isfield(summary, 'localTrials'), ...
        'Full core summary is missing local tracking metrics.');
    actualLocal = zeros(3, numel(expectedArmNames));
    for armIdx = 1:numel(expectedArmNames)
        eOspa = summary.localTrials.eOspa(:, :, armIdx);
        rmse = summary.localTrials.rmse(:, :, armIdx);
        cardErr = summary.localTrials.cardErr(:, :, armIdx);
        actualLocal(:, armIdx) = [ ...
            mean(eOspa(:)); mean(rmse(:), 'omitnan'); mean(cardErr(:))];
    end
    expectedLocal = [ ...
        2.862938, 2.184698, 2.334915, 2.019842; ...
        1.649569, 1.734381, 1.605910, 1.720931; ...
        1.455125, 0.388050, 0.578775, 0.223700];
    maxLocalAggregateDelta = max(abs(actualLocal(:) - expectedLocal(:)));
    assert(maxLocalAggregateDelta <= tolerance, ...
        ['Core local metrics differ from the canonical N=50 report ' ...
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

function legacy = parseLegacyRows(path, expectedArmNames)
content = fileread(path);
heading = '## Per-Trial Network Disagreement Metrics';
startIndices = strfind(content, heading);
assert(numel(startIndices) == 1, 'Unable to isolate the legacy per-trial table.');
sectionStart = startIndices(1) + numel(heading);
remaining = content(sectionStart:end);
nextHeading = strfind(remaining, sprintf('\n## '));
assert(~isempty(nextHeading), 'Legacy per-trial table has no closing heading.');
section = remaining(1:nextHeading(1) - 1);

pattern = ['\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*([^|]+?)\s*\|\s*' ...
    '([-+0-9.eE]+)\s*\|\s*([-+0-9.eE]+)\s*\|\s*' ...
    '([-+0-9.eE]+)\s*\|'];
tokens = regexp(section, pattern, 'tokens');
assert(numel(tokens) == 50 * numel(expectedArmNames), ...
    'Expected 200 legacy per-trial rows, found %d.', numel(tokens));

legacy.seeds = zeros(1, 50);
legacy.ospa = zeros(50, numel(expectedArmNames));
legacy.localization = zeros(50, numel(expectedArmNames));
legacy.cardinality = zeros(50, numel(expectedArmNames));
seen = false(50, numel(expectedArmNames));
for rowIdx = 1:numel(tokens)
    row = tokens{rowIdx};
    trialIdx = str2double(row{1});
    seed = str2double(row{2});
    armName = strtrim(row{3});
    armIdx = find(strcmp(expectedArmNames, armName), 1, 'first');
    assert(~isempty(armIdx), 'Unexpected arm in legacy report: %s', armName);
    assert(trialIdx >= 1 && trialIdx <= 50, 'Invalid legacy trial index.');
    assert(~seen(trialIdx, armIdx), 'Duplicate legacy trial/arm row.');
    if legacy.seeds(trialIdx) == 0
        legacy.seeds(trialIdx) = seed;
    else
        assert(legacy.seeds(trialIdx) == seed, 'Inconsistent seed within legacy trial.');
    end
    legacy.ospa(trialIdx, armIdx) = str2double(row{4});
    legacy.localization(trialIdx, armIdx) = str2double(row{5});
    legacy.cardinality(trialIdx, armIdx) = str2double(row{6});
    seen(trialIdx, armIdx) = true;
end
assert(all(seen(:)), 'Legacy per-trial table is incomplete.');
end
