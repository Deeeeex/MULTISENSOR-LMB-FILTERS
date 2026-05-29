function synthesize_paper_tables_n50()
%SYNTHESIZE_PAPER_TABLES_N50 Build paper-facing N=50 tables from batch outputs.

runDir = '/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137';
outPath = fullfile(runDir, 'paper_tables_n50_synthesis.md');

fid = fopen(outPath, 'w');
if fid < 0
    error('Unable to write %s', outPath);
end
cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, '# Paper-facing N=50 synthesis\n\n');
fprintf(fid, 'Generated at: %s\n\n', datestr(now, 31));
fprintf(fid, 'This file reorders the batch outputs into the exact paper-table logic. ');
fprintf(fid, 'It does not rerun simulations.\n\n');

main = loadSummary(runDir, '01_tiered_main_fidfia_n50_seed1.mat');
ablation = loadSummary(runDir, '02_tiered_factor_ablation_n50_seed1.mat');
pdFi = loadSummary(runDir, '03_tiered_pd_fi_baselines_n50_seed1.mat');
ideal = loadSummary(runDir, '04_ideal_fidfia_n50_seed1.mat');

fprintf(fid, '## Source mapping\n\n');
fprintf(fid, '| Paper table | Source summary | Rows used |\n');
fprintf(fid, '|:--|:--|:--|\n');
fprintf(fid, '| Main consensus/local | 01 + 03 | Fixed/FID/Balanced/Cardinality from 01; PD/FI from 03 |\n');
fprintf(fid, '| Runtime cost | 01, optional 03 | Fixed/FID/Balanced/Cardinality from 01; PD/FI optional from 03 |\n');
fprintf(fid, '| Factor ablation | 02 + 01 | First five backbone rows from 02; Cardinality-critical from 01 |\n');
fprintf(fid, '| Ideal-support | 04 | Reordered as Ordinary, Balanced, FID-FIA, Cardinality-critical |\n');
fprintf(fid, '| Communication robustness | 05-08 | Fixed/Balanced pair per communication level |\n\n');

writeMainTables(fid, main, pdFi);
writeAblationTable(fid, ablation, main);
writeIdealTable(fid, ideal);
writeCommunicationTables(fid, runDir);

fprintf('Paper-facing synthesis written: %s\n', outPath);
end

function data = loadSummary(runDir, fileName)
path = fullfile(runDir, fileName);
data = struct('available', false, 'path', path, 'summary', [], 'reportPath', '');
if ~exist(path, 'file')
    return;
end
loaded = load(path);
data.available = true;
data.summary = loaded.summary;
if isfield(loaded, 'reportPath')
    data.reportPath = loaded.reportPath;
end
end

function writeMainTables(fid, main, pdFi)
fprintf(fid, '## Main tiered-drop table\n\n');
if ~main.available
    fprintf(fid, 'Missing 01_tiered_main_fidfia_n50_seed1.mat.\n\n');
    return;
end
if ~pdFi.available
    fprintf(fid, 'PD/FI summary is not available yet; rerun this synthesis after step 03 finishes.\n\n');
end

rows = {
    'Fixed Metropolis', main, 'fixed weights';
    'PD-weighted GA', pdFi, 'PD-weighted GA';
    'FI-weighted GA', pdFi, 'FI-weighted GA';
    'FID-FIA baseline', main, 'Cao-Zhao FID-FIA baseline';
    'Balanced mode', main, '+structure-aware decoupled KLA';
    'Cardinality-critical mode', main, '+FID-FIA existence refinement';
};
writeConsensusTable(fid, rows);
writeLocalTable(fid, rows);

fprintf(fid, '### Runtime rows used by the current paper\n\n');
runtimeRows = {
    'Fixed Metropolis', main, 'fixed weights';
    'FID-FIA baseline', main, 'Cao-Zhao FID-FIA baseline';
    'Balanced mode', main, '+structure-aware decoupled KLA';
    'Cardinality-critical mode', main, '+FID-FIA existence refinement';
};
writeRuntimeTable(fid, runtimeRows);

if pdFi.available
    fprintf(fid, '### Optional PD/FI runtime probe\n\n');
    pdFiRuntimeRows = {
        'PD-weighted GA', pdFi, 'PD-weighted GA';
        'FI-weighted GA', pdFi, 'FI-weighted GA';
    };
    writeRuntimeTable(fid, pdFiRuntimeRows);
end
end

function writeAblationTable(fid, ablation, main)
fprintf(fid, '## Factor ablation table\n\n');
if ~ablation.available || ~main.available
    fprintf(fid, 'Missing ablation or main summary.\n\n');
    return;
end
rows = {
    'Fixed Metropolis', ablation, 'fixed weights';
    'Covariance-only adaptive', ablation, '+covariance';
    'Covariance-link adaptive', ablation, '+link quality';
    'Three-factor adaptive backbone', ablation, '+existence confidence';
    'Balanced mode', ablation, '+structure-aware decoupled KLA';
    'Cardinality-critical mode', main, '+FID-FIA existence refinement';
};
writeConsensusTable(fid, rows);
end

function writeIdealTable(fid, ideal)
fprintf(fid, '## Ideal-communication support table\n\n');
if ~ideal.available
    fprintf(fid, 'Ideal-support summary is not available yet.\n\n');
    return;
end
rows = {
    'Ordinary GA', ideal, 'fixed weights';
    'Balanced mode', ideal, '+structure-aware decoupled KLA';
    'FID-FIA baseline', ideal, 'Cao-Zhao FID-FIA baseline';
    'Cardinality-critical mode', ideal, '+FID-FIA existence refinement';
};
writeConsensusTable(fid, rows);
writeLocalTable(fid, rows);
end

function writeCommunicationTables(fid, runDir)
fprintf(fid, '## Communication-level robustness table\n\n');
fprintf(fid, '| Level | Fixed OSPA | Balanced OSPA | Fixed loc. disag. | Balanced loc. disag. | Fixed card. disp. | Balanced card. disp. |\n');
fprintf(fid, '|--:|--:|--:|--:|--:|--:|--:|\n');
for level = 0:3
    labelNum = 5 + level;
    fileName = sprintf('%02d_comm_level%d_balanced_n50_seed1.mat', labelNum, level);
    data = loadSummary(runDir, fileName);
    if ~data.available
        fprintf(fid, '| %d | pending | pending | pending | pending | pending | pending |\n', level);
        continue;
    end
    fixed = getIndex(data.summary, 'fixed weights');
    balanced = getIndex(data.summary, '+structure-aware decoupled KLA');
    fprintf(fid, '| %d | %.6f | %.6f | %.6f | %.6f | %.6f | %.6f |\n', ...
        level, ...
        data.summary.consensus.ospa(fixed), data.summary.consensus.ospa(balanced), ...
        data.summary.consensus.pos(fixed), data.summary.consensus.pos(balanced), ...
        data.summary.consensus.card(fixed), data.summary.consensus.card(balanced));
end
fprintf(fid, '\n');
end

function writeConsensusTable(fid, rows)
fprintf(fid, '| Arm | OSPA err. | Loc. disag. | Card. disp. | OSPA std | Loc. std | Card. std | Source |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|:--|\n');
for r = 1:size(rows, 1)
    [ok, s, idx, source] = resolveRow(rows{r, 2}, rows{r, 3});
    if ~ok
        fprintf(fid, '| %s | pending | pending | pending | pending | pending | pending | %s |\n', rows{r, 1}, source);
        continue;
    end
    fprintf(fid, '| %s | %.6f | %.6f | %.6f | %.6f | %.6f | %.6f | %s |\n', ...
        rows{r, 1}, s.consensus.ospa(idx), s.consensus.pos(idx), s.consensus.card(idx), ...
        std(s.consensusTrials.ospa(:, idx)), std(s.consensusTrials.pos(:, idx)), ...
        std(s.consensusTrials.card(:, idx)), source);
end
fprintf(fid, '\n');
end

function writeLocalTable(fid, rows)
fprintf(fid, '| Arm | E-OSPA | RMSE | CardErr | Source |\n');
fprintf(fid, '|:--|--:|--:|--:|:--|\n');
for r = 1:size(rows, 1)
    [ok, s, idx, source] = resolveRow(rows{r, 2}, rows{r, 3});
    if ~ok
        fprintf(fid, '| %s | pending | pending | pending | %s |\n', rows{r, 1}, source);
        continue;
    end
    fprintf(fid, '| %s | %.6f | %.6f | %.6f | %s |\n', rows{r, 1}, ...
        s.local.meanAcrossSensors.eOspa(idx), s.local.meanAcrossSensors.rmse(idx), ...
        s.local.meanAcrossSensors.cardErr(idx), source);
end
fprintf(fid, '\n');
end

function writeRuntimeTable(fid, rows)
fprintf(fid, '| Arm | Runtime (s) | Runtime/step (s) | Relative runtime | Runtime std | Source |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|:--|\n');
baselineRuntime = NaN;
for r = 1:size(rows, 1)
    [ok, s, idx] = resolveRow(rows{r, 2}, rows{r, 3});
    if ok
        baselineRuntime = s.runtime.meanFilterSeconds(idx);
        break;
    end
end
for r = 1:size(rows, 1)
    [ok, s, idx, source] = resolveRow(rows{r, 2}, rows{r, 3});
    if ~ok
        fprintf(fid, '| %s | pending | pending | pending | pending | %s |\n', rows{r, 1}, source);
        continue;
    end
    rel = s.runtime.meanFilterSeconds(idx) / max(baselineRuntime, eps);
    fprintf(fid, '| %s | %.6f | %.6f | %.6f | %.6f | %s |\n', rows{r, 1}, ...
        s.runtime.meanFilterSeconds(idx), s.runtime.meanSecondsPerStep(idx), rel, ...
        s.runtime.stdFilterSeconds(idx), source);
end
fprintf(fid, '\n');
end

function [ok, summary, idx, source] = resolveRow(data, armName)
ok = false;
summary = [];
idx = NaN;
source = 'missing';
if ~data.available
    return;
end
summary = data.summary;
source = data.path;
idx = getIndex(summary, armName);
ok = ~isnan(idx);
end

function idx = getIndex(summary, armName)
idx = NaN;
for k = 1:numel(summary.armNames)
    if strcmp(summary.armNames{k}, armName)
        idx = k;
        return;
    end
end
end
