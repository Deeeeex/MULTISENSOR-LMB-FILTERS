function run_paper_mc50_batch()
%RUN_PAPER_MC50_BATCH Rerun paper Monte Carlo experiments with N=50.

projectRoot = '/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS';
runDir = '/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137';
manifestPath = fullfile(runDir, 'manifest.txt');

if ~exist(runDir, 'dir')
    mkdir(runDir);
end

cd(projectRoot);
addpath(projectRoot);
addpath(fullfile(projectRoot, 'RUN', 'GA'));
setPath;

writeManifestHeader(manifestPath, projectRoot, runDir);

runExperiment(runDir, manifestPath, '01_tiered_main_fidfia_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        50, 1, true, struct(), true, 'fidFiaExistenceRefinement', struct(), []));

runExperiment(runDir, manifestPath, '02_tiered_factor_ablation_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        50, 1, true, struct(), true, 'structureAware', struct(), []));

runExperiment(runDir, manifestPath, '03_tiered_pd_fi_baselines_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_FiWeightedGaCompare( ...
        50, 1, true, true, struct(), struct()));

runExperiment(runDir, manifestPath, '04_ideal_fidfia_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        50, 1, true, idealCommOverrides(), true, ...
        'fidFiaExistenceRefinement', struct(), []));

runExperiment(runDir, manifestPath, '05_comm_level0_balanced_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        50, 1, true, commLevelOverrides(0), true, ...
        'structureAware', struct(), [1 5]));

runExperiment(runDir, manifestPath, '06_comm_level1_balanced_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        50, 1, true, commLevelOverrides(1), true, ...
        'structureAware', struct(), [1 5]));

runExperiment(runDir, manifestPath, '07_comm_level2_balanced_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        50, 1, true, commLevelOverrides(2), true, ...
        'structureAware', struct(), [1 5]));

runExperiment(runDir, manifestPath, '08_comm_level3_balanced_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        50, 1, true, commLevelOverrides(3), true, ...
        'structureAware', struct(), [1 5]));

% Appendix/secondary probes are queued after the main-body experiments so
% the core tables become available first.
runExperiment(runDir, manifestPath, '09_appendix_nis_compare_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkNISCompare( ...
        50, 1, true, struct(), struct(), true));

runExperiment(runDir, manifestPath, '10_appendix_freshness_compare_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkFreshnessCompare( ...
        50, 1, true, struct(), struct(), true));

runExperiment(runDir, manifestPath, '11_appendix_cardinality_consensus_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        50, 1, true, struct(), true, 'cardinality', struct(), []));

runExperiment(runDir, manifestPath, '12_appendix_association_ambiguity_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_AssociationAmbiguityCompare( ...
        50, 1, true, true, struct()));

runExperiment(runDir, manifestPath, '13_appendix_posterior_structure_n50_seed1', ...
    @() runMultisensorFilters_formation_4plus4_PosteriorStructureCompare( ...
        50, 1, true, true, struct()));

fid = fopen(fullfile(runDir, 'DONE'), 'w');
if fid >= 0
    fprintf(fid, 'completed_at=%s\n', datestr(now, 31));
    fclose(fid);
end
appendManifest(manifestPath, sprintf('ALL_DONE\t%s', datestr(now, 31)));
end

function runExperiment(runDir, manifestPath, label, runner)
fprintf('\n===== START %s at %s =====\n', label, datestr(now, 31));
appendManifest(manifestPath, sprintf('START\t%s\t%s', label, datestr(now, 31)));
try
    [reportPath, summary] = runner();
    matPath = fullfile(runDir, [label '.mat']);
    save(matPath, 'reportPath', 'summary');
    fprintf('===== DONE %s at %s =====\n', label, datestr(now, 31));
    fprintf('Report: %s\n', reportPath);
    fprintf('Summary MAT: %s\n', matPath);
    appendManifest(manifestPath, sprintf('DONE\t%s\t%s\treport=%s\tmat=%s', ...
        label, datestr(now, 31), reportPath, matPath));
catch err
    fprintf(2, '===== ERROR %s at %s =====\n', label, datestr(now, 31));
    fprintf(2, '%s\n', err.message);
    for k = 1:numel(err.stack)
        fprintf(2, '  at %s:%d\n', err.stack(k).file, err.stack(k).line);
    end
    appendManifest(manifestPath, sprintf('ERROR\t%s\t%s\tmessage=%s', ...
        label, datestr(now, 31), err.message));
end
end

function commConfig = idealCommOverrides()
commConfig = struct();
commConfig.level = 0;
commConfig.globalMaxMeasurementsPerStep = inf;
commConfig.pDrop = 0.0;
commConfig.pDropBySensor = zeros(1, 8);
commConfig.pDropLevels = [];
commConfig.pDropLevelCounts = [];
commConfig.maxOutageNodes = 0;
end

function commConfig = commLevelOverrides(level)
commConfig = struct();
commConfig.level = level;
switch level
    case 0
        commConfig.globalMaxMeasurementsPerStep = inf;
        commConfig.pDrop = 0.0;
        commConfig.pDropBySensor = zeros(1, 8);
        commConfig.pDropLevels = [];
        commConfig.pDropLevelCounts = [];
        commConfig.maxOutageNodes = 0;
    case 1
        commConfig.globalMaxMeasurementsPerStep = 80;
        commConfig.pDrop = 0.0;
        commConfig.pDropBySensor = zeros(1, 8);
        commConfig.pDropLevels = [];
        commConfig.pDropLevelCounts = [];
        commConfig.maxOutageNodes = 0;
    case 2
        commConfig.globalMaxMeasurementsPerStep = 80;
        commConfig.pDrop = 0.2;
        commConfig.pDropLevels = [0, 0.1, 0.2, 0.5];
        commConfig.pDropLevelCounts = [1, 4, 1, 2];
        commConfig.maxOutageNodes = 0;
    case 3
        commConfig.globalMaxMeasurementsPerStep = 80;
        commConfig.pDrop = 0.2;
        commConfig.pDropLevels = [0, 0.1, 0.2, 0.5];
        commConfig.pDropLevelCounts = [1, 4, 1, 2];
        commConfig.maxOutageNodes = 1;
end
end

function writeManifestHeader(manifestPath, projectRoot, runDir)
fid = fopen(manifestPath, 'w');
if fid < 0
    error('Unable to write manifest: %s', manifestPath);
end
fprintf(fid, 'paper_mc50_batch\n');
fprintf(fid, 'created_at=%s\n', datestr(now, 31));
fprintf(fid, 'projectRoot=%s\n', projectRoot);
fprintf(fid, 'runDir=%s\n', runDir);
fprintf(fid, 'trials=50\n');
fprintf(fid, 'baseSeed=1\n');
fprintf(fid, 'trialSeeds=2:51\n');
fprintf(fid, '\n');
fclose(fid);
end

function appendManifest(manifestPath, line)
fid = fopen(manifestPath, 'a');
if fid < 0
    warning('Unable to append manifest: %s', manifestPath);
    return;
end
fprintf(fid, '%s\n', line);
fclose(fid);
end
