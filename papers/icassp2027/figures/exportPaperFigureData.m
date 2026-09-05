function exportPaperFigureData()
% Export existing episode summaries only. No filter or route is rerun.
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(fileparts(here)));
budgetPath = fullfile(root, 'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v279', 'set_error_budget_seed1301', 'SET_ERROR_BUDGET_V279.mat');
b = load(budgetPath, 'result'); budget = b.result;
selfPath = fullfile(root, 'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v278', 'x36_missing_packet_self_seed1301', ...
    'MISSING_PACKET_SELF_WEIGHT_V278.mat');
fid = fopen(fullfile(here, 'routing_tradeoff_source.csv'), 'w');
assert(fid >= 0); cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, ['scale,policy,seed,steps,eospa_m,conditional_rmse_m,', ...
    'focus_consistency_m,attempted_payload_bytes,mean_absolute_count_error,', ...
    'attempted_messages,source_commit,source_mat\n']);
keys = {'referenceFixedTree', 'referenceFullCausal', 'candidate'};
policies = {'fixed', 'full', 'sparse'};
scales = {'M24', 'X36'};
for s = 1:numel(budget.sourcePaths)
    a = load(budget.sourcePaths{s}, 'result'); r = a.result;
    for k = 1:numel(keys)
        writeArm(fid, scales{s}, policies{k}, r.seed, r.(keys{k}), ...
            r.generationGitCommit, budget.sourcePaths{s});
    end
end
a = load(selfPath, 'result'); r = a.result;
writeArm(fid, 'X36', 'self', r.seed, r.candidate, r.generationGitCommit, selfPath);

f = fopen(fullfile(here, 'count_budget_source.csv'), 'w');
assert(f >= 0); cleanup2 = onCleanup(@() fclose(f)); %#ok<NASGU>
fprintf(f, ['scale,policy,seed,mean_eospa_m,count_share_lower,count_share_upper,', ...
    'localization_only_headroom_m,ambiguous_cells,total_cells,source_commit\n']);
for s = 1:numel(budget.scenes)
    scene = budget.scenes{s};
    for k = 1:numel(scene.arms)
        arm = scene.arms{k};
        fprintf(f, '%s,%s,%d,%.12g,%.12g,%.12g,%.12g,%d,%d,%s\n', ...
            scales{s}, policies{k}, scene.seed, arm.meanEospa, ...
            arm.cardinalitySquaredErrorShareLower, arm.cardinalitySquaredErrorShareUpper, ...
            arm.maxEospaGainFromLocalizationOnly, arm.ambiguousCount, ...
            arm.sensorTimeCount, scene.sourceCommit);
    end
end
fprintf('Exported seven paired-arm rows and six count-budget rows; no filter rerun.\n');
end

function writeArm(fid, scale, policy, seed, a, commit, path)
fprintf(fid, '%s,%s,%d,%d,%.12g,%.12g,%.12g,%.0f,%.12g,%d,%s,%s\n', ...
    scale, policy, seed, size(a.positionEospaBySensorTime, 2), ...
    a.fullHorizonPositionEospa, a.fullHorizonPositionRmse, ...
    a.focusInterFormationPositionOspa, a.attemptedPayloadBytes, ...
    a.meanAbsoluteCardinalityError, a.attemptedMessageCount, commit, path);
end
