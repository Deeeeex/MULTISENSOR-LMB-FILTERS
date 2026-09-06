function exportPaperGuardrailData()
% Export tail and common-finite checks from completed paired summaries only.
here=fileparts(mfilename('fullpath'));
root=fileparts(fileparts(fileparts(here)));
s=load(fullfile(root,'RUN','GA','dynamic_topology','evidence', ...
    'tracking_aligned_v279','set_error_budget_seed1301','SET_ERROR_BUDGET_V279.mat'),'result');
budget=s.result;
fid=fopen(fullfile(here,'paired_guardrail_source.csv'),'w');assert(fid>=0);
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'scale,candidate,reference,common_finite_fraction,common_reference_rmse,common_candidate_rmse,common_rmse_gain_percent,minimum_formation_rmse_gain_percent,source_commit\n');
for k=1:2
    s=load(budget.sourcePaths{k},'result'); result=s.result;
    scales={'M24','X36'};
    writeRow(fid,scales{k},'sparse','fixed',result.candidate, ...
        result.referenceFixedTree,result.generationGitCommit);
end
s=load(fullfile(root,'RUN','GA','dynamic_topology','evidence', ...
    'tracking_aligned_v278','x36_missing_packet_self_seed1301', ...
    'MISSING_PACKET_SELF_WEIGHT_V278.mat'),'result');
writeRow(fid,'X36','self','sparse',s.result.candidate,s.result.reference, ...
    s.result.generationGitCommit);
fprintf('Exported three paired coverage/tail rows; no filter rerun.\n');
end

function writeRow(fid,scale,candidate,reference,a,b,commit)
x=a.positionRmseBySensorTime; y=b.positionRmseBySensorTime;
mask=isfinite(x)&isfinite(y);
gain=100*(1-mean(x(mask))/mean(y(mask)));
tail=min(100*(1-a.perFormationPositionRmse./b.perFormationPositionRmse));
fprintf(fid,'%s,%s,%s,%.12g,%.12g,%.12g,%.12g,%.12g,%s\n', ...
    scale,candidate,reference,mean(mask(:)),mean(y(mask)),mean(x(mask)),gain,tail,commit);
end
