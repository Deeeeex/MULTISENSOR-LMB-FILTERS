function [objects,stats]=fuseMarkedInputs(inputs,weights,model,details,cfg,arm,t)
% The local mark model is shared; only the named fusion rule differs.
name=erase(arm,'marked_');
assert(any(strcmp(name,{'lineage','er','ceiling_association','ceiling_score','ceiling_calibrated'})));
cfg.captureIterationRecords=true;
if any(strcmp(name,{'lineage','er'})),rule='qualified_exist';else,rule=name;end
[objects,stats]=fuseEvidenceCeiling(inputs,weights,model,details,cfg,rule,t);
if strcmp(name,'lineage')
    % The already computed no-age scalar uses the same spatial density and
    % eligibility as the exact original No-age rule. ER fields are retained
    % as same-input diagnostics, and are never used as this arm's posterior.
    assert(numel(objects)==size(stats.records,1));
    for j=1:numel(objects)
        assert(objects(j).birthTime==stats.records(j,3) && objects(j).birthLocation==stats.records(j,4));
        objects(j).r=stats.records(j,8);
    end
    stats.records(:,7)=stats.records(:,8);stats.records(:,10)=stats.records(:,8);
    stats.records(:,25)=0;stats.records(:,26)=stats.records(:,24);stats.weightChange=0;
end
end
