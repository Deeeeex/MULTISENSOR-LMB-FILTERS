function checkRecursionIntervention()
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.6;
o.numberOfGmComponents=1;o.w=1;o.mu={[1;0;0;0]};o.Sigma={diag([2,2,4,4])};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
o.localLogOddsIncrement=.7;o.positiveInnovationSupport=.8;o.negativeInnovationSupport=.3;
o.localSpatialLogRatio=gaussianLogRatio(zeros(4,1),diag([8,8,4,4]),o.mu{1},o.Sigma{1});
model.birthParameters=o;model.object=o([]);
cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',20);
p=o;p.mu={[-1;1;0;0]};p.r=.4;p.localLogOddsIncrement=-.3;
p.localSpatialLogRatio=gaussianLogRatio(zeros(4,1),diag([8,8,4,4]),p.mu{1},p.Sigma{1});
[gce,g]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
[gs,s]=fuseReviewerEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence_guarded_scalar',20);
assert(abs(gce.r-gs.r)>1e-5 && max(abs(gce.Sigma{1}-gs.Sigma{1}),[],'all')>1e-3);
for mode={'none','joint','existence','spatial'}
    [one,stats,records]=spliceRecursionIntervention(gs,s,gce,g,mode{1},20,1);
    useR=ismember(mode{1},{'joint','existence'});useSpace=ismember(mode{1},{'joint','spatial'});
    if useR,assert(one.r==gce.r);else,assert(one.r==gs.r);end
    if useSpace,assert(isequal(one.mu,gce.mu) && isequal(one.Sigma,gce.Sigma));
    else,assert(isequal(one.mu,gs.mu) && isequal(one.Sigma,gs.Sigma));end
    assert(one.r==stats.records(7) && one.r==stats.records(10));
    if strcmp(mode{1},'none'),assert(isempty(records) && isequaln(stats,s));
    else,assert(size(records,1)==1 && records(8)==one.r);end
end
[objects,stats,records]=spliceRecursionIntervention(gs([]),struct('records',zeros(0,60), ...
    'lineageExcluded',0,'observableAbsences',0,'weightChange',0),gce([]), ...
    struct('records',zeros(0,60),'lineageExcluded',0,'observableAbsences',0,'weightChange',0),'joint',20,1);
assert(isempty(objects) && isempty(stats.records) && isempty(records));
fprintf('RECURSION INTERVENTION CHECK PASSED: none, joint, existence, spatial, empty; exact component assignment.\n');
end
