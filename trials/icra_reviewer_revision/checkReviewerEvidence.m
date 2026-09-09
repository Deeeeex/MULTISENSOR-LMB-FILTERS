function checkReviewerEvidence()
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
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
[base,~]=fuseLmbPosteriorsByLabel({o,p},[.5,.5],model,[.5,.5],d,cfg);
assert(isequal(gs.mu,base.mu) && isequal(gs.Sigma,base.Sigma));
assert(isequal(s.records(53:54),g.records(53:54)) && isequal(s.records(58:60),g.records(58:60)));
assert(s.records(57)==s.records(11));
expected=logistic(sum(s.records(29:30).*[logit(o.r),logit(p.r)])+ ...
    sum(s.records(53:54).*[o.localLogOddsIncrement,p.localLogOddsIncrement])+s.records(11));
assert(abs(gs.r-expected)<1e-14 && max(abs(gs.Sigma{1}-gce.Sigma{1}),[],'all')>1e-3);
for code={'025','050','100'}
    arm=['gaussian_evidence_fixed_',code{1}];lambda=str2double(code{1})/100;
    [fixed,f]=fuseReviewerEvidence({o,p},[.5,.5],model,d,cfg,arm,20);
    a=o;b=p;a.positiveInnovationSupport=lambda;a.negativeInnovationSupport=lambda;
    b.positiveInnovationSupport=lambda;b.negativeInnovationSupport=lambda;
    [manual,m]=fuseGaussianEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence',20);
    assert(fixed.r==manual.r && isequal(fixed.mu,manual.mu) && isequal(fixed.Sigma,manual.Sigma));
    assert(isequal(f.records,m.records));
end
% Rejected curvature suppresses the whole scalar increment as in GCE.
a=o;a.Sigma={diag([16,16,4,4])};
a.localSpatialLogRatio=gaussianLogRatio(zeros(4,1),diag([8,8,4,4]),a.mu{1},a.Sigma{1});
[~,g]=fuseGaussianEvidence({a,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
[~,s]=fuseReviewerEvidence({a,p},[.5,.5],model,d,cfg,'gaussian_evidence_guarded_scalar',20);
assert(s.records(53)==0 && ~s.records(58) && isequal(s.records(53:54),g.records(53:54)));
% Stale sources never receive a fixed correction even when lambda=1.
a=o;b=p;a.lastDirectOpportunity=19;b.lastDirectOpportunity=19;
[~,f]=fuseReviewerEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence_fixed_100',20);
assert(all(f.records(53:54)==0));
[single,f]=fuseReviewerEvidence({o,p},[1,0],model,d,cfg,'gaussian_evidence_guarded_scalar',20);
assert(all(f.records(53:54)==0) && abs(single.r-o.r)<1e-12);
% Shared numerical guard includes the aggregate condition-number fallback.
a=o;a.Sigma={diag([1e14,1e14,1,1])};a.localSpatialLogRatio=zeros(1,15);b=a;
[~,g]=fuseGaussianEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence',20);
[~,s]=fuseReviewerEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence_guarded_scalar',20);
assert(g.records(60) && s.records(60) && all(s.records(53:54)==0));
fprintf('REVIEW CONTROL CHECK PASSED: GS p0/I0 and guarded exponents; fixed ratio equivalence; rejection, stale/single source, aggregate fallback.\n');
end

function z=logit(r),z=log(r)-log1p(-r);end
function r=logistic(z),if z>=0,r=1/(1+exp(-z));else,e=exp(z);r=e/(1+e);end;end
