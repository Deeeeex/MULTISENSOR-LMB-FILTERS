function checkCompatibleEvidence()
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.6;
o.numberOfGmComponents=1;o.w=1;o.mu={[1;0;0;0]};o.Sigma={diag([2,2,4,4])};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
o.localLogOddsIncrement=.7;o.positiveInnovationSupport=.8;o.negativeInnovationSupport=.3;
priorMean=zeros(4,1);priorCov=diag([8,8,4,4]);
o.localSpatialLogRatio=gaussianLogRatio(priorMean,priorCov,o.mu{1},o.Sigma{1});
model.birthParameters=o;model.object=o([]);
cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',20);
modes={'agreement_all','agreement_positive','agreement_conflict','positive_consensus','positive_only'};
% Identical posteriors and positive current updates recover the original rule.
[reference,~]=fuseGaussianEvidence({o,o},[.5,.5],model,d,cfg,'gaussian_evidence',20);
for k=1:numel(modes)
    [candidate,c]=fuseCompatibleEvidence({o,o},[.5,.5],model,d,cfg,['gaussian_evidence_compatible_',modes{k}],20);
    assert(abs(candidate.r-reference.r)<1e-14 && norm(candidate.mu{1}-reference.mu{1})<1e-14);
    assert(norm(candidate.Sigma{1}-reference.Sigma{1},'fro')<1e-14);
end
% A sign conflict admits the independently computed overlap-scaled exponents.
p=o;p.mu={[-1;1;0;0]};p.r=.4;p.localLogOddsIncrement=-.3;
p.localSpatialLogRatio=gaussianLogRatio(priorMean,priorCov,p.mu{1},p.Sigma{1});
[~,base]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
overlap=exp(base.records(11));assert(overlap>0 && overlap<1);
factors={[overlap,overlap],[overlap,1],[overlap,overlap],[0,1],[1,0]};
for k=1:numel(modes)
    [candidate,c]=fuseCompatibleEvidence({o,p},[.5,.5],model,d,cfg,['gaussian_evidence_compatible_',modes{k}],20);
    expected=[.4,.15].*factors{k};assert(norm(c.records(53:54)-expected)<1e-14);
    z=.5*(logit(o.r)+logit(p.r))+sum(expected.*[.7,-.3])+c.records(57);
    assert(abs(candidate.r-logistic(z))<1e-14);
end
% Suppressing added misses retains the already updated local posterior pool.
a=o;a.localLogOddsIncrement=-.7;
[candidate,c]=fuseCompatibleEvidence({a,p},[.5,.5],model,d,cfg,'gaussian_evidence_compatible_positive_only',20);
assert(all(c.records(53:54)==0));
assert(abs(candidate.r-logistic(.5*(logit(a.r)+logit(p.r))+c.records(11)))<1e-14);
assert(norm(candidate.mu{1}(1:2)-c.records(39:40)')<1e-14);
% Source curvature rejection is unchanged even with a positive compatibility.
a=o;a.Sigma={2*priorCov};a.localSpatialLogRatio=gaussianLogRatio(priorMean,priorCov,a.mu{1},a.Sigma{1});
[~,c]=fuseCompatibleEvidence({a,p},[.5,.5],model,d,cfg,'gaussian_evidence_compatible_agreement_all',20);
assert(c.records(53)==0 && ~c.records(58));
for k=1:numel(modes)
    arm=['gaussian_evidence_compatible_',modes{k}];a=o;b=p;
    a.lastDirectOpportunity=19;b.lastDirectOpportunity=19;
    [candidate,c]=fuseCompatibleEvidence({a,b},[.5,.5],model,d,cfg,arm,20);
    [manual,~]=fuseGaussianEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence',20);
    assert(all(c.records(53:54)==0) && candidate.r==manual.r);
    [single,c]=fuseCompatibleEvidence({o,p},[1,0],model,d,cfg,arm,20);
    assert(all(c.records(53:54)==0) && abs(single.r-o.r)<1e-12);
    [single,c]=fuseCompatibleEvidence({o,model.object},[.5,.5],model,d,cfg,arm,20);
    assert(all(c.records(53:54)==0) && isfinite(single.r));
end
a=o;a.Sigma={diag([1e14,1e14,1,1])};a.localSpatialLogRatio=zeros(1,15);b=a;
[candidate,c]=fuseCompatibleEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence_compatible_agreement_all',20);
assert(c.records(60) && all(c.records(53:54)==0) && isfinite(candidate.r));
fprintf('COMPATIBLE CHECK PASSED: equal-source parity, overlap/sign factors, no-added-miss pool, original curvature rejection, opportunity limits, and aggregate fallback.\n');
end

function z=logit(r),z=log(r)-log1p(-r);end
function r=logistic(z),if z>=0,r=1/(1+exp(-z));else,e=exp(z);r=e/(1+e);end;end
