function checkProjectedEvidence()
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
p=o;p.mu={[-1;1;0;0]};p.r=.4;p.localLogOddsIncrement=-.3;
p.localSpatialLogRatio=gaussianLogRatio(priorMean,priorCov,p.mu{1},p.Sigma{1});
modes={'space','all','consensus'};
[original,g]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
for i=1:numel(modes)
    [candidate,c]=fuseProjectedEvidence({o,p},[.5,.5],model,d,cfg,['gaussian_evidence_projected_',modes{i}],20);
    assert(candidate.r==original.r && isequal(candidate.mu,original.mu) && isequal(candidate.Sigma,original.Sigma));
    assert(isequal(c.records,g.records));
end
% A mixed update preserves the normalized one-dimensional contracting marginal.
a=o;a.mu={[1;2;0;0]};a.Sigma={diag([4,16,4,4])};
a.localSpatialLogRatio=gaussianLogRatio(priorMean,priorCov,a.mu{1},a.Sigma{1});
% Use a zero-weight carrier to apply this ratio to the prior density.
base=o;base.mu={priorMean};base.Sigma={priorCov};base.localSpatialLogRatio=zeros(1,15);
[m,P,I,k,allowed,fallback,ranks]=applyProjectedEvidence({base,a},[1,0],[0,1]);
assert(norm(m-[1;0;0;0])<1e-12 && norm(P-diag([4,8,4,4]),'fro')<1e-12);
assert(abs(I)<1e-12 && isequal(k,[0,1]) && ~allowed(2) && ~fallback && ranks(2)==1);
% General invertible coordinate changes preserve the selected marginal ratio.
A=[2,.3,0,.2;0,.7,.1,0;0,0,1.4,.2;.1,0,0,1];shift=[8;-6;2;1];
changedBase=base;changedA=a;
changedBase.mu={A*priorMean+shift};changedBase.Sigma={A*priorCov*A'};
changedA.mu={A*a.mu{1}+shift};changedA.Sigma={A*a.Sigma{1}*A'};
changedA.localSpatialLogRatio=gaussianLogRatio(changedBase.mu{1},changedBase.Sigma{1},changedA.mu{1},changedA.Sigma{1});
[newM,newP,newI]=applyProjectedEvidence({changedBase,changedA},[1,0],[0,1]);
assert(norm(newM-(A*m+shift))<1e-10 && norm(newP-A*P*A','fro')<1e-10 && abs(newI-I)<1e-10);
% A uniformly expanding posterior supplies no contracting spatial evidence.
wide=a;wide.Sigma={2*priorCov};wide.localSpatialLogRatio=gaussianLogRatio(priorMean,priorCov,wide.mu{1},wide.Sigma{1});
[m,P,I,k,allowed,~,ranks]=applyProjectedEvidence({base,wide},[1,0],[0,1]);
assert(norm(m-priorMean)<1e-12 && norm(P-priorCov,'fro')<1e-12 && abs(I)<1e-12);
assert(k(2)==0 && ~allowed(2) && ranks(2)==0);
% Scalar selection uses original guarded exponents and current sign agreement.
for sign=[-1,1]
    a.localLogOddsIncrement=sign*.7;
    [~,legacy]=fuseGaussianEvidence({a,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
    assert(legacy.records(53)==0);
    values=zeros(1,3);
    for i=1:numel(modes)
        [candidate,c]=fuseProjectedEvidence({a,p},[.5,.5],model,d,cfg,['gaussian_evidence_projected_',modes{i}],20);
        values(i)=logit(candidate.r)-c.records(57);
        assert(c.records(53)>0 && ~c.records(58));
    end
    gate=.8;if sign<0,gate=.3;end
    assert(abs(values(2)-values(1)-.5*gate*a.localLogOddsIncrement)<1e-12);
    assert(abs(values(3)-values(2))<1e-12);
end
a.localLogOddsIncrement=-.7;b=p;b.localLogOddsIncrement=.3;
[space,~]=fuseProjectedEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence_projected_space',20);
[consensus,~]=fuseProjectedEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence_projected_consensus',20);
assert(space.r==consensus.r && isequal(space.mu,consensus.mu));
for i=1:numel(modes)
    arm=['gaussian_evidence_projected_',modes{i}];a=o;b=p;
    a.lastDirectOpportunity=19;b.lastDirectOpportunity=19;
    [candidate,c]=fuseProjectedEvidence({a,b},[.5,.5],model,d,cfg,arm,20);
    [manual,~]=fuseGaussianEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence',20);
    assert(all(c.records(53:54)==0) && candidate.r==manual.r);
    [single,c]=fuseProjectedEvidence({o,p},[1,0],model,d,cfg,arm,20);
    assert(all(c.records(53:54)==0) && abs(single.r-o.r)<1e-12);
    [single,c]=fuseProjectedEvidence({o,model.object},[.5,.5],model,d,cfg,arm,20);
    assert(all(c.records(53:54)==0) && isfinite(single.r));
end
a=o;a.Sigma={diag([1e14,1e14,1,1])};a.localSpatialLogRatio=zeros(1,15);b=a;
[candidate,c]=fuseProjectedEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence_projected_all',20);
assert(c.records(60) && all(c.records(53:54)==0) && isfinite(candidate.r));
fprintf('PROJECTED CHECK PASSED: exact admitted-source parity, normalized contracting marginal, coordinate invariance, expansion rejection, scalar rules, opportunity limits, and aggregate fallback.\n');
end

function z=logit(r),z=log(r)-log1p(-r);end
