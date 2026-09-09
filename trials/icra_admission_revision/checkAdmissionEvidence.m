function checkAdmissionEvidence()
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
modes={'all','positive','negative','consensus'};
[original,g]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
for i=1:numel(modes)
    [candidate,c]=fuseAdmissionEvidence({o,p},[.5,.5],model,d,cfg,['gaussian_evidence_decoupled_',modes{i}],20);
    assert(candidate.r==original.r && isequal(candidate.mu,original.mu) && isequal(candidate.Sigma,original.Sigma));
    assert(isequal(c.records,g.records));
end
% A spatially rejected positive increment changes only the admitted scalar.
a=o;a.Sigma={diag([16,16,4,4])};
a.localSpatialLogRatio=gaussianLogRatio(zeros(4,1),diag([8,8,4,4]),a.mu{1},a.Sigma{1});
for sign=[-1,1]
    a.localLogOddsIncrement=sign*.7;
    [original,g]=fuseGaussianEvidence({a,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
    assert(g.records(53)==0 && ~g.records(58));
    gate=.8;if sign<0,gate=.3;end
    for i=1:numel(modes)
        mode=modes{i};
        [candidate,c]=fuseAdmissionEvidence({a,p},[.5,.5],model,d,cfg,['gaussian_evidence_decoupled_',mode],20);
        restore=strcmp(mode,'all') || (sign>0 && strcmp(mode,'positive')) || ...
            (sign<0 && (strcmp(mode,'negative') || strcmp(mode,'consensus')));
        expected=logistic(logit(original.r)+double(restore)*.5*gate*a.localLogOddsIncrement);
        assert(abs(candidate.r-expected)<2e-14);
        assert(isequal(candidate.mu,original.mu) && isequal(candidate.Sigma,original.Sigma));
        assert(isequal(c.records(53:60),g.records(53:60)));
    end
end
% Opposite-sign current evidence disables consensus restoration.
a.localLogOddsIncrement=-.7;b=p;b.localLogOddsIncrement=.3;
[original,~]=fuseGaussianEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence',20);
[candidate,~]=fuseAdmissionEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence_decoupled_consensus',20);
assert(candidate.r==original.r && isequal(candidate.mu,original.mu));
% Fixed strength uses the exact original rule with manually set supports.
for code={'000','050','100','125'}
    eta=str2double(code{1})/1000;
    [candidate,c]=fuseAdmissionEvidence({o,p},[.5,.5],model,d,cfg,['gaussian_evidence_fixedx_',code{1}],20);
    a=o;b=p;a.positiveInnovationSupport=eta;a.negativeInnovationSupport=eta;
    b.positiveInnovationSupport=eta;b.negativeInnovationSupport=eta;
    [manual,m]=fuseGaussianEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence',20);
    assert(candidate.r==manual.r && isequal(candidate.mu,manual.mu) && isequal(candidate.Sigma,manual.Sigma));
    assert(isequal(c.records,m.records));
end
% Stale, unrepresented, and single-source cases admit no current increment.
for i=1:numel(modes)
    arm=['gaussian_evidence_decoupled_',modes{i}];a=o;b=p;
    a.lastDirectOpportunity=19;b.lastDirectOpportunity=19;
    [candidate,c]=fuseAdmissionEvidence({a,b},[.5,.5],model,d,cfg,arm,20);
    [manual,~]=fuseGaussianEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence',20);
    assert(all(c.records(53:54)==0) && candidate.r==manual.r);
    [single,c]=fuseAdmissionEvidence({o,p},[1,0],model,d,cfg,arm,20);
    assert(all(c.records(53:54)==0) && abs(single.r-o.r)<1e-12);
    [single,c]=fuseAdmissionEvidence({o,model.object},[.5,.5],model,d,cfg,arm,20);
    assert(all(c.records(53:54)==0) && isfinite(single.r));
end
% Aggregate spatial fallback preserves a finite base density and scalar rule.
a=o;a.Sigma={diag([1e14,1e14,1,1])};a.localSpatialLogRatio=zeros(1,15);b=a;
[original,g]=fuseGaussianEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence',20);
[candidate,c]=fuseAdmissionEvidence({a,b},[.5,.5],model,d,cfg,'gaussian_evidence_decoupled_all',20);
assert(g.records(60) && c.records(60) && all(c.records(53:54)==0));
assert(isequal(candidate.mu,original.mu) && isequal(candidate.Sigma,original.Sigma));
assert(abs(candidate.r-logistic(logit(original.r)+.8*.7))<2e-14);
fprintf('ADMISSION CHECK PASSED: exact admitted-source parity, sign separation, consensus, fixed-strength endpoints, stale/single/absent source, and aggregate fallback.\n');
end

function z=logit(r),z=log(r)-log1p(-r);end
function r=logistic(z),if z>=0,r=1/(1+exp(-z));else,e=exp(z);r=e/(1+e);end;end
