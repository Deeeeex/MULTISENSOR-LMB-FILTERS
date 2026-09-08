function checkJointEvidence()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_reunion_fusion'),fullfile(root,'trials','icra_external_fusion'), ...
    fullfile(root,'trials','icra_method_iteration'));
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.5;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
o.localLogOddsIncrement=log(9);model.birthParameters=o;model.object=o([]);p=o;
cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
cfg.missingLabelFusionMode='fov-aware-censored';cfg.untouchedPriorExclusionEnabled=true;
details=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',20);
for arm={'joint_evidence','joint_evidence_recency'}
    for increments={[log(9),log(9)],[-log(9),-log(9)],[log(9),-log(9)]}
        delta=increments{1};prior=.1;
        o.r=logistic(log(prior/(1-prior))+delta(1));p.r=logistic(log(prior/(1-prior))+delta(2));
        o.localLogOddsIncrement=delta(1);p.localLogOddsIncrement=delta(2);
        [one,~]=fuseJointEvidence({o,p},[.5,.5],model,details,cfg,arm{1},20);
        expected=logistic(log(prior/(1-prior))+sum(delta));
        assert(abs(one.r-expected)<1e-12 && isequal(one.mu,o.mu) && isequal(one.Sigma,o.Sigma));
    end
    [single,~]=fuseJointEvidence({o,p},[1,0],model,details,cfg,arm{1},20);
    assert(abs(single.r-o.r)<1e-12);
    p.hasObservationLineage=false;p.localLogOddsIncrement=100;
    [excluded,d]=fuseJointEvidence({o,p},[.5,.5],model,details,cfg,arm{1},20);
    assert(abs(excluded.r-o.r)<1e-12 && d.lineageExcluded==1);
    p.hasObservationLineage=true;p.localLogOddsIncrement=0;o.localLogOddsIncrement=0;
    p.lastDirectOpportunity=1;
    if strcmp(arm{1},'joint_evidence'),fallback='lineage';else,fallback='qualified_exist';end
    [zero,~]=fuseJointEvidence({o,p},[.5,.5],model,details,cfg,arm{1},20);
    [ref,~]=fuseValidationInputs({o,p},[.5,.5],model,details,cfg,fallback,20);
    assert(abs(zero.r-ref.r)<1e-12 && isequal(zero.mu,ref.mu) && isequal(zero.Sigma,ref.Sigma));
    o.localLogOddsIncrement=7;
    [missing,~]=fuseJointEvidence({o,model.object},[.5,.5],model,details,cfg,arm{1},20);
    [ref,~]=fuseValidationInputs({o,model.object},[.5,.5],model,details,cfg,fallback,20);
    assert(isequal(missing.r,ref.r) && isequal(missing.mu,ref.mu));
end
[decoded,bytes]=innovationLmbPacket([o,p],model,1,20);
assert(numel(bytes)==32+2*216 && isequal([decoded.localLogOddsIncrement],[o.localLogOddsIncrement,p.localLogOddsIncrement]));
fprintf('JE CHECK PASSED: common-prior scalar Bayes identity, positive/negative/opposing evidence, zero increment, single source, untouched exclusion, unchanged censor fallback and spatial density, packet round trip.\n');
end

function value=logistic(value),value=1/(1+exp(-value));end
