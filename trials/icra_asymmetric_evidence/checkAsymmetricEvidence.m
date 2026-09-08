function checkAsymmetricEvidence()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'trials','icra_selective_innovation'));
checkSelectiveInnovations();
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.5;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
o.localLogOddsIncrement=log(9);o.positiveInnovationSupport=1;o.negativeInnovationSupport=1;
model.birthParameters=o;model.object=o([]);p=o;
cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',20);
for arm={'asymmetric','asymmetric_no_history','asymmetric_no_mark'}
    for delta=[log(9),-log(9),0]
        o.r=.5;p.r=.5;o.localLogOddsIncrement=delta;p.localLogOddsIncrement=delta;
        [one,~]=fuseAsymmetricEvidence({o,p},[.5,.5],model,d,cfg,arm{1},20);
        assert(abs(one.r-logistic(delta))<1e-12 && isequal(one.mu,o.mu) && isequal(one.Sigma,o.Sigma));
        assert(one.positiveInnovationSupport==0 && one.negativeInnovationSupport==0);
    end
    o.r=.9;p.r=.1;o.localLogOddsIncrement=log(9);p.localLogOddsIncrement=-log(9);
    [one,~]=fuseAsymmetricEvidence({o,p},[.5,.5],model,d,cfg,arm{1},20);
    assert(abs(one.r-.5)<1e-12);
    [one,~]=fuseAsymmetricEvidence({o,p},[1,0],model,d,cfg,arm{1},20);
    assert(abs(one.r-o.r)<1e-12);
    p.hasObservationLineage=false;
    [one,st]=fuseAsymmetricEvidence({o,p},[.5,.5],model,d,cfg,arm{1},20);
    assert(abs(one.r-o.r)<1e-12 && st.lineageExcluded==1);p.hasObservationLineage=true;
end
% No negative gate is precisely the previous selective scalar/spatial rule.
o.negativeInnovationSupport=0;p.negativeInnovationSupport=0;p.lastDirectOpportunity=1;
[one,st]=fuseAsymmetricEvidence({o,p},[.5,.5],model,d,cfg,'asymmetric',20);
[old,oldStats]=fuseSelectiveInnovations({o,p},[.5,.5],model,d,cfg,'selective',20);
assert(one.r==old.r && isequal(one.mu,old.mu) && isequal(one.Sigma,old.Sigma));
assert(isequaln(st.records(1:35),oldStats.records));
o.lastDirectOpportunity=19;o.negativeInnovationSupport=1;p.negativeInnovationSupport=1;
[~,st]=fuseAsymmetricEvidence({o,p},[.5,.5],model,d,cfg,'asymmetric',20);
assert(all(st.records([27,28,36,37])==0));
g=negativeInnovationSupport([0,.25,1,0],[.9,.9,.9,0]);
assert(max(abs(g-[.9/1.1,.75*.9/1.1,0,0]))<1e-14);
assert(isempty(negativeInnovationSupport([],[])));
[decoded,bytes]=asymmetricInnovationPacket([o,p],model,1,20);
assert(numel(bytes)==32+2*232);
assert(isequal([decoded.negativeInnovationSupport],[o.negativeInnovationSupport,p.negativeInnovationSupport]));
assert(isequal([decoded.positiveInnovationSupport],[o.positiveInnovationSupport,p.positiveInnovationSupport]));
assert(isequal([decoded.localLogOddsIncrement],[o.localLogOddsIncrement,p.localLogOddsIncrement]));
fprintf('ASYMMETRIC CHECK PASSED: both evidence signs, opposing scalar endpoints, zero-negative exact SI fallback, single-source/exclusion, stale reset, missed-likelihood contrast and native 232 B round trip.\n');
end

function value=logistic(value),value=1/(1+exp(-value));end
