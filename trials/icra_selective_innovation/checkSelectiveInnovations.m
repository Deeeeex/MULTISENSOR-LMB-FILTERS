function checkSelectiveInnovations()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_reunion_fusion'),fullfile(root,'trials','icra_external_fusion'), ...
    fullfile(root,'trials','icra_method_iteration'),fullfile(root,'trials','icra_ceiling_iteration'));
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.5;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
o.localLogOddsIncrement=log(9);o.positiveInnovationSupport=1;
model.birthParameters=o;model.object=o([]);p=o;
cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',20);
for arm={'selective','selective_no_history','selective_no_mark','selective_signed'}
    % Two identical posteriors from prior .1 and independent positive LR=9.
    [one,~]=fuseSelectiveInnovations({o,p},[.5,.5],model,d,cfg,arm{1},20);
    assert(abs(one.r-.9)<1e-12 && isequal(one.mu,o.mu) && isequal(one.Sigma,o.Sigma));
    assert(one.positiveInnovationSupport==0);
    [single,~]=fuseSelectiveInnovations({o,p},[1,0],model,d,cfg,arm{1},20);
    assert(abs(single.r-o.r)<1e-12);
    p.hasObservationLineage=false;
    [single,stats]=fuseSelectiveInnovations({o,p},[.5,.5],model,d,cfg,arm{1},20);
    assert(abs(single.r-o.r)<1e-12 && stats.lineageExcluded==1);
    p.hasObservationLineage=true;
    [missing,stats]=fuseSelectiveInnovations({o,model.object},[.5,.5],model,d,cfg,arm{1},20);
    assert(stats.observableAbsences==1 && stats.records(31)==0 && isfinite(missing.r));
end
% A zero or stale gate and a negative innovation cannot add positive evidence.
o.r=.9;p.r=.1;p.lastDirectOpportunity=1;
for delta=[-log(9),0,log(9)]
    o.localLogOddsIncrement=delta;p.localLogOddsIncrement=delta;
    for gate=[0,.2,.7,1]
        o.positiveInnovationSupport=gate;
        [one,st]=fuseSelectiveInnovations({o,p},[.5,.5],model,d,cfg,'selective',20);
        expected=logistic(.5*gate*max(delta,0));
        assert(abs(one.r-expected)<1e-12 && st.records(28)==0);
        [ref,~]=fuseConservativeRecency({o,p},[.5,.5],model,d,cfg,'conservative_recency',20);
        assert(one.r>=ref.r-1e-12 && isequal(one.mu,ref.mu) && isequal(one.Sigma,ref.Sigma));
    end
end
o.lastDirectOpportunity=19;
[one,st]=fuseSelectiveInnovations({o,p},[.5,.5],model,d,cfg,'selective',20);
assert(abs(one.r-.5)<1e-12 && all(st.records(27:28)==0));
% Negative age shifts use q; numerical ties use b deterministically.
o.lastDirectOpportunity=20;o.r=.1;p.r=.9;o.localLogOddsIncrement=0;p.localLogOddsIncrement=0;
[one,st]=fuseSelectiveInnovations({o,p},[.5,.5],model,d,cfg,'selective',20);
[ref,~]=fuseConservativeRecency({o,p},[.5,.5],model,d,cfg,'conservative_recency',20);
assert(abs(one.r-ref.r)<1e-12 && isequal(st.records(29:30),st.records(12:13)));
o.r=.5;p.r=.5;
[~,st]=fuseSelectiveInnovations({o,p},[.5,.5],model,d,cfg,'selective',20);
assert(isequal(st.records(29:30),[.5,.5]));
% Association weighting, neutral marks, no opportunity, and no-mark ablation.
W=[.2,.3,.5;.4,.5,.1];L=[9,1/9];
[g,a]=positiveInnovationSupport(W,L,[true,false],false);
assert(max(abs(g-[.24,0]))<1e-14 && max(abs(a-[.8,0]))<1e-14);
[g,a]=positiveInnovationSupport(W,L,[true,true],true);
assert(isequal(g,a) && max(abs(a-[.8,.6]))<1e-14);
[g,~]=positiveInnovationSupport(W,[1,1],[true,true],false);assert(all(g==0));
[g,~]=positiveInnovationSupport([],[],[true,false],false);assert(all(g==0));
[decoded,bytes]=selectiveInnovationPacket([o,p],model,1,20);
assert(numel(bytes)==32+2*224 && isequal([decoded.localLogOddsIncrement],[o.localLogOddsIncrement,p.localLogOddsIncrement]));
assert(isequal([decoded.positiveInnovationSupport],[o.positiveInnovationSupport,p.positiveInnovationSupport]));
fprintf('SELECTIVE CHECK PASSED: scalar identity, gated increments, conservative fallback, stale resets, numerical ties, exclusion, censor, spatial parity, association and mark endpoints, 224 B packet round trip.\n');
end

function value=logistic(value),value=1/(1+exp(-value));end
