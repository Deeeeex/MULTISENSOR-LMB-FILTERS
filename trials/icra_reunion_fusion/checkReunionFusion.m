function checkReunionFusion()
% Check fusion semantics and the per-label wrapper independently of task truth.
out=fileparts(mfilename('fullpath')); root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'));
model=generateMultisensorModel(2,[1,1],[.9,.9],[.4,.4],'GA','LBP');
model.T=.5;
o=model.birthParameters(1); o.birthTime=1; o.birthLocation=1;
o.mu={zeros(4,1)}; o.Sigma={eye(4)}; o.w=1; o.numberOfGmComponents=1;
o.hasObservationLineage=true; o.lastDirectOpportunity=50; o.r=.9;
model.object=o([]);
weak=o; weak.r=.1; weak.hasObservationLineage=false; weak.lastDirectOpportunity=0;
cfg=buildMixtureAwareKlaReferenceConfig();
details=struct('eventType',[0,2],'sourceIndices',[1,2], ...
    'isStale',[false,false],'isSelf',[true,false],'currentTime',50);
[ordinary,~]=fuseRobotReunionInputs({o,weak},[.5,.5],model,details,cfg,'kla',50);
[recent,~]=fuseRobotReunionInputs({o,weak},[.5,.5],model,details,cfg,'recent',50);
assert(abs(ordinary.r-.5)<1e-10 && recent.r>ordinary.r);
assert(abs(recent.r-1/(1+exp(-.6*log(9))))<1e-10);
% Recent negative evidence is not suppressed merely because r is low.
negative=o; negative.r=.1;
old=weak; old.r=.9;
[result,~]=fuseRobotReunionInputs({negative,old},[.5,.5],model,details,cfg,'recent',50);
assert(result.r<.5 && abs(result.r-(1-recent.r))<1e-10);
% No direct opportunity anywhere is exactly the ordinary-weight fallback.
a=o; a.lastDirectOpportunity=0;
[result,stats]=fuseRobotReunionInputs({a,weak},[.5,.5],model,details,cfg,'recent',50);
assert(abs(result.r-ordinary.r)<1e-10 && stats.weightChange==0);
% Per-label decomposition does not change the existing ordinary fusion rule.
b=o; b.birthLocation=2; b.mu={[8;0;0;0]}; b.r=.8;
c=b; c.r=.3;
inputs={[o,b],[weak,c]};
expected=fuseLmbPosteriorsByLabel(inputs,[.5,.5],model,[.5,.5],details,cfg);
actual=fuseRobotReunionInputs(inputs,[.5,.5],model,details,cfg,'kla',50);
assert(numel(actual)==numel(expected));
for k=1:numel(expected)
    assert(abs(actual(k).r-expected(k).r)<1e-12);
    assert(norm(actual(k).mu{1}-expected(k).mu{1})<1e-12);
    assert(norm(actual(k).Sigma{1}-expected(k).Sigma{1},'fro')<1e-12);
end
% A pure source-order permutation changes neither density nor temporal score.
reverse=details; reverse.sourceIndices=[2,1]; reverse.isSelf=[false,true]; reverse.eventType=[2,0];
permuted=fuseRobotReunionInputs({weak,o},[.5,.5],model,reverse,cfg,'recent',50);
assert(abs(permuted.r-recent.r)<1e-12);
fprintf('PASS reunion semantics: positive/negative evidence, no-opportunity fallback, wrapper equivalence, source-order invariance.\n');
end
