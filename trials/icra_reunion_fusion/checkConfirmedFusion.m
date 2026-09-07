function checkConfirmedFusion()
out=fileparts(mfilename('fullpath')); root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'));
model=generateMultisensorModel(2,[1,1],[.9,.9],[.4,.4],'GA','LBP'); model.T=.5;
o=model.birthParameters(1); o.birthTime=1; o.birthLocation=1;
o.mu={zeros(4,1)}; o.Sigma={eye(4)}; o.w=1; o.numberOfGmComponents=1;
o.hasObservationLineage=true; o.lastDirectOpportunity=50;
o.positiveConfirmation=false; o.r=.9; model.object=o([]);
weak=o; weak.r=.1; weak.hasObservationLineage=false; weak.lastDirectOpportunity=0;
cfg=buildMixtureAwareKlaReferenceConfig();
d=struct('eventType',[0,2],'sourceIndices',[1,2], ...
    'isStale',[false,false],'isSelf',[true,false],'currentTime',50);
ordinary=fuseLmbPosteriorsByLabel({o,weak},[.5,.5],model,[.5,.5],d,cfg);
noCertificate=fuseConfirmedReunionInputs({o,weak},[.5,.5],model,d,cfg,'confirmed_lineage',50);
assert(abs(noCertificate.r-ordinary.r)<1e-12 && ~noCertificate.positiveConfirmation);
o.positiveConfirmation=true;
certified=fuseConfirmedReunionInputs({o,weak},[.5,.5],model,d,cfg,'confirmed_lineage',50);
assert(abs(certified.r-o.r)<1e-12 && certified.positiveConfirmation);
% A valid miss remains negative even when its probability is low.
negative=weak; negative.r=.1; negative.hasObservationLineage=true;
negative.lastDirectOpportunity=50; o.lastDirectOpportunity=1;
current=fuseConfirmedReunionInputs({o,negative},[.5,.5],model,d,cfg,'confirmed_exist',50);
assert(current.r<.5 && current.positiveConfirmation);
% At fixed inputs, changing temporal trust leaves the spatial density fixed.
negative.mu={[1;0;0;0]}; negative.Sigma={diag([2,1,1,1])};
uniform=fuseConfirmedReunionInputs({o,negative},[.5,.5],model,d,cfg,'confirmed_lineage',50);
current=fuseConfirmedReunionInputs({o,negative},[.5,.5],model,d,cfg,'confirmed_exist',50);
assert(norm(uniform.mu{1}-current.mu{1})<1e-12);
assert(norm(uniform.Sigma{1}-current.Sigma{1},'fro')<1e-12);
assert(abs(uniform.r-current.r)>.01);
reverse=d; reverse.sourceIndices=[2,1]; reverse.isSelf=[false,true]; reverse.eventType=[2,0];
permuted=fuseConfirmedReunionInputs({negative,o},[.5,.5],model,reverse,cfg,'confirmed_exist',50);
assert(norm(permuted.mu{1}-current.mu{1})<1e-12 && abs(permuted.r-current.r)<1e-12);
fprintf('PASS confirmation fallback, certificate propagation, negative evidence, fixed-input spatial invariance, source permutation.\n');
end
