function checkGaussianMil()
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_reunion_fusion'));
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');
one=model.birthParameters(1); one.birthTime=1; one.birthLocation=1;
one.r=.8; one.numberOfGmComponents=1; one.w=1; one.mu={zeros(4,1)}; one.Sigma={eye(4)};
one.hasObservationLineage=true; one.lastDirectOpportunity=1; one.positiveConfirmation=false;
two=one; two.birthLocation=200001; two.r=.6; two.mu={[1;0;0;0]};
far=two; far.birthLocation=200002; far.mu={[100;0;0;0]};
[mapped,stats]=alignGaussianLmbPair(one,[two,far],50);
assert(mapped(1).birthLocation==1 && mapped(2).birthLocation==200002 && stats.assignedPairs==1);
low=two; low.r=1e-10; [lowMapped,~]=alignGaussianLmbPair(one,low,50);
assert(lowMapped.birthLocation==1,'Matching must not use Bernoulli existence.');
% 0.25*(trace(I)+trace(I)+delta''(2I)delta-8)=0.5 for delta=[1,0,0,0].
assert(abs(stats.maximumMatchedCost-.5)<1e-12);
model.object=one([]); cfg=buildMixtureAwareKlaReferenceConfig();
details=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',1);
[fused,~]=fuseValidationInputs({one,mapped},[.5,.5],model,details,cfg,'mil_support',1);
shared=fused([fused.birthLocation]==1); exclusive=fused([fused.birthLocation]==200002);
assert(abs(shared.r-.7)<1e-12 && abs(exclusive.r-.6)<1e-12);
mu=cell2mat(shared.mu)*shared.w'; assert(norm(mu-[3/7;0;0;0])<1e-12);
covariance=zeros(4);
for g=1:numel(shared.w)
    delta=shared.mu{g}-mu;covariance=covariance+shared.w(g)*(shared.Sigma{g}+delta*delta');
end
assert(norm(covariance-diag([61/49,1,1,1]),'fro')<1e-12);
assert(isequal(exclusive.mu,far.mu) && isequal(exclusive.Sigma,far.Sigma));
% Empty input identity and input order/weight covariance.
[empty,~]=alignGaussianLmbPair(model.object,two,50); assert(isequaln(empty,two));
[reversed,~]=fuseValidationInputs({mapped,one},[.5,.5],model,details,cfg,'mil_support',1);
assert(abs(reversed([reversed.birthLocation]==1).r-shared.r)<1e-12);
fprintf('GAUSSIAN MIL PASS: independent labels, exact symmetric KL, unmatched slots, existence-independent assignment, common/exclusive subspaces, arithmetic moments, identity and source reversal.\n');
end
