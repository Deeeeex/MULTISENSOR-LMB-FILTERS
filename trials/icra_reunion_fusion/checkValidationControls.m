function checkValidationControls()
% Independent analytic cases for the additional support-partition MIL arm.
out=fileparts(mfilename('fullpath')); root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'));
model=generateMultisensorModel(3,ones(1,3),.9*ones(1,3),.4*ones(1,3),'GA','LBP');
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.8;
o.mu={zeros(4,1)};o.Sigma={eye(4)};o.w=1;o.numberOfGmComponents=1;
o.hasObservationLineage=true;o.lastDirectOpportunity=0;o.positiveConfirmation=false;
model.object=o([]);weak=o;weak.r=.2;weak.mu={[2;0;0;0]};
cfg=buildMixtureAwareKlaReferenceConfig();
details=struct('eventType',[0,2,2],'sourceIndices',[1,2,3], ...
    'isStale',false(1,3),'isSelf',[true,false,false],'currentTime',10);
inputs={o,weak,o([])};weights=[.2,.3,.5];
[support,~]=fuseValidationInputs(inputs,weights,model,details,cfg,'mil_support',10);
zero=fuseCommonLabelLmbMil(inputs,weights,model,8);
assert(abs(support.r-.44)<1e-12 && abs(zero.r-.22)<1e-12);
expectedMean=(.3*.2*2)/(.2*.8+.3*.2);
assert(abs(sum(support.w.*cellfun(@(x)x(1),support.mu))-expectedMean)<1e-12);
alone=fuseValidationInputs({o,o([]),o([])},weights,model,details,cfg,'mil_support',10);
assert(abs(alone.r-o.r)<1e-12 && isequal(alone.mu,o.mu));
permuted=fuseValidationInputs(inputs([3,1,2]),weights([3,1,2]),model,details,cfg,'mil_support',10);
assert(abs(permuted.r-support.r)<1e-12 && isequal(permuted.mu,support.mu) && isequal(permuted.w,support.w));
fprintf('PASS MIL support specialization: zero-extension distinction, exact arithmetic existence/spatial mean, exclusive-label identity, source permutation.\n');
end
