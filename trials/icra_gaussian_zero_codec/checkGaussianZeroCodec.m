function checkGaussianZeroCodec()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'trials','icra_gaussian_evidence'),fullfile(root,'trials','icra_asymmetric_evidence'), ...
    fullfile(root,'trials','icra_selective_innovation'),fullfile(root,'trials','icra_method_iteration'), ...
    fullfile(root,'trials','icra_fusion_holdout'),fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'));
base=uint8(mod(0:263,256));x=zeros(15,4);
negativeZero=typecast(bitshift(uint64(1),63),'double');
x(1:2:15,2)=negativeZero;x(:,3)=(1:15)'/7;x(3:3:15,4)=negativeZero;
[bytes,y,b,tags]=gaussianZeroCodec(base,x);
assert(numel(bytes)==numel(base)+8+120 && isequal(b,base));
assert(tags(1)==0 && tags(3)==32768 && tags(2)>0 && tags(4)>0);
assert(isequal(typecast(x(:),'uint64'),typecast(y(:),'uint64')));
[bytes,y,b,tags]=gaussianZeroCodec(base,zeros(15,0));
assert(isequal(bytes,base) && isequal(b,base) && isempty(y) && isempty(tags));
checkGaussianEvidence();
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.2;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={diag([25,25,4,4])};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
o.localLogOddsIncrement=0;o.positiveInnovationSupport=1;o.negativeInnovationSupport=1;
o.localSpatialLogRatio=x(:,2)';model.birthParameters=o;model.object=o([]);
p=o;p.birthLocation=2;p.localSpatialLogRatio=x(:,3)';
[received,bytes,tags]=gaussianZeroPacket([o,p],model,1,20);
[old,oldBytes]=gaussianEvidencePacket([o,p],model,1,20);
assert(numel(bytes)==32+234*2+120 && numel(oldBytes)==32+352*2);
assert(isequaln(received,old));
assert(isequal(typecast([received.localSpatialLogRatio],'uint64'),typecast([o.localSpatialLogRatio,p.localSpatialLogRatio],'uint64')));
[received,bytes,tags]=gaussianZeroPacket(model.object,model,1,20);
assert(isempty(received) && numel(bytes)==32 && isempty(tags));
fprintf('GAUSSIAN ZERO CODEC UNIT PASS: empty, opaque base, mixed signed zeros, nonzero vectors and actual Bernoulli packets all 64-bit exact; original fusion fixtures pass.\n');
end
