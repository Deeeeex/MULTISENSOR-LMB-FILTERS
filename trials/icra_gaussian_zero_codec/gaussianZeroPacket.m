function [decoded,bytes,tags]=gaussianZeroPacket(objects,model,source,t)
[decoded,base]=asymmetricInnovationPacket(objects,model,source,t);
encoded=reshape([objects.localSpatialLogRatio],15,[]);
[bytes,received,restoredBase,tags]=gaussianZeroCodec(base,encoded);
assert(isequal(base,restoredBase));
for j=1:numel(decoded),decoded(j).localSpatialLogRatio=received(:,j)';end
assert(numel(bytes)==32+234*numel(objects)+120*nnz(any(encoded~=0,1)));
end
