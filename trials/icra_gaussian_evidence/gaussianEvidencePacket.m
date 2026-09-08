function [decoded,bytes]=gaussianEvidencePacket(objects,model,source,t)
[decoded,base]=asymmetricInnovationPacket(objects,model,source,t);
encoded=reshape([objects.localSpatialLogRatio],1,[]);
assert(all(isfinite(encoded)) && numel(encoded)==15*numel(objects));
bytes=[base,reshape(typecast(double(encoded),'uint8'),1,[])];
received=reshape(typecast(bytes(numel(base)+1:end),'double'),15,[]);
assert(isequal(encoded,reshape(received,1,[])));
for j=1:numel(decoded),decoded(j).localSpatialLogRatio=received(:,j)';end
assert(numel(bytes)==32+352*numel(objects));
end
