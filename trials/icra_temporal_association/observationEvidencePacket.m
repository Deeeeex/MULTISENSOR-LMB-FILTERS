function [decoded,bytes]=observationEvidencePacket(objects,model,source,t)
[decoded,base]=gaussianEvidencePacket(objects,model,source,t);
encoded=reshape([objects.localDirectObservation],1,[]);
assert(all(isfinite(encoded)) && numel(encoded)==8*numel(objects));
bytes=[base,reshape(typecast(double(encoded),'uint8'),1,[])];
received=reshape(typecast(bytes(numel(base)+1:end),'double'),8,[]);
assert(isequal(encoded,reshape(received,1,[])));
for j=1:numel(decoded),decoded(j).localDirectObservation=received(:,j)';end
assert(numel(bytes)==32+416*numel(objects));
end
