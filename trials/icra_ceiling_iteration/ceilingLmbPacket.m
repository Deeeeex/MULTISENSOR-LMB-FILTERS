function [decoded,bytes]=ceilingLmbPacket(objects,model,source,t)
% One bounded support scalar, computed afresh at the sending sensor.
[decoded,base]=gaussianLmbPacket(objects,model,source,t);
v=reshape([objects.directEvidenceCeiling],1,[]);
assert(all(isfinite(v)) && all(v>=0 & v<=1));
bytes=[base,reshape(typecast(double(v),'uint8'),1,[])];
received=reshape(typecast(bytes(numel(base)+1:end),'double'),1,[]);
assert(isequal(v,received));
for j=1:numel(decoded),decoded(j).directEvidenceCeiling=received(j);end
assert(numel(bytes)==32+216*numel(objects));
end
