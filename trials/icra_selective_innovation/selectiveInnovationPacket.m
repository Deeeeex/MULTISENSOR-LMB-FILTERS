function [decoded,bytes]=selectiveInnovationPacket(objects,model,source,t)
% Native two-scalar extension; receiving never reads a remote private object.
[decoded,base]=innovationLmbPacket(objects,model,source,t);
gate=reshape([objects.positiveInnovationSupport],1,[]);
assert(all(isfinite(gate)) && all(gate>=0 & gate<=1));
extra=typecast(double(gate),'uint8');bytes=[base,reshape(extra,1,[])];
received=reshape(typecast(bytes(numel(base)+1:end),'double'),1,[]);
assert(isequal(gate,received));
for k=1:numel(decoded),decoded(k).positiveInnovationSupport=received(k);end
assert(numel(bytes)==32+224*numel(objects));
end
