function [decoded,bytes]=asymmetricInnovationPacket(objects,model,source,t)
% Current delta, positive branch support, and negative branch support.
[decoded,base]=selectiveInnovationPacket(objects,model,source,t);
gate=reshape([objects.negativeInnovationSupport],1,[]);
assert(all(isfinite(gate)) && all(gate>=0 & gate<=1));
extra=typecast(double(gate),'uint8');bytes=[base,reshape(extra,1,[])];
received=reshape(typecast(bytes(numel(base)+1:end),'double'),1,[]);
assert(isequal(gate,received));
for k=1:numel(decoded),decoded(k).negativeInnovationSupport=received(k);end
assert(numel(bytes)==32+232*numel(objects));
end
