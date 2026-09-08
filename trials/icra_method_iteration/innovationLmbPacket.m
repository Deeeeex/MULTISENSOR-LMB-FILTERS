function [decoded,bytes]=innovationLmbPacket(objects,model,source,t)
% Extend the frozen packet by exactly one current local evidence scalar.
[decoded,base]=gaussianLmbPacket(objects,model,source,t);
increment=reshape([objects.localLogOddsIncrement],1,[]);extra=typecast(double(increment),'uint8');
bytes=[base,reshape(extra,1,[])];decodedIncrement=reshape(typecast(bytes(numel(base)+1:end),'double'),1,[]);
assert(isequal(increment,decodedIncrement));
for k=1:numel(decoded),decoded(k).localLogOddsIncrement=decodedIncrement(k);end
assert(numel(bytes)==32+216*numel(objects));
end
