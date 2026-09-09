function [minimumQuality,cutoffs]=associationScreenSettings(mode)
assert(ismember(mode,{'quality','nis','quality_nis'}));
minimumQuality=.5;
cutoffs=[9.21034037197618,13.2767041359876,16.8118938297709];
if ismember(mode,{'quality','quality_nis'}),minimumQuality=.9;end
if ismember(mode,{'nis','quality_nis'})
    cutoffs=[13.815510557964274,18.46682695290317,22.457744484825326];
end
end
