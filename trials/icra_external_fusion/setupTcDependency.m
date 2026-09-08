function directory=setupTcDependency()
% Pinned research-only author code; do not add its local-filter directories.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
directory=fullfile(root,'tmp','external_baselines','Distributed-limitedFoV-MOT');
assert(isfile(fullfile(directory,'data_fusion','fusion_main_tc.m')), ...
    'Download the pinned author repository using fetch_dependencies.py.');
[status,revision]=system(sprintf('git -C "%s" rev-parse HEAD',directory));
assert(status==0 && strcmp(strtrim(revision),'b6b20ec30b7854dcee6f4a718237d82d96ac7c2a'));
[status,dirty]=system(sprintf('git -C "%s" status --porcelain',directory));
assert(status==0 && isempty(strtrim(dirty)),'Author source must be unmodified.');
addpath(fullfile(directory,'data_fusion'),fullfile(directory,'track_matching'), ...
    fullfile(directory,'misc'),fullfile(directory,'misc','_common'));
end
