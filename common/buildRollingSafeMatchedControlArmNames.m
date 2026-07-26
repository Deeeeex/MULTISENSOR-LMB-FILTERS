function controls = buildRollingSafeMatchedControlArmNames( ...
        groupCount, sourceWeight)
% BUILDROLLINGSAFEMATCHEDCONTROLARMNAMES Complete rolling-B3 controls.
%
% The scheduled family contains every non-redundant constant-quota cyclic
% chunk and every root/direction/temporal phase of the sparse
% tree/edge/empty burst. A rolling-constrained link-aware arm is added as
% the strongest truth- and posterior-free online control.

if nargin < 2 || isempty(sourceWeight)
    sourceWeight = 0.70;
end
groupCount = round(groupCount);
if groupCount < 2
    error('Matched rolling controls need at least two formations.');
end
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('Matched rolling-control weight must lie in (0,1).');
end
weightToken = round(100 * sourceWeight);
if abs(weightToken / 100 - sourceWeight) > 1e-12
    error([ ...
        'Matched rolling-control weights must be exactly representable ', ...
        'as an integer percentage.']);
end
directions = {'cw', 'ccw'};
chunkNames = {};
chunkMetadata = repmat(struct( ...
    'quota', NaN, 'direction', '', 'formationPhase', NaN), 0, 1);
for quota = ceil(groupCount / 3):(groupCount - 1)
    phaseCount = gcd(groupCount, quota);
    for directionIdx = 1:numel(directions)
        for formationPhase = 0:(phaseCount - 1)
            chunkNames{end + 1} = sprintf( ... %#ok<AGROW>
                'directed-rolling-chunk-q%d-d%s-c%d-w%d', ...
                quota, directions{directionIdx}, ...
                formationPhase, weightToken);
            chunkMetadata(end + 1, 1) = struct( ... %#ok<AGROW>
                'quota', quota, ...
                'direction', directions{directionIdx}, ...
                'formationPhase', formationPhase);
        end
    end
end

burstNames = {};
burstMetadata = repmat(struct( ...
    'rootFormation', NaN, 'direction', '', ...
    'temporalPhase', NaN), 0, 1);
for rootFormation = 1:groupCount
    for directionIdx = 1:numel(directions)
        for temporalPhase = 0:2
            burstNames{end + 1} = sprintf( ... %#ok<AGROW>
                'directed-rolling-burst-r%d-d%s-p%d-w%d', ...
                rootFormation, directions{directionIdx}, ...
                temporalPhase, weightToken);
            burstMetadata(end + 1, 1) = struct( ... %#ok<AGROW>
                'rootFormation', rootFormation, ...
                'direction', directions{directionIdx}, ...
                'temporalPhase', temporalPhase);
        end
    end
end

linkAwareName = sprintf( ...
    'directed-link-aware-rolling-b3-w%d', weightToken);
controls = struct();
controls.groupCount = groupCount;
controls.sourceWeight = sourceWeight;
controls.chunkArmNames = chunkNames;
controls.chunkMetadata = chunkMetadata;
controls.burstArmNames = burstNames;
controls.burstMetadata = burstMetadata;
controls.linkAwareArmName = linkAwareName;
controls.scheduledArmNames = [chunkNames, burstNames];
controls.allArmNames = [ ...
    controls.scheduledArmNames, {linkAwareName}];
controls.chunkArmCount = numel(chunkNames);
controls.burstArmCount = numel(burstNames);
controls.scheduledArmCount = ...
    numel(controls.scheduledArmNames);
controls.totalArmCount = numel(controls.allArmNames);
controls.contractVersion = ...
    'rolling-b3-cycle-backbone-matched-controls-v1';
end
