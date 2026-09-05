function objects = markLmbObservationLineageV284( ...
        objects, predicted, model, sensorIdx, currentTime, isScheduled)
% An opportunity includes both detection and missed-detection outcomes.
% Use the same component-mean p_D approximation as the existing local update.
if ~isScheduled, return; end
assert(numel(objects) == numel(predicted));
for k = 1:numel(objects)
    assert(objects(k).birthTime == predicted(k).birthTime && ...
        objects(k).birthLocation == predicted(k).birthLocation);
    if objects(k).hasObservationLineage, continue; end
    pD = 0;
    for c = 1:predicted(k).numberOfGmComponents
        if predicted(k).w(c) <= 0, continue; end
        value = evaluateSensorQuality( ...
            model, sensorIdx, predicted(k).mu{c}, currentTime);
        pD = pD + predicted(k).w(c) * value;
    end
    objects(k).hasObservationLineage = pD > 0;
end
end
