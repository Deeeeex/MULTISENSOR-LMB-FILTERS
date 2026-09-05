function objects = initializeLmbObservationLineageV284(objects)
% Add a false birth-prior flag without changing the Bernoulli/GM parameters.
if isfield(objects, 'hasObservationLineage'), return; end
if isempty(objects)
    names = [fieldnames(objects); {'hasObservationLineage'}];
    template = cell2struct(cell(numel(names), 1), names, 1);
    objects = repmat(template, size(objects));
else
    [objects.hasObservationLineage] = deal(false);
end
end
