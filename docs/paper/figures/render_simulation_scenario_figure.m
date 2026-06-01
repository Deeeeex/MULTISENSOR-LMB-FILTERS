function render_simulation_scenario_figure()
% Compatibility wrapper for the Python-rendered scenario schematic.
%
% The paper asset is now drawn with Matplotlib so the PDF has the same
% physical width as the manuscript text block and does not shrink labels
% during LaTeX inclusion.

script_dir = fileparts(mfilename('fullpath'));
python_script = fullfile(script_dir, 'render_simulation_scenario_figure.py');
python_exe = getenv('PYTHON');
if isempty(python_exe)
    python_exe = 'python3';
end

cmd = sprintf('"%s" "%s"', python_exe, python_script);
[status, output] = system(cmd);
fprintf('%s', output);
if status ~= 0
    error('render_simulation_scenario_figure:python_failed', ...
        'Python scenario renderer failed with status %d.', status);
end
end
