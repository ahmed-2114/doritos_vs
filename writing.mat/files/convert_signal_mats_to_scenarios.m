function convert_signal_mats_to_scenarios(folder_path)
% CONVERT_SIGNAL_MATS_TO_SCENARIOS Convert raw signal MAT files in-place
% into Signal Editor compatible files containing a `scenario` dataset.

if nargin < 1 || strlength(string(folder_path)) == 0
    folder_path = fileparts(mfilename('fullpath'));
end

mat_files = dir(fullfile(folder_path, '*_signal.mat'));
if isempty(mat_files)
    error('No *_signal.mat files found in %s.', folder_path);
end

for idx = 1:numel(mat_files)
    file_path = fullfile(mat_files(idx).folder, mat_files(idx).name);
    raw = load(file_path);

    [time, x, y, z] = extract_xyz_signals(raw, file_path);
    raw = set_first_available_field(raw, {'z', 'Z'}, z);

    scenario = Simulink.SimulationData.Dataset;
    scenario = scenario.addElement(timeseries(x, time), 'x');
    scenario = scenario.addElement(timeseries(y, time), 'y');
    scenario = scenario.addElement(timeseries(z, time), 'z');

    save(file_path, '-struct', 'raw');
    save(file_path, 'scenario', '-append');

    fprintf('Converted %s\n', file_path);
end
end

function [time, x, y, z] = extract_xyz_signals(raw, file_path)
z_plane = -320;

time = get_first_available_field(raw, {'time', 'Time', 't', 'T'}, file_path);
x = get_first_available_field(raw, {'x', 'X'}, file_path);
y = get_first_available_field(raw, {'y', 'Y'}, file_path);
z = get_first_available_field(raw, {'z', 'Z'}, file_path);

time = ensure_column_vector(time, 'time', file_path);
x = ensure_column_vector(x, 'x', file_path);
y = ensure_column_vector(y, 'y', file_path);
z = ensure_column_vector(z, 'z', file_path);
z = z_plane * ones(size(z));

sample_count = numel(time);
if any([numel(x), numel(y), numel(z)] ~= sample_count)
    error('Signal length mismatch in %s.', file_path);
end
end

function value = get_first_available_field(raw, candidate_names, file_path)
value = [];
for idx = 1:numel(candidate_names)
    field_name = candidate_names{idx};
    if isfield(raw, field_name)
        value = raw.(field_name);
        return;
    end
end

error('Missing one of [%s] in %s.', strjoin(candidate_names, ', '), file_path);
end

function raw = set_first_available_field(raw, candidate_names, value)
for idx = 1:numel(candidate_names)
    field_name = candidate_names{idx};
    if isfield(raw, field_name)
        raw.(field_name) = value;
        return;
    end
end
end

function value = ensure_column_vector(value, signal_name, file_path)
if ~isnumeric(value) || ~isvector(value)
    error('Field %s in %s must be a numeric vector.', signal_name, file_path);
end

value = double(value(:));
end