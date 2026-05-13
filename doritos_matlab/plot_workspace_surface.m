function workspace_data = plot_workspace_surface(varargin)
% PLOT_WORKSPACE_SURFACE  Plot a volumetric dotted reachable-workspace cloud.
% Reuses delta_params(), delta_IK(), and delta_singularity() when available.

params = delta_params();
res_xyz = [31, 31, 31];
metric_mode = 'inv_cond';
quality_threshold = [];
marker_size = 18;
save_outputs = true;
output_dir = fileparts(mfilename('fullpath'));

k = 1;
while k <= numel(varargin)
    key = lower(varargin{k});
    switch key
        case 'res'
            value = varargin{k+1};
            if isscalar(value)
                res_xyz = [value, value, value];
            else
                res_xyz = value;
            end
            k = k + 2;
        case 'res_x'
            res_xyz(1) = varargin{k+1};
            k = k + 2;
        case 'res_y'
            res_xyz(2) = varargin{k+1};
            k = k + 2;
        case 'res_z'
            res_xyz(3) = varargin{k+1};
            k = k + 2;
        case 'metric'
            metric_mode = lower(char(string(varargin{k+1})));
            k = k + 2;
        case 'threshold'
            quality_threshold = varargin{k+1};
            k = k + 2;
        case 'marker_size'
            marker_size = varargin{k+1};
            k = k + 2;
        case 'save'
            save_outputs = varargin{k+1};
            k = k + 2;
        case 'output_dir'
            output_dir = varargin{k+1};
            k = k + 2;
        otherwise
            warning('plot_workspace_surface: unknown option "%s", skipping.', key);
            k = k + 1;
    end
end

lim = params.ws_lim;
xv = linspace(lim(1,1), lim(1,2), res_xyz(1));
yv = linspace(lim(2,1), lim(2,2), res_xyz(2));
zv = linspace(lim(3,1), lim(3,2), res_xyz(3));

[X, Y, Z] = meshgrid(xv, yv, zv);
candidate_points = [X(:), Y(:), Z(:)];
num_points = size(candidate_points, 1);

valid_mask = false(num_points, 1);
theta_valid = nan(num_points, 3);
quality_values = nan(num_points, 1);
quality_available = false;

has_singularity_fn = exist('delta_singularity', 'file') == 2;

fprintf('[plot_workspace_surface] Sampling %d candidate points (%dx%dx%d)...\n', ...
        num_points, res_xyz(1), res_xyz(2), res_xyz(3));

t_start = tic;
for idx = 1:num_points
    p = candidate_points(idx, :)';
    [theta, valid, ~] = delta_IK(params, p);
    if ~valid
        continue
    end

    valid_mask(idx) = true;
    theta_valid(idx, :) = theta';

    if has_singularity_fn
        try
            evalc('[Jx, Jq, ~, ~, info] = delta_singularity(params, theta);');
            switch metric_mode
                case 'manip'
                    cart_jac = Jx \ Jq;
                    quality_values(idx) = prod(svd(cart_jac));
                case 'inv_cond'
                    quality_values(idx) = 1 / max(info.cond_Jx, eps);
                case 'cond_score'
                    quality_values(idx) = 1 / (1 + max(info.cond_Jx, 0));
                otherwise
                    error('Unsupported metric mode: %s', metric_mode);
            end
            quality_available = true;
        catch
            % Leave quality unavailable and fall back to pure reachable cloud.
        end
    end
end
elapsed = toc(t_start);

valid_points = candidate_points(valid_mask, :);
valid_quality = quality_values(valid_mask);

if quality_available && any(isfinite(valid_quality))
    finite_quality = valid_quality(isfinite(valid_quality));
    if isempty(quality_threshold)
        quality_threshold = 0.15 * max(finite_quality);
    end
    keep_mask_valid = isfinite(valid_quality) & (valid_quality >= quality_threshold);
else
    quality_available = false;
    quality_threshold = NaN;
    keep_mask_valid = true(size(valid_points, 1), 1);
    valid_quality = valid_points(:, 3);
end

filtered_points = valid_points(keep_mask_valid, :);
filtered_quality = valid_quality(keep_mask_valid);

workspace_data = struct();
workspace_data.params = params;
workspace_data.res_xyz = res_xyz;
workspace_data.metric_mode = metric_mode;
workspace_data.quality_available = quality_available;
workspace_data.quality_threshold = quality_threshold;
workspace_data.candidate_points = candidate_points;
workspace_data.valid_mask = valid_mask;
workspace_data.valid_points = valid_points;
workspace_data.theta_valid = theta_valid(valid_mask, :);
workspace_data.quality_values = valid_quality;
workspace_data.filtered_points = filtered_points;
workspace_data.filtered_quality = filtered_quality;
workspace_data.elapsed_s = elapsed;

fig = figure('Name', 'Delta Workspace Point Cloud', ...
             'Color', 'w', 'Position', [100, 80, 980, 720]);

scatter3(filtered_points(:, 1) * 1e3, filtered_points(:, 2) * 1e3, filtered_points(:, 3) * 1e3, ...
    marker_size, filtered_quality, 'filled', 'MarkerFaceAlpha', 0.85, 'MarkerEdgeAlpha', 0.15);

colormap(parula(256));
cb = colorbar;
if quality_available
    cb.Label.String = sprintf('%s quality', format_metric_name(metric_mode));
else
    cb.Label.String = 'Z [m]';
end

grid on;
axis equal;
view(45, 25);
xlabel('X [mm]', 'FontSize', 11);
ylabel('Y [mm]', 'FontSize', 11);
zlabel('Z [mm]', 'FontSize', 11);

if quality_available
    title(sprintf('Usable Workspace Cloud (%s >= %.4g)', ...
        format_metric_name(metric_mode), quality_threshold), 'FontSize', 13);
else
    title('Reachable Workspace Cloud (IK-valid points)', 'FontSize', 13);
end

ax = gca;
ax.Box = 'on';
ax.GridAlpha = 0.18;
ax.MinorGridAlpha = 0.08;
ax.LineWidth = 0.8;
ax.FontSize = 10;

fprintf('[plot_workspace_surface] IK-valid points: %d / %d\n', size(valid_points, 1), num_points);
fprintf('[plot_workspace_surface] Filtered points: %d\n', size(filtered_points, 1));
fprintf('[plot_workspace_surface] Sampling time: %.2f s\n', elapsed);
if quality_available
    fprintf('[plot_workspace_surface] Metric: %s, threshold: %.6g\n', ...
        format_metric_name(metric_mode), quality_threshold);
end

if save_outputs
    png_file = fullfile(output_dir, 'delta_workspace_surface.png');
    mat_file = fullfile(output_dir, 'delta_workspace_surface.mat');
    saveas(fig, png_file);
    workspace_data_to_save = workspace_data;
    save(mat_file, 'workspace_data_to_save');
    workspace_data.png_file = png_file;
    workspace_data.mat_file = mat_file;
    fprintf('[plot_workspace_surface] Figure saved: %s\n', png_file);
    fprintf('[plot_workspace_surface] MAT file saved: %s\n', mat_file);
end
end

function metric_name = format_metric_name(metric_mode)
switch metric_mode
    case 'manip'
        metric_name = 'manipulability';
    case 'inv_cond'
        metric_name = '1/cond(Jx)';
    case 'cond_score'
        metric_name = 'condition score';
    otherwise
        metric_name = char(metric_mode);
end
end