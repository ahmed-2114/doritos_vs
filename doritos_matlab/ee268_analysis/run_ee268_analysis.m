analysis_root = fileparts(mfilename('fullpath'));
project_root = fileparts(analysis_root);
addpath(project_root);

cfg = struct();
cfg.res_xyz = [50, 50, 50];
cfg.marker_size = 15;
cfg.save_outputs = true;
cfg.keep_figures_open = true;
cfg.output_dir = analysis_root;
cfg.manip_threshold_ratio = 0.30;
cfg.clearance_threshold_ratio = 0.25;

context = build_ee268_context();
results = sample_ee268_workspace(context, cfg);
save_outputs(results, cfg.output_dir);
results.figures = plot_outputs(results, cfg.output_dir, cfg.marker_size, cfg.keep_figures_open);
print_summary(results);

assignin('base', 'ee268_context', context);
assignin('base', 'ee268_results', results);

function context = build_ee268_context()
params = delta_params();

[platform_home, fk_ok, fk_msg] = delta_FK(params, deg2rad([0; 0; 0]));
if ~fk_ok
    error('Failed to compute home pose from FK: %s', fk_msg);
end

ee_home = [0; 0; -0.2680];
tool_offset_base = ee_home - platform_home;

ee_ws_lim = params.ws_lim;
ee_ws_lim(:, 1) = params.ws_lim(:, 1) + tool_offset_base;
ee_ws_lim(:, 2) = params.ws_lim(:, 2) + tool_offset_base;

context = struct();
context.params = params;
context.platform_home = platform_home;
context.ee_home = ee_home;
context.tool_offset_base = tool_offset_base;
context.ee_distance_from_base = norm(ee_home);
context.ee_ws_lim = ee_ws_lim;
context.assumption = ['EE point is modeled as a fixed point in the base frame ', ...
    'offset from the platform FK/IK reference by tool_offset_base.'];
end

function results = sample_ee268_workspace(context, cfg)
lim = context.ee_ws_lim;
xv = linspace(lim(1,1), lim(1,2), cfg.res_xyz(1));
yv = linspace(lim(2,1), lim(2,2), cfg.res_xyz(2));
zv = linspace(lim(3,1), lim(3,2), cfg.res_xyz(3));

[X, Y, Z] = meshgrid(xv, yv, zv);
ee_points = [X(:), Y(:), Z(:)];
num_points = size(ee_points, 1);

reachable_mask = false(num_points, 1);
theta_all = nan(num_points, 3);
platform_points = nan(num_points, 3);
manip = nan(num_points, 1);
inv_cond = nan(num_points, 1);
det_jx = nan(num_points, 1);
joint_margin = nan(num_points, 1);
type1 = false(num_points, 1);
type2 = false(num_points, 1);

fprintf('[ee268] Sampling %d EE candidate points (%dx%dx%d)...\n', ...
    num_points, cfg.res_xyz(1), cfg.res_xyz(2), cfg.res_xyz(3));

t_start = tic;
for idx = 1:num_points
    p_ee = ee_points(idx, :)';
    p_platform = p_ee - context.tool_offset_base;
    platform_points(idx, :) = p_platform';

    [theta, ok, ~] = delta_IK(context.params, p_platform);
    if ~ok
        continue
    end

    reachable_mask(idx) = true;
    theta_all(idx, :) = theta';

    evalc('[Jx, Jq, is_type1, is_type2, info] = delta_singularity(context.params, theta);');
    cart_jac = Jx \ Jq;
    manip(idx) = prod(svd(cart_jac));
    inv_cond(idx) = 1 / max(info.cond_Jx, eps);
    det_jx(idx) = info.det_Jx;
    joint_margin(idx) = min(abs(diag(Jq)));
    type1(idx) = is_type1;
    type2(idx) = is_type2;
end
elapsed = toc(t_start);

reach_ee = ee_points(reachable_mask, :);
reach_platform = platform_points(reachable_mask, :);
reach_theta = theta_all(reachable_mask, :);
reach_manip = manip(reachable_mask);
reach_inv_cond = inv_cond(reachable_mask);
reach_det_jx = det_jx(reachable_mask);
reach_joint_margin = joint_margin(reachable_mask);
reach_type1 = type1(reachable_mask);
reach_type2 = type2(reachable_mask);

manip_threshold = cfg.manip_threshold_ratio * max(reach_manip);
clearance_threshold = cfg.clearance_threshold_ratio * max(reach_inv_cond);

manip_keep = reach_manip >= manip_threshold;
clearance_keep = reach_inv_cond >= clearance_threshold;

results = struct();
results.context = context;
results.cfg = cfg;
results.elapsed_s = elapsed;
results.ee_points = ee_points;
results.platform_points = platform_points;
results.reachable_mask = reachable_mask;
results.reach_ee = reach_ee;
results.reach_platform = reach_platform;
results.reach_theta = reach_theta;
results.manip = reach_manip;
results.inv_cond = reach_inv_cond;
results.det_jx = reach_det_jx;
results.joint_margin = reach_joint_margin;
results.type1 = reach_type1;
results.type2 = reach_type2;
results.manip_threshold = manip_threshold;
results.clearance_threshold = clearance_threshold;
results.manip_keep = manip_keep;
results.clearance_keep = clearance_keep;
results.summary = build_summary(reach_ee, reach_platform, reach_manip, reach_inv_cond, ...
    reach_joint_margin, reach_type1, reach_type2, elapsed);
end

function summary = build_summary(reach_ee, reach_platform, reach_manip, reach_inv_cond, reach_joint_margin, reach_type1, reach_type2, elapsed)
summary = struct();
summary.num_reachable = size(reach_ee, 1);
summary.ee_bounds = [min(reach_ee(:,1)), max(reach_ee(:,1));
                     min(reach_ee(:,2)), max(reach_ee(:,2));
                     min(reach_ee(:,3)), max(reach_ee(:,3))];
summary.platform_bounds = [min(reach_platform(:,1)), max(reach_platform(:,1));
                           min(reach_platform(:,2)), max(reach_platform(:,2));
                           min(reach_platform(:,3)), max(reach_platform(:,3))];
summary.manip_range = [min(reach_manip), max(reach_manip)];
summary.inv_cond_range = [min(reach_inv_cond), max(reach_inv_cond)];
summary.joint_margin_range = [min(reach_joint_margin), max(reach_joint_margin)];
summary.num_type1 = sum(reach_type1);
summary.num_type2 = sum(reach_type2);
summary.elapsed_s = elapsed;
end

function save_outputs(results, output_dir)
results_file = fullfile(output_dir, 'ee268_analysis_results.mat');
context_file = fullfile(output_dir, 'ee268_context.mat');
context = results.context;
save(results_file, 'results');
save(context_file, 'context');
end

function figures = plot_outputs(results, output_dir, marker_size, keep_figures_open)
figures = struct();
figures.workspace = plot_workspace_cloud(results, output_dir, marker_size, keep_figures_open);
figures.manipulability = plot_manipulability_cloud(results, output_dir, marker_size, keep_figures_open);
figures.singularity = plot_singularity_cloud(results, output_dir, marker_size, keep_figures_open);
figures.relationship = plot_relationship_cloud(results, output_dir, marker_size, keep_figures_open);
end

function fig = plot_workspace_cloud(results, output_dir, marker_size, keep_figures_open)
fig = figure('Name', 'EE268 Reachable Workspace', 'Color', 'w', 'Position', [80 80 980 720]);
scatter3(results.reach_ee(:,1) * 1e3, results.reach_ee(:,2) * 1e3, results.reach_ee(:,3) * 1e3, ...
    marker_size, results.manip, 'filled', 'MarkerFaceAlpha', 0.85, 'MarkerEdgeAlpha', 0.12);
format_cloud_axes(results, 'EE Reachable Workspace Cloud', ...
    sprintf('Colored by manipulability | range %.4f to %.4f', results.summary.manip_range(1), results.summary.manip_range(2)));
cb = colorbar;
cb.Label.String = sprintf('Manipulability | higher is better | threshold %.4f', results.manip_threshold);
set_fixed_colorbar_ticks(cb, min(results.manip), max(results.manip), '%.4f');
saveas(fig, fullfile(output_dir, 'ee268_workspace_cloud.png'));
if ~keep_figures_open
    close(fig);
end
end

function fig = plot_manipulability_cloud(results, output_dir, marker_size, keep_figures_open)
fig = figure('Name', 'EE268 Manipulability Space', 'Color', 'w', 'Position', [90 90 980 720]);
pts = results.reach_ee(results.manip_keep, :);
vals = results.manip(results.manip_keep);
scatter3(pts(:,1) * 1e3, pts(:,2) * 1e3, pts(:,3) * 1e3, ...
    marker_size, vals, 'filled', 'MarkerFaceAlpha', 0.90, 'MarkerEdgeAlpha', 0.15);
format_cloud_axes(results, sprintf('EE Manipulability Space (threshold = %.4f)', results.manip_threshold), ...
    sprintf('Only points with manipulability >= %.4f are shown', results.manip_threshold));
cb = colorbar;
cb.Label.String = sprintf('Manipulability | higher is better | shown >= %.4f', results.manip_threshold);
set_fixed_colorbar_ticks(cb, min(vals), max(vals), '%.4f');
saveas(fig, fullfile(output_dir, 'ee268_manipulability_cloud.png'));
if ~keep_figures_open
    close(fig);
end
end

function fig = plot_singularity_cloud(results, output_dir, marker_size, keep_figures_open)
fig = figure('Name', 'EE268 Singularity Clearance', 'Color', 'w', 'Position', [100 100 980 720]);
scatter3(results.reach_ee(:,1) * 1e3, results.reach_ee(:,2) * 1e3, results.reach_ee(:,3) * 1e3, ...
    marker_size, results.inv_cond, 'filled', 'MarkerFaceAlpha', 0.90, 'MarkerEdgeAlpha', 0.15);
format_cloud_axes(results, sprintf('EE Singularity Clearance (threshold = %.4f)', results.clearance_threshold), ...
    'Colored by 1 / cond(Jx) | higher means farther from singularity');
cb = colorbar;
cb.Label.String = sprintf('1 / cond(Jx) | higher is better | threshold %.4f', results.clearance_threshold);
set_fixed_colorbar_ticks(cb, min(results.inv_cond), max(results.inv_cond), '%.4f');
saveas(fig, fullfile(output_dir, 'ee268_singularity_cloud.png'));
if ~keep_figures_open
    close(fig);
end
end

function fig = plot_relationship_cloud(results, output_dir, marker_size, keep_figures_open)
fig = figure('Name', 'EE268 Relationship View', 'Color', 'w', 'Position', [110 110 1120 760]);
ax = axes(fig, 'Position', [0.08 0.12 0.72 0.80]);
hold(ax, 'on');

reachable_pts = results.reach_ee;
manip_pts = results.reach_ee(results.manip_keep, :);
clear_pts = results.reach_ee(results.clearance_keep, :);
both_keep = results.manip_keep & results.clearance_keep;
both_pts = results.reach_ee(both_keep, :);

reachable_color = [0.72 0.60 0.92];
manip_color = [0.10 0.45 0.90];
clear_color = [0.95 0.50 0.10];
both_color = [0.10 0.70 0.25];

h_reach = scatter3(ax, reachable_pts(:,1) * 1e3, reachable_pts(:,2) * 1e3, reachable_pts(:,3) * 1e3, ...
    max(8, marker_size - 4), reachable_color, 'filled', 'MarkerFaceAlpha', 0.45, 'MarkerEdgeAlpha', 0.06, ...
    'DisplayName', 'Reachable cloud');
h_manip = scatter3(ax, manip_pts(:,1) * 1e3, manip_pts(:,2) * 1e3, manip_pts(:,3) * 1e3, ...
    marker_size, manip_color, 'filled', 'MarkerFaceAlpha', 0.65, 'MarkerEdgeAlpha', 0.10, ...
    'DisplayName', 'Manipulability-passing');
h_clear = scatter3(ax, clear_pts(:,1) * 1e3, clear_pts(:,2) * 1e3, clear_pts(:,3) * 1e3, ...
    marker_size, clear_color, 'filled', 'MarkerFaceAlpha', 0.45, 'MarkerEdgeAlpha', 0.10, ...
    'DisplayName', 'Singularity-safe');
h_both = scatter3(ax, both_pts(:,1) * 1e3, both_pts(:,2) * 1e3, both_pts(:,3) * 1e3, ...
    marker_size + 4, both_color, 'filled', 'MarkerFaceAlpha', 0.90, 'MarkerEdgeAlpha', 0.15, ...
    'DisplayName', 'Passes both');

h_frame = plot3(ax, nan, nan, nan, 'k--', 'LineWidth', 0.8, 'DisplayName', 'Base-frame axes (X, Y, Z)');
h_base = scatter3(ax, nan, nan, nan, 90, 'k', 'filled', 'd', 'DisplayName', 'Base origin');
h_home = scatter3(ax, nan, nan, nan, 80, [0.85 0.15 0.15], 'filled', 'DisplayName', 'EE home point');

format_cloud_axes(results, 'Relationship Between Reachable / Manipulability / Singularity Spaces', ...
    sprintf('Purple=all reachable | Blue=manip >= %.4f | Orange=1/cond(Jx) >= %.4f | Green=both', ...
    results.manip_threshold, results.clearance_threshold));
legend(ax, [h_reach, h_manip, h_clear, h_both, h_frame, h_base, h_home], 'Location', 'northeastoutside');

uicontrol(fig, 'Style', 'checkbox', 'String', 'Reachable cloud', 'Value', 1, ...
    'Units', 'normalized', 'Position', [0.83 0.78 0.15 0.05], ...
    'Callback', @(src,~) set(h_reach, 'Visible', on_off(src.Value)));
uicontrol(fig, 'Style', 'checkbox', 'String', 'Manipulability-passing', 'Value', 1, ...
    'Units', 'normalized', 'Position', [0.83 0.71 0.15 0.05], ...
    'Callback', @(src,~) set(h_manip, 'Visible', on_off(src.Value)));
uicontrol(fig, 'Style', 'checkbox', 'String', 'Singularity-safe', 'Value', 1, ...
    'Units', 'normalized', 'Position', [0.83 0.64 0.15 0.05], ...
    'Callback', @(src,~) set(h_clear, 'Visible', on_off(src.Value)));
uicontrol(fig, 'Style', 'checkbox', 'String', 'Passes both', 'Value', 1, ...
    'Units', 'normalized', 'Position', [0.83 0.57 0.15 0.05], ...
    'Callback', @(src,~) set(h_both, 'Visible', on_off(src.Value)));

annotation(fig, 'textbox', [0.82 0.20 0.16 0.23], 'String', {
    'How to read this figure:', ...
    '1. Purple = everything IK can reach', ...
    '2. Blue = good manipulability', ...
    '3. Orange = safely away from singularity', ...
    '4. Green = overlap of both filters', ...
    '5. Dashed lines = base-frame axes', ...
    '6. Diamond = base origin'}, ...
    'FitBoxToText', 'on', 'EdgeColor', [0.8 0.8 0.8], 'BackgroundColor', [1 1 1]);

saveas(fig, fullfile(output_dir, 'ee268_relationship_cloud.png'));
if ~keep_figures_open
    close(fig);
end
end

function format_cloud_axes(results, title_text, subtitle_text)
colormap(parula(256));
grid on;
axis equal;
view(45, 25);
xlabel('X [mm]', 'FontSize', 11);
ylabel('Y [mm]', 'FontSize', 11);
zlabel('Z [mm]', 'FontSize', 11);
title({title_text, subtitle_text}, 'FontSize', 13);
ax = gca;
ax.Box = 'on';
ax.GridAlpha = 0.18;
ax.MinorGridAlpha = 0.08;
ax.LineWidth = 0.8;
ax.FontSize = 10;
apply_robot_reference_frame(ax, results);
set_normal_axes(ax);
end

function apply_robot_reference_frame(ax, results)
hold(ax, 'on');

pts_mm = results.reach_ee * 1e3;
base_mm = [0, 0, 0];
all_pts = [pts_mm; base_mm];

xy_abs = max(abs(all_pts(:,1:2)), [], 'all');
xy_abs = max(xy_abs, 50);
xy_margin = 0.08 * xy_abs;
xy_lim = [-1, 1] * (xy_abs + xy_margin);

z_min = min(all_pts(:,3));
z_max = max(all_pts(:,3));
z_margin = max(20, 0.08 * (z_max - z_min + eps));
z_lim = [min(z_min - z_margin, -20), max(0, z_max + z_margin)];

xlim(ax, xy_lim);
ylim(ax, xy_lim);
zlim(ax, z_lim);

hx = plot3(ax, [xy_lim(1), xy_lim(2)], [0, 0], [0, 0], 'k--', 'LineWidth', 0.8);
hy = plot3(ax, [0, 0], [xy_lim(1), xy_lim(2)], [0, 0], 'k--', 'LineWidth', 0.8);
hz = plot3(ax, [0, 0], [0, 0], [z_lim(1), z_lim(2)], 'k--', 'LineWidth', 0.8);
hbase = scatter3(ax, 0, 0, 0, 90, 'k', 'filled', 'd');
text(ax, 12, 12, 12, 'Base {0,0,0}', 'FontSize', 9, 'Color', 'k');

ee_home_mm = results.context.ee_home(:)' * 1e3;
heehome = scatter3(ax, ee_home_mm(1), ee_home_mm(2), ee_home_mm(3), 80, [0.85 0.15 0.15], 'filled');
text(ax, ee_home_mm(1) + 8, ee_home_mm(2) + 8, ee_home_mm(3), 'EE home', 'FontSize', 9, 'Color', [0.75 0.1 0.1]);

set(get(get(hx, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
set(get(get(hy, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
set(get(get(hz, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
set(get(get(hbase, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
set(get(get(heehome, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');

hold(ax, 'off');
end

function set_normal_axes(ax)
ax.XAxis.Exponent = 0;
ax.YAxis.Exponent = 0;
ax.ZAxis.Exponent = 0;
xtickformat(ax, '%.0f');
ytickformat(ax, '%.0f');
ztickformat(ax, '%.0f');
end

function set_fixed_colorbar_ticks(cb, min_val, max_val, fmt)
if ~isfinite(min_val) || ~isfinite(max_val) || min_val == max_val
    cb.Ticks = min_val;
    cb.TickLabels = {sprintf(fmt, min_val)};
    return
end

ticks = linspace(min_val, max_val, 5);
cb.Ticks = ticks;
cb.TickLabels = arrayfun(@(v) sprintf(fmt, v), ticks, 'UniformOutput', false);
end

function state = on_off(value)
if value
    state = 'on';
else
    state = 'off';
end
end

function print_summary(results)
fprintf('\n=== EE268 Analysis Summary ===\n');
fprintf('Platform home point from FK   : [%.4f, %.4f, %.4f] m\n', results.context.platform_home);
fprintf('Assumed physical EE home      : [%.4f, %.4f, %.4f] m\n', results.context.ee_home);
fprintf('Applied EE tool offset        : [%.4f, %.4f, %.4f] m\n', results.context.tool_offset_base);
fprintf('Reachable EE points           : %d\n', results.summary.num_reachable);
fprintf('EE bounds X [m]               : [%.4f, %.4f]\n', results.summary.ee_bounds(1,1), results.summary.ee_bounds(1,2));
fprintf('EE bounds Y [m]               : [%.4f, %.4f]\n', results.summary.ee_bounds(2,1), results.summary.ee_bounds(2,2));
fprintf('EE bounds Z [m]               : [%.4f, %.4f]\n', results.summary.ee_bounds(3,1), results.summary.ee_bounds(3,2));
fprintf('Manipulability range          : [%.4g, %.4g]\n', results.summary.manip_range(1), results.summary.manip_range(2));
fprintf('1/cond(Jx) range              : [%.4g, %.4g]\n', results.summary.inv_cond_range(1), results.summary.inv_cond_range(2));
fprintf('Manipulability threshold      : %.4g\n', results.manip_threshold);
fprintf('Singularity clearance thresh. : %.4g\n', results.clearance_threshold);
fprintf('Type-1 points                 : %d\n', results.summary.num_type1);
fprintf('Type-2 points                 : %d\n', results.summary.num_type2);
fprintf('Sampling time                 : %.2f s\n', results.summary.elapsed_s);
fprintf('Saved folder                  : %s\n', fileparts(mfilename('fullpath')));
end