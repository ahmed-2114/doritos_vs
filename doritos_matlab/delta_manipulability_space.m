function analysis = delta_manipulability_space(params, varargin)
% DELTA_MANIPULABILITY_SPACE  Sample the reachable workspace and compute
% manipulability and singularity metrics at each reachable point.

res = 21;
do_plot = true;
save_outputs = true;
output_dir = fileparts(mfilename('fullpath'));

k = 1;
while k <= numel(varargin)
    key = lower(varargin{k});
    switch key
        case 'res'
            res = varargin{k+1};
            k = k + 2;
        case 'plot'
            do_plot = varargin{k+1};
            k = k + 2;
        case 'save'
            save_outputs = varargin{k+1};
            k = k + 2;
        case 'output_dir'
            output_dir = varargin{k+1};
            k = k + 2;
        otherwise
            warning('delta_manipulability_space: unknown option "%s", skipping.', key);
            k = k + 1;
    end
end

lim = params.ws_lim;
xv = linspace(lim(1,1), lim(1,2), res);
yv = linspace(lim(2,1), lim(2,2), res);
zv = linspace(lim(3,1), lim(3,2), res);

[X, Y, Z] = meshgrid(xv, yv, zv);
pts = [X(:), Y(:), Z(:)];
N = size(pts, 1);

reachable = false(N, 1);
theta_all = nan(N, 3);
cond_Jx = nan(N, 1);
inv_cond_Jx = nan(N, 1);
det_Jx = nan(N, 1);
min_abs_diag_Jq = nan(N, 1);
manip = nan(N, 1);
type1 = false(N, 1);
type2 = false(N, 1);

fprintf('[delta_manipulability_space] Sampling %d grid points (res=%d)...\n', N, res);
t_start = tic;

for idx = 1:N
    p = pts(idx, :)';
    [theta, ok, ~] = delta_IK(params, p);
    if ~ok
        continue
    end

    evalc('[Jx, Jq, is_type1, is_type2, info] = delta_singularity(params, theta);');

    reachable(idx) = true;
    theta_all(idx, :) = theta';
    cond_Jx(idx) = info.cond_Jx;
    inv_cond_Jx(idx) = 1 / max(info.cond_Jx, eps);
    det_Jx(idx) = info.det_Jx;
    min_abs_diag_Jq(idx) = min(abs(diag(Jq)));
    type1(idx) = is_type1;
    type2(idx) = is_type2;

    cart_jac = Jx \ Jq;
    sing_vals = svd(cart_jac);
    manip(idx) = prod(sing_vals);
end

elapsed = toc(t_start);
reach_pts = pts(reachable, :);

analysis = struct();
analysis.params = params;
analysis.res = res;
analysis.points = pts;
analysis.reachable_mask = reachable;
analysis.reachable_points = reach_pts;
analysis.theta = theta_all;
analysis.cond_Jx = cond_Jx;
analysis.inv_cond_Jx = inv_cond_Jx;
analysis.det_Jx = det_Jx;
analysis.min_abs_diag_Jq = min_abs_diag_Jq;
analysis.manipulability = manip;
analysis.type1 = type1;
analysis.type2 = type2;
analysis.elapsed_s = elapsed;
analysis.summary = build_summary(reach_pts, reachable, manip, cond_Jx, type1, type2, min_abs_diag_Jq);

fprintf('[delta_manipulability_space] Reachable points: %d / %d\n', ...
        analysis.summary.num_reachable, N);
fprintf('[delta_manipulability_space] Manipulability range: [%.4g, %.4g]\n', ...
        analysis.summary.manip_min, analysis.summary.manip_max);
fprintf('[delta_manipulability_space] cond(Jx) range: [%.4g, %.4g]\n', ...
        analysis.summary.cond_min, analysis.summary.cond_max);
fprintf('[delta_manipulability_space] Type-1 near-singular points: %d\n', analysis.summary.num_type1);
fprintf('[delta_manipulability_space] Type-2 near-singular points: %d\n', analysis.summary.num_type2);

if do_plot && ~isempty(reach_pts)
    fig = create_analysis_figure(analysis);
    analysis.figure_handle = fig;
    if save_outputs
        png_file = fullfile(output_dir, 'delta_manipulability_analysis.png');
        saveas(fig, png_file);
        analysis.png_file = png_file;
        fprintf('[delta_manipulability_space] Figure saved: %s\n', png_file);
    end
end

if save_outputs
    mat_file = fullfile(output_dir, 'delta_manipulability_analysis.mat');
    save(mat_file, 'analysis');
    analysis.mat_file = mat_file;
    fprintf('[delta_manipulability_space] MAT file saved: %s\n', mat_file);
end
end

function summary = build_summary(reach_pts, reachable, manip, cond_Jx, type1, type2, min_abs_diag_Jq)
summary = struct();
summary.num_reachable = sum(reachable);
summary.num_total = numel(reachable);
summary.num_type1 = sum(type1 & reachable);
summary.num_type2 = sum(type2 & reachable);

if isempty(reach_pts)
    summary.bounds = nan(3, 2);
    summary.manip_min = nan;
    summary.manip_max = nan;
    summary.cond_min = nan;
    summary.cond_max = nan;
    summary.min_joint_margin = nan;
    return
end

summary.bounds = [min(reach_pts(:,1)), max(reach_pts(:,1));
                  min(reach_pts(:,2)), max(reach_pts(:,2));
                  min(reach_pts(:,3)), max(reach_pts(:,3))];

reach_manip = manip(reachable);
reach_cond = cond_Jx(reachable);
reach_joint_margin = min_abs_diag_Jq(reachable);

summary.manip_min = min(reach_manip);
summary.manip_max = max(reach_manip);
summary.cond_min = min(reach_cond);
summary.cond_max = max(reach_cond);
summary.min_joint_margin = min(reach_joint_margin);
end

function fig = create_analysis_figure(analysis)
reach = analysis.reachable_points;
reach_idx = analysis.reachable_mask;

manip = analysis.manipulability(reach_idx);
inv_cond = analysis.inv_cond_Jx(reach_idx);
cond_vals = analysis.cond_Jx(reach_idx);
joint_margin = analysis.min_abs_diag_Jq(reach_idx);
type_code = double(analysis.type1(reach_idx)) + 2 * double(analysis.type2(reach_idx));

fig = figure('Name', 'Delta Robot Manipulability Analysis', ...
             'Color', 'w', 'Position', [80, 80, 1300, 900]);
tl = tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile(tl, 1);
scatter_metric(reach, manip, 'Velocity Manipulability');

nexttile(tl, 2);
scatter_metric(reach, inv_cond, 'Inverse cond(Jx)');

nexttile(tl, 3);
scatter_metric(reach, joint_margin, 'Min |diag(Jq)|');

nexttile(tl, 4);
scatter3(reach(:,1)*1e3, reach(:,2)*1e3, reach(:,3)*1e3, 12, type_code, 'filled');
grid on;
axis equal;
view(45, 25);
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title('Singularity Classification');
cb = colorbar;
cb.Ticks = [0, 1, 2, 3];
cb.TickLabels = {'regular', 'type1', 'type2', 'both'};

sgtitle(tl, sprintf('Reachable points: %d, cond(Jx) range [%.2f, %.2f]', ...
        analysis.summary.num_reachable, min(cond_vals), max(cond_vals)));
end

function scatter_metric(reach, metric, title_text)
scatter3(reach(:,1)*1e3, reach(:,2)*1e3, reach(:,3)*1e3, 12, metric, 'filled');
grid on;
axis equal;
view(45, 25);
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title(title_text);
colorbar;
end