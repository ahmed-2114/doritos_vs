function report = run_delta_analysis(varargin)
% RUN_DELTA_ANALYSIS  One-command workspace, manipulability, and
% singularity analysis for the current delta robot parameters.

params = delta_params();
res = 21;
do_plot = true;
save_outputs = true;

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
        otherwise
            k = k + 1;
    end
end

fprintf('=== Delta Analysis Runner ===\n');
fprintf('Resolution: %d\n', res);
fprintf('Home pose: [%.4f %.4f %.4f] m\n', params.home_pos(1), params.home_pos(2), params.home_pos(3));

[points, in_ws] = delta_workspace(params, 'res', res, 'plot', do_plot);
analysis = delta_manipulability_space(params, 'res', res, 'plot', do_plot, 'save', save_outputs);

home_theta = delta_command_to_model(params, repmat(params.home_command_deg, 3, 1));
[~, ~, type1_home, type2_home, info_home] = delta_singularity(params, home_theta);

report = struct();
report.params = params;
report.res = res;
report.workspace_points = points;
report.workspace_mask = in_ws;
report.analysis = analysis;
report.home_type1 = type1_home;
report.home_type2 = type2_home;
report.home_cond_Jx = info_home.cond_Jx;
report.home_det_Jx = info_home.det_Jx;

fprintf('Home pose singularity: type1=%s, type2=%s, cond(Jx)=%.3f\n', ...
        mat2str(type1_home), mat2str(type2_home), info_home.cond_Jx);
fprintf('Workspace bounds [m]:\n');
disp(analysis.summary.bounds);

if save_outputs
    report_file = fullfile(fileparts(mfilename('fullpath')), 'delta_analysis_report.mat');
    save(report_file, 'report');
    report.report_file = report_file;
    fprintf('Analysis report saved: %s\n', report_file);
end
end
