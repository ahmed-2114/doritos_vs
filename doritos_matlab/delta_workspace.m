function [points, in_ws] = delta_workspace(params, varargin)
% DELTA_WORKSPACE  Samples and visualises the reachable Cartesian workspace
%                  of a 3-DOF RRS Delta-type parallel robot.
%                  Also provides a fast single-point membership test.
%
% =========================================================================
% USAGE
% =========================================================================
%
%   Full 3-D scan + plot (default):
%       [points, in_ws] = delta_workspace(params)
%
%   Adjust grid resolution:
%       [points, in_ws] = delta_workspace(params, 'res', 30)
%
%   Suppress the plot:
%       [points, in_ws] = delta_workspace(params, 'plot', false)
%
%   Single-point membership test (fast, no plot):
%       in_ws = delta_workspace(params, 'check', [0; 0; 0.35])
%
% =========================================================================
% INPUTS
% =========================================================================
%   params          struct from delta_params()
%
%   Optional name-value pairs:
%   'res'   N       Grid resolution (N points per axis); default 25.
%                   Total samples = N^3, so large N is slow.
%   'plot'  true/false  Whether to generate a 3-D scatter plot; default true.
%   'check' p       3x1 point to test; triggers single-point mode.
%
% =========================================================================
% OUTPUTS
% =========================================================================
%   points  Nx3 array of sampled [x, y, z] positions [m]
%   in_ws   Nx1 logical: true = IK solution exists with joints in limits
%
% =========================================================================
% METHOD
% =========================================================================
%   The workspace is characterised by exhaustive sampling of the Cartesian
%   bounding box defined in params.ws_lim.  For each candidate point,
%   delta_IK() is called and the 'valid' flag is stored.
%
%   The reachable workspace of a delta robot is bounded by:
%     - Outer boundary: maximum arm extension  (L1 + L2) - (Rb - Rp)
%     - Inner boundary: minimum arm extension  |L2 - L1| - (Rb - Rp)
%                       (annular dead zone at the centre if L1 ≈ L2)
%     - Upper boundary (z_min): joint upper-limit plane
%     - Lower boundary (z_max): joint lower-limit plane
%     - Lateral tilt: asymmetric shrinkage away from robot centre
%
% =========================================================================
% REFERENCES
% =========================================================================
%   [1] Merlet, J.-P. (2006). Parallel Robots, 2nd ed. Springer, §6.2.
%   [2] Clavel, R. (1988). "Delta, a fast robot with parallel geometry."
%   [3] Huang, Z. et al. (2013). Theory of Parallel Mechanisms. Springer.

    % -------------------------------------------------------------------------
    % Parse name-value arguments
    % -------------------------------------------------------------------------
    p_check = [];
    do_plot = true;
    res     = 25;

    k = 1;
    while k <= numel(varargin)
        key = lower(varargin{k});
        switch key
            case 'check'
                p_check = varargin{k+1}(:);
                do_plot = false;
                k = k + 2;
            case 'res'
                res = varargin{k+1};
                k = k + 2;
            case 'plot'
                do_plot = varargin{k+1};
                k = k + 2;
            otherwise
                warning('delta_workspace: unknown option "%s", skipping.', key);
                k = k + 1;
        end
    end

    % -------------------------------------------------------------------------
    % Single-point mode
    % -------------------------------------------------------------------------
    if ~isempty(p_check)
        [~, in_ws] = delta_IK(params, p_check);
        points = p_check';
        if nargout == 0
            if in_ws
                fprintf('[delta_workspace] Point (%.4f, %.4f, %.4f) m is REACHABLE.\n', ...
                        p_check(1), p_check(2), p_check(3));
            else
                fprintf('[delta_workspace] Point (%.4f, %.4f, %.4f) m is UNREACHABLE.\n', ...
                        p_check(1), p_check(2), p_check(3));
            end
        end
        return
    end

    % -------------------------------------------------------------------------
    % Full workspace scan
    % -------------------------------------------------------------------------
    lim = params.ws_lim;
    xv  = linspace(lim(1,1), lim(1,2), res);
    yv  = linspace(lim(2,1), lim(2,2), res);
    zv  = linspace(lim(3,1), lim(3,2), res);

    [X, Y, Z] = meshgrid(xv, yv, zv);
    pts  = [X(:), Y(:), Z(:)];
    N    = size(pts, 1);
    in_ws = false(N, 1);

    fprintf('[delta_workspace] Scanning %d grid points (res=%d)...\n', N, res);
    t_start = tic;

    for k = 1:N
        [~, ok] = delta_IK(params, pts(k,:)');
        in_ws(k) = ok;
    end

    elapsed = toc(t_start);
    n_reach = sum(in_ws);
    fprintf('[delta_workspace] Done: %d / %d points reachable (%.1f%%) in %.2f s.\n', ...
            n_reach, N, 100*n_reach/N, elapsed);

    points = pts;

    % -------------------------------------------------------------------------
    % Workspace statistics
    % -------------------------------------------------------------------------
    if n_reach > 0
        reach = pts(in_ws, :);
        fprintf('[delta_workspace] Workspace bounds (reachable points):\n');
        fprintf('  X: [%.4f, %.4f] m\n', min(reach(:,1)), max(reach(:,1)));
        fprintf('  Y: [%.4f, %.4f] m\n', min(reach(:,2)), max(reach(:,2)));
        fprintf('  Z: [%.4f, %.4f] m\n', min(reach(:,3)), max(reach(:,3)));

        % Approximate volume (voxel counting)
        dx = (lim(1,2)-lim(1,1)) / (res-1);
        dy = (lim(2,2)-lim(2,1)) / (res-1);
        dz = (lim(3,2)-lim(3,1)) / (res-1);
        approx_vol = n_reach * dx * dy * dz * 1e6;  % cm^3
        fprintf('  Approximate volume: %.1f cm^3\n', approx_vol);
    end

    % -------------------------------------------------------------------------
    % 3-D scatter visualisation
    % -------------------------------------------------------------------------
    if do_plot && n_reach > 0
        reach = pts(in_ws, :);

        fig = figure('Name', 'Delta Robot Reachable Workspace', ...
                     'Color', 'w', 'Position', [100, 100, 900, 700]);

        % Main 3-D scatter
        ax = axes(fig);
        scatter3(ax, reach(:,1)*1e3, reach(:,2)*1e3, reach(:,3)*1e3, ...
                 5, reach(:,3)*1e3, 'filled', 'MarkerFaceAlpha', 0.25);

        % Base platform marker
        hold(ax, 'on');
        scatter3(ax, 0, 0, 0, 150, 'k', 'filled', 'Marker', 'd');
        text(ax, 5, 5, -10, 'Base', 'FontSize', 9, 'Color', 'k');

        % Base joint markers
        Rb_mm = params.Rb * 1e3;
        for i = 1:3
            bx = Rb_mm * cos(params.phi(i));
            by = Rb_mm * sin(params.phi(i));
            scatter3(ax, bx, by, 0, 60, [0.8 0.2 0.2], 'filled');
        end

        colorbar(ax);
        colormap(ax, 'parula');

        xlabel(ax, 'X [mm]', 'FontSize', 11);
        ylabel(ax, 'Y [mm]', 'FontSize', 11);
        zlabel(ax, 'Z [mm]', 'FontSize', 11);
        title(ax, 'Delta Robot — Reachable Workspace', 'FontSize', 13);

        grid(ax, 'on');
        axis(ax, 'equal');
        view(ax, 45, 25);
        set(ax, 'ZDir', 'reverse');   % flip Z-axis so workspace appears below base

        % Projected XZ slice at Y=0
        subplot_ax = axes(fig, 'Position', [0.72, 0.12, 0.23, 0.30]);
        near_mid = abs(reach(:,2)) < (lim(2,2) - lim(2,1)) / (2*res);
        if sum(near_mid) > 0
            scatter(subplot_ax, reach(near_mid,1)*1e3, reach(near_mid,3)*1e3, ...
                    3, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
            xlabel(subplot_ax, 'X [mm]', 'FontSize', 8);
            ylabel(subplot_ax, 'Z [mm]', 'FontSize', 8);
            title(subplot_ax, 'XZ slice (Y≈0)', 'FontSize', 8);
            set(subplot_ax, 'YDir', 'reverse');
            grid(subplot_ax, 'on');
        end

        fprintf('[delta_workspace] Plot generated.\n');
    end
end
