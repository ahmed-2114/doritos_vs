function [theta, valid, err_msg] = delta_IK(params, p)
% DELTA_IK  Closed-form inverse kinematics for a 3-DOF delta robot.
% Uses the convention +Z upward, so points below the base have negative Z.

    % --- Input handling ---
    p = p(:);
    if numel(p) ~= 3
        theta   = [0; 0; 0];
        valid   = false;
        err_msg = 'bad_input';
        return;
    end

    % --- Unpack ---
    Rb  = params.Rb;
    Rp  = params.Rp;
    L1  = params.L1;
    L2  = params.L2;
    phi = params.phi;

    x = p(1);
    y = p(2);
    z = p(3);

    % --- Initialise outputs ---
    theta   = zeros(3,1);
    valid   = true;
    err_msg = '';

    % --- Workspace bounding-box check ---
    if isfield(params, 'ws_lim')
        lim = params.ws_lim;
        if x < lim(1,1) || x > lim(1,2) || ...
           y < lim(2,1) || y > lim(2,2) || ...
           z < lim(3,1) || z > lim(3,2)
            theta   = [0; 0; 0];
            valid   = false;
            err_msg = 'workspace';
            return;
        end
    end

    % --- Per-chain IK ---
    for i = 1:3
        cp = cos(phi(i));
        sp = sin(phi(i));

        % Projections of p into the chain plane
        s_i =  x*cp + y*sp;
        t_i = -x*sp + y*cp;

        % Geometry term
        a_i = (Rb - Rp) - s_i;

        % Closed-form constant
        C_i = (L2^2 - L1^2 - a_i^2 - t_i^2 - z^2) / (2*L1);

        % Degenerate case
        denom = sqrt(a_i^2 + z^2);
        if denom < 1e-12
            theta   = [0; 0; 0];
            valid   = false;
            err_msg = 'degenerate';
            return;
        end

        ratio = C_i / denom;

        % Reachability test
        if abs(ratio) > 1 + 1e-9
            theta   = [0; 0; 0];
            valid   = false;
            err_msg = 'unreachable';
            return;
        end

        % Clamp for numerical safety
        ratio = max(-1, min(1, ratio));

        % Physical solution for the +Z-up convention used by the model
        theta(i) = atan2(z, a_i) + acos(ratio);
    end

    % --- Joint limit check ---
    for i = 1:3
        if theta(i) < params.th_min - 1e-6 || theta(i) > params.th_max + 1e-6
            theta   = [0; 0; 0];
            valid   = false;
            err_msg = 'joint_limit';
            return;
        end
    end
end