function [Jx, Jq, type1, type2, info] = delta_singularity(params, theta)
% DELTA_SINGULARITY  Velocity Jacobian computation and singularity detection
%                    for a 3-DOF RRS Delta-type parallel robot.
%
% USAGE:
%   [Jx, Jq, type1, type2, info] = delta_singularity(params, theta)
%
% INPUTS:
%   params  - struct returned by delta_params()
%   theta   - 3x1 (or 1x3) joint angles in radians
%
% OUTPUTS:
%   Jx      3x3  Cartesian (direct) Jacobian
%                Row i = unit forearm vector n_i' (from P_i to E_i).
%                det(Jx) = 0  →  Type-1 (FK / direct kinematic) singularity.
%
%   Jq      3x3  Joint-space (inverse) Jacobian  (diagonal matrix)
%                Jq(i,i) = n_i . v_i   (forearm dotted with elbow tangent)
%                Jq(i,i) = 0  →  Type-2 (IK / inverse kinematic) singularity.
%
%   type1   logical  true if |det(Jx)| < SING_TOL
%   type2   logical  true if any |Jq(i,i)| < SING_TOL
%
%   info    struct with extended diagnostics:
%     .det_Jx      det(Jx)
%     .diag_Jq     3x1 diagonal elements of Jq
%     .cond_Jx     condition number of Jx
%     .n_vecs      3x3 matrix of forearm unit vectors (column i = n_i)
%     .v_vecs      3x3 matrix of elbow tangent vectors (column i = v_i)
%     .p           3x1 end-effector position [m] (from FK)
%     .SING_TOL    threshold used
%
% =========================================================================
% THEORY — VELOCITY KINEMATICS OF A DELTA ROBOT
% =========================================================================
%
% The velocity kinematics are derived by differentiating the kinematic
% constraints |E_i(theta_i) - P_i(p)|^2 = L2^2  with respect to time:
%
%   2*(E_i - P_i)' * (dE_i/dt - dp/dt) = 0
%
% Since E_i depends only on theta_i:
%   dE_i/dt = (dE_i/dtheta_i) * theta_dot_i = v_i * theta_dot_i
%
% where  v_i = L1 * [-sin(theta_i)*cos(phi_i),
%                    -sin(theta_i)*sin(phi_i),
%                     cos(theta_i)]'
%
% Defining the forearm unit vector:
%   n_i = (E_i - P_i) / L2
%
% The constraint becomes:
%   n_i . v_i * theta_dot_i = n_i . p_dot
%
% Stacking three chains → MATRIX FORM:
%   Jq * theta_dot = Jx * p_dot
%
%   Jq = diag([n_1.v_1,  n_2.v_2,  n_3.v_3])   (3x3 diagonal)
%   Jx = [n_1'; n_2'; n_3']                      (3x3)
%
% The FULL Jacobian mapping theta_dot to p_dot is:
%   p_dot = Jq^{-1} * Jx * ... wait, that's:
%   Jx * p_dot = Jq * theta_dot  =>  p_dot = Jx^{-1} * Jq * theta_dot
%   OR:  theta_dot = Jq^{-1} * Jx * p_dot
%
% =========================================================================
% SINGULARITY TYPES
% =========================================================================
%
% TYPE 1 — Direct / FK Singularity:  det(Jx) = 0
%   Occurs when the three forearm unit vectors {n_1, n_2, n_3} become
%   LINEARLY DEPENDENT (coplanar).  The platform gains one or more
%   uncontrollable degrees of freedom even when all joints are locked.
%   This is the MOST DANGEROUS singularity because the robot loses
%   structural stiffness and the joint forces become indeterminate.
%
%   Physical cause examples:
%   (a) All three forearms are parallel (e.g. all vertical at extreme
%       low positions).
%   (b) All three forearms lie in the same horizontal plane (all elbows
%       at the same height and same radial distance, very top of workspace).
%   (c) The three platform attachment points P_i are collinear.
%
% TYPE 2 — Inverse / IK Singularity:  det(Jq) = 0  ⟺  some Jq(i,i) = 0
%   Occurs for chain i when  n_i . v_i = 0, i.e. the forearm vector is
%   PERPENDICULAR to the elbow tangent vector v_i.
%   Geometrically this means the upper arm and forearm of chain i are
%   COLLINEAR (arm fully extended OR fully folded back).
%   The robot reaches the BOUNDARY of the workspace for that chain;
%   that chain can no longer contribute force along the arm direction.
%
%   Physical cause:
%   (a) Arm fully extended: theta_i = theta_max and the forearm and upper
%       arm point in opposite directions (chain is stretched).
%   (b) Arm fully retracted: elbow swings back and forearm folds onto
%       upper arm.
%
% TYPE 3 — Combined Singularity:  both Type 1 and Type 2 simultaneously
%   Extremely rare; requires special symmetry.
%
% =========================================================================
% CONDITION NUMBER AS PROXIMITY MEASURE
% =========================================================================
%   cond(Jx) = sigma_max / sigma_min  (ratio of singular values of Jx)
%   Large cond(Jx) → robot is NEAR a Type-1 singularity.
%   As a rule of thumb: cond(Jx) > 50 warrants caution in trajectory
%   planning; cond(Jx) > 200 is considered dangerously close.
%
% =========================================================================
% REFERENCES
% =========================================================================
%   [1] Gosselin, C. & Angeles, J. (1990). "Singularity Analysis of
%       Closed-Loop Kinematic Chains." IEEE T-RA 6(3):281-290.
%       (THE foundational paper on parallel robot singularities)
%   [2] Merlet, J.-P. (2006). Parallel Robots, 2nd ed. Springer, §6.3-6.4.
%   [3] Liu, X.-J. & Wang, J. (2014). Parallel Kinematics. Springer.
%   [4] Zlatanov, D., Fenton, R.G. & Benhabib, B. (1994). "Singularity
%       Analysis of Mechanisms and Robots Via a Velocity-Equation Model."
%       Proc. IEEE ICRA, pp. 986-991.

    % --- Input handling --------------------------------------------------
    theta = theta(:);
    if numel(theta) ~= 3
        error('delta_singularity: theta must be a 3-element vector.');
    end

    SING_TOL = 1e-4;   % threshold for near-zero; adjust if needed

    % --- Unpack ----------------------------------------------------------
    Rb  = params.Rb;
    Rp  = params.Rp;
    L1  = params.L1;
    L2  = params.L2;
    phi = params.phi;

    % --- Run FK to obtain end-effector position --------------------------
    [p, fk_ok, fk_msg] = delta_FK(params, theta);
    if ~fk_ok
        error('delta_singularity: FK failed — %s', fk_msg);
    end

    % --- Allocate Jacobians ----------------------------------------------
    Jx   = zeros(3, 3);
    Jq   = zeros(3, 3);
    n_vecs = zeros(3, 3);
    v_vecs = zeros(3, 3);

    % --- Per-chain computation -------------------------------------------
    for i = 1:3
        cp = cos(phi(i));
        sp = sin(phi(i));
        ct = cos(theta(i));
        st = sin(theta(i));

        % Elbow position
        E_i = [(Rb + L1*ct)*cp;
               (Rb + L1*ct)*sp;
                L1*st];

        % Platform attachment
        P_i = p + Rp*[cp; sp; 0];

        % Forearm vector (from P_i to E_i) and its unit vector
        diff_i  = E_i - P_i;
        norm_diff = norm(diff_i);

        if norm_diff < 1e-12
            warning('delta_singularity: chain %d has zero-length forearm — degenerate.', i);
            n_i = [0; 0; 0];
        else
            n_i = diff_i / norm_diff;
        end

        % Elbow tangent (partial derivative of E_i w.r.t. theta_i)
        %   dE_i/dtheta_i = L1 * [-st*cp, -st*sp, ct]'
        v_i = L1 * [-st*cp; -st*sp; ct];

        % Fill Jacobians
        Jx(i, :)  = n_i';
        Jq(i, i)  = n_i' * v_i;

        % Store for info struct
        n_vecs(:, i) = n_i;
        v_vecs(:, i) = v_i;
    end

    % --- Singularity tests -----------------------------------------------
    det_Jx  = det(Jx);
    diag_Jq = diag(Jq);
    cond_Jx = cond(Jx);

    type1 = abs(det_Jx)     < SING_TOL;
    type2 = any(abs(diag_Jq) < SING_TOL);

    % --- Build info struct -----------------------------------------------
    info.det_Jx   = det_Jx;
    info.diag_Jq  = diag_Jq;
    info.cond_Jx  = cond_Jx;
    info.n_vecs   = n_vecs;
    info.v_vecs   = v_vecs;
    info.p        = p;
    info.SING_TOL = SING_TOL;

    % Angles between consecutive forearm vectors (diagnostic)
    for i = 1:3
        j = mod(i, 3) + 1;
        info.forearm_angle_deg(i) = rad2deg(acos(max(-1, min(1, ...
            n_vecs(:,i)' * n_vecs(:,j)))));
    end

    % --- Console report --------------------------------------------------
    fprintf('=== Singularity Analysis ===\n');
    fprintf('  Joint angles:  [%.2f, %.2f, %.2f] deg\n', ...
            rad2deg(theta(1)), rad2deg(theta(2)), rad2deg(theta(3)));
    fprintf('  EE position:   [%.4f, %.4f, %.4f] m\n', p(1), p(2), p(3));
    fprintf('\n  --- Cartesian Jacobian Jx ---\n');
    disp(Jx);
    fprintf('  det(Jx)        = %.6f\n', det_Jx);
    fprintf('  cond(Jx)       = %.2f\n', cond_Jx);
    fprintf('  Type-1 singularity (det(Jx)≈0): %s\n', mat2str(type1));
    fprintf('\n  --- Joint Jacobian diag(Jq) ---\n');
    fprintf('  [%.4f,  %.4f,  %.4f]\n', diag_Jq(1), diag_Jq(2), diag_Jq(3));
    fprintf('  Type-2 singularity (any Jq_ii≈0): %s\n', mat2str(type2));
    fprintf('\n  Angles between forearms:\n');
    fprintf('  n1^n2=%.1f°,  n2^n3=%.1f°,  n3^n1=%.1f°\n', ...
            info.forearm_angle_deg(1), info.forearm_angle_deg(2), info.forearm_angle_deg(3));
    fprintf('============================\n');
end
