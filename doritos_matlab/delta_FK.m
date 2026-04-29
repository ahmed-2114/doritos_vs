function [p, valid, err_msg] = delta_FK(params, theta)
% DELTA_FK  Forward kinematics for a 3-DOF RRS Delta-type parallel robot
%           using the sphere-intersection method.
%
% USAGE:
%   [p, valid]          = delta_FK(params, theta)
%   [p, valid, err_msg] = delta_FK(params, theta)
%
% INPUTS:
%   params  - struct returned by delta_params()
%   theta   - 3x1 (or 1x3) joint angles [theta1; theta2; theta3] in radians
%
% OUTPUTS:
%   p       - 3x1 end-effector position [x; y; z] in metres
%             NaN vector if no real solution exists.
%   valid   - true if a real, unique solution exists
%   err_msg - string describing failure reason (empty if valid)
%
% =========================================================================
% DERIVATION
% =========================================================================
%
% STEP 1 — COMPUTE SHIFTED ELBOW CENTRES
%   For each chain i the elbow E_i depends only on theta_i:
%
%     E_i = [(Rb + L1*cos(theta_i))*cos(phi_i),
%            (Rb + L1*cos(theta_i))*sin(phi_i),
%             L1*sin(theta_i)]'
%
%   Because the platform attachment point is
%     P_i = p + Rp*[cos(phi_i), sin(phi_i), 0]'
%
%   the constraint |E_i - P_i|^2 = L2^2 can be rewritten as
%     |p - E_i'|^2 = L2^2
%
%   where the *shifted* elbow centre E_i' absorbs the platform radius:
%     E_i' = E_i - Rp*[cos(phi_i), sin(phi_i), 0]'
%          = [(Rb - Rp + L1*cos(theta_i))*cos(phi_i),
%             (Rb - Rp + L1*cos(theta_i))*sin(phi_i),
%              L1*sin(theta_i)]'
%
% STEP 2 — LINEARISE BY SPHERE SUBTRACTION
%   We have three sphere equations:
%     |p - E_1'|^2 = L2^2    ... (S1)
%     |p - E_2'|^2 = L2^2    ... (S2)
%     |p - E_3'|^2 = L2^2    ... (S3)
%
%   (S2)-(S1) and (S3)-(S1) eliminate the quadratic p^Tp term and yield
%   two LINEAR equations in [x, y, z]:
%
%     2*(E_k' - E_1')^T * p = |E_k'|^2 - |E_1'|^2,   k = 2, 3
%
%   Written as a 2x3 system:
%       A * p = b
%   where  A(k-1,:) = 2*(E_k' - E_1')'
%          b(k-1)   = |E_k'|^2 - |E_1'|^2
%
% STEP 3 — PARAMETERISE IN z, SOLVE QUADRATIC
%   Partitioning A = [A_xy | A_z] (columns for x,y vs z):
%     A_xy * [x; y] = b - A_z * z
%     [x; y] = A_xy \ (b - A_z * z)         ... linear in z
%            = k_const + k_slope * z
%
%   Substituting into (S1) gives a QUADRATIC in z:
%     (sx^2+sy^2+sz^2)*z^2 + 2*(dx0*sx+dy0*sy+dz0*sz)*z
%       + (dx0^2+dy0^2+dz0^2 - L2^2) = 0
%
%   where  [dx0; dy0; -E1z] is the constant part and [sx; sy; 1] the slope.
%
% STEP 4 — ROOT SELECTION
%   The quadratic yields up to two real roots.  Since +Z points downward,
%   the physically relevant (deeper) workspace root has the LARGER z value.
%   The smaller root corresponds to a mirror position above the base, which
%   is mechanically unreachable in normal delta-robot operation.
%
% =========================================================================
% REFERENCES
% =========================================================================
%   [1] Merlet, J.-P. (2006). Parallel Robots, 2nd ed. Springer, §3.4.
%   [2] Stock, M. & Miller, K. (2003). "Optimal Kinematic Design of
%       Spatial Parallel Manipulators." ASME J. Mech. Des. 125(2):292-301.
%   [3] Tsai, L.-W. (1999). Robot Analysis. Wiley, §7.3.
%   [4] Gosselin, C. & Angeles, J. (1990). "Singularity Analysis of
%       Closed-Loop Kinematic Chains." IEEE T-RA 6(3):281-290.

    % --- Input handling --------------------------------------------------
    theta = theta(:);
    if numel(theta) ~= 3
        error('delta_FK: theta must be a 3-element vector.');
    end

    % --- Unpack ----------------------------------------------------------
    Rb  = params.Rb;
    Rp  = params.Rp;
    L1  = params.L1;
    L2  = params.L2;
    phi = params.phi;

    % --- Default outputs -------------------------------------------------
    p       = [NaN; NaN; NaN];
    valid   = false;
    err_msg = '';

    % --- Step 1: Shifted elbow centres E_i' -----------------------------
    E = zeros(3, 3);    % column i = E_i'
    for i = 1:3
        cp   = cos(phi(i));
        sp   = sin(phi(i));
        r_i  = Rb - Rp + L1*cos(theta(i));   % effective horizontal reach
        E(:,i) = [r_i*cp; r_i*sp; L1*sin(theta(i))];
    end

    % --- Step 2: Build linear system from sphere subtraction -------------
    A = zeros(2, 3);
    b = zeros(2, 1);
    for k = 2:3
        A(k-1, :) = 2 * (E(:,k) - E(:,1))';
        b(k-1)    = dot(E(:,k), E(:,k)) - dot(E(:,1), E(:,1));
    end

    % Partition into xy block and z column
    A_xy = A(:, 1:2);
    A_z  = A(:, 3);

    if abs(det(A_xy)) < 1e-10
        err_msg = 'FK degenerate: A_xy is singular (collinear elbow centres).';
        return
    end

    % --- Step 3: Express x, y linearly in z ----------------------------
    %   [x; y] = k_const + k_slope * z
    k_const = A_xy \ b;           % 2x1
    k_slope = A_xy \ (-A_z);      % 2x1

    % --- Step 4: Substitute into sphere 1 → quadratic in z -------------
    %   p(z) = [k_const(1)+k_slope(1)*z; k_const(2)+k_slope(2)*z; z]
    %   |p(z) - E_1'|^2 = L2^2
    e1 = E(:, 1);

    % Constant and slope parts of (p - E_1')
    dx0 = k_const(1) - e1(1);    dy0 = k_const(2) - e1(2);    dz0 = -e1(3);
    sx  = k_slope(1);             sy  = k_slope(2);             sz  = 1;

    a_q = sx^2 + sy^2 + sz^2;
    b_q = 2*(dx0*sx + dy0*sy + dz0*sz);
    c_q = dx0^2 + dy0^2 + dz0^2 - L2^2;

    disc = b_q^2 - 4*a_q*c_q;

    if disc < -1e-9
        err_msg = sprintf('FK no real solution: discriminant = %.6g.', disc);
        return
    end
    disc = max(disc, 0);    % clamp tiny negatives from floating-point

    z1 = (-b_q + sqrt(disc)) / (2*a_q);
    z2 = (-b_q - sqrt(disc)) / (2*a_q);

    % Select deeper root (+Z downward → larger z = deeper workspace)
    z_sol = max(z1, z2);

    x_sol = k_const(1) + k_slope(1)*z_sol;
    y_sol = k_const(2) + k_slope(2)*z_sol;

    p     = [x_sol; y_sol; z_sol];
    valid = true;
    err_msg = '';

    % --- Optional residual verification ----------------------------------
    % (disabled in production; enable for debugging)
    %{
    for i = 1:3
        P_i = p + Rp*[cos(phi(i)); sin(phi(i)); 0];
        E_i_full = [(Rb+L1*cos(theta(i)))*cos(phi(i));
                    (Rb+L1*cos(theta(i)))*sin(phi(i));
                    L1*sin(theta(i))];
        resid = norm(E_i_full - P_i) - L2;
        fprintf('Chain %d residual: %.2e m\n', i, resid);
    end
    %}
end
