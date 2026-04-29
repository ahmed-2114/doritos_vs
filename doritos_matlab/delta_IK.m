function [theta, valid, err_msg] = delta_IK(params, p)
% DELTA_IK  Closed-form inverse kinematics for a 3-DOF RRS Delta-type
%           parallel robot.
%
% USAGE:
%   [theta, valid]          = delta_IK(params, p)
%   [theta, valid, err_msg] = delta_IK(params, p)
%
% INPUTS:
%   params  - struct returned by delta_params()
%   p       - 3x1 (or 1x3) desired end-effector position [x; y; z] in metres
%             (+Z convention: z > 0 is below the base platform)
%
% OUTPUTS:
%   theta   - 3x1 joint angles [theta1; theta2; theta3] in radians
%             NaN for each chain that has no solution.
%   valid   - true  if a real IK solution exists AND all joint angles
%                   are within [params.th_min, params.th_max]
%             false otherwise
%   err_msg - string describing failure reason (empty string if valid)
%
% =========================================================================
% DERIVATION
% =========================================================================
%
% Let i = 1, 2, 3 denote the three kinematic chains.
%
% NOTATION
%   phi_i   azimuth of chain i in the base plane (0, 120, 240 deg)
%   Rb      base circumradius
%   Rp      platform circumradius
%   L1      upper-arm length
%   L2      forearm length
%   theta_i active joint angle (measured from horizontal, +ve = arm down)
%
% BASE JOINT CENTRE (fixed):
%   B_i = Rb * [cos(phi_i), sin(phi_i), 0]'
%
% ELBOW CENTRE (depends on theta_i):
%   E_i = [(Rb + L1*cos(theta_i))*cos(phi_i),
%          (Rb + L1*cos(theta_i))*sin(phi_i),
%           L1*sin(theta_i)]'
%
% PLATFORM ATTACHMENT POINT (depends on end-effector position p):
%   P_i = p + Rp * [cos(phi_i), sin(phi_i), 0]'
%
% KINEMATIC CONSTRAINT  (forearm length is constant):
%   |E_i - P_i|^2 = L2^2   ... (*)
%
% REDUCTION TO ONE UNKNOWN
%   Define the in-plane and out-of-plane projections of p:
%       s_i =  x*cos(phi_i) + y*sin(phi_i)   [in-plane]
%       t_i = -x*sin(phi_i) + y*cos(phi_i)   [out-of-plane, perpendicular]
%
%   Let  a_i = (Rb - Rp) - s_i
%
%   Expanding (*) and collecting trig terms:
%       a_i*cos(theta_i) - z*sin(theta_i) = C_i
%
%   where  C_i = (L2^2 - L1^2 - a_i^2 - t_i^2 - z^2) / (2*L1)
%
% SOLUTION VIA AUXILIARY ANGLE
%   The equation  A*cos(theta) + B*sin(theta) = C  where A=a_i, B=-z
%   has the form   R * cos(theta - psi) = C
%   with  R = sqrt(A^2 + B^2),  psi = atan2(B, A)
%
%   =>  theta_i = atan2(-z, a_i) + acos(C_i / sqrt(a_i^2 + z^2))
%
%   The "+" sign selects the ELBOW-UP (arm folded above the forearm),
%   which is the mechanically achievable configuration for a standard
%   delta robot.  The "-" sign would give the elbow-down (self-intersecting)
%   solution and is discarded.
%
%   The solution is REAL only when  |C_i / sqrt(a_i^2 + z^2)| <= 1,
%   i.e. the point is within the annular reachable shell of chain i.
%
% =========================================================================
% REFERENCES
% =========================================================================
%   [1] Clavel, R. (1988). "Delta, a fast robot with parallel geometry."
%       Proc. ISIR, pp. 91-100.
%   [2] Merlet, J.-P. (2006). Parallel Robots, 2nd ed. Springer, §3.3.
%   [3] Huang, Z., Liu, J. & Zeng, D. (2013). Theory of Parallel
%       Mechanisms. Springer, ch. 4.
%   [4] Siciliano, B. et al. (2009). Robotics: Modelling, Planning and
%       Control. Springer, ch. 2.

    % --- Input handling --------------------------------------------------
    p = p(:);   % force column vector
    if numel(p) ~= 3
        error('delta_IK: p must be a 3-element vector.');
    end

    % --- Unpack parameters -----------------------------------------------
    Rb   = params.Rb;
    Rp   = params.Rp;
    L1   = params.L1;
    L2   = params.L2;
    phi  = params.phi;

    x = p(1);
    y = p(2);
    z = p(3);

    % --- Initialise outputs ----------------------------------------------
    theta   = NaN(3, 1);
    valid   = true;
    err_msg = '';

    % --- Workspace boundary pre-check -----------------------------------
    if isfield(params, 'ws_lim')
        lim = params.ws_lim;
        if x < lim(1,1) || x > lim(1,2) || ...
           y < lim(2,1) || y > lim(2,2) || ...
           z < lim(3,1) || z > lim(3,2)
            valid   = false;
            err_msg = 'Point outside Cartesian workspace bounding box.';
            return
        end
    end

    % --- Per-chain IK ----------------------------------------------------
    for i = 1:3
        cp = cos(phi(i));
        sp = sin(phi(i));

        % In-plane and out-of-plane projections
        s_i = x*cp + y*sp;          % projection of EE onto chain direction
        t_i = -x*sp + y*cp;         % perpendicular component

        a_i = (Rb - Rp) - s_i;      % in-plane offset from base joint

        % Right-hand-side constant
        C_i = (L2^2 - L1^2 - a_i^2 - t_i^2 - z^2) / (2*L1);

        % Denominator = sqrt(a_i^2 + z^2)
        denom = sqrt(a_i^2 + z^2);

        if denom < 1e-10
            % EE lies exactly on the vertical axis through the base joint;
            % geometrically degenerate for this chain.
            valid   = false;
            err_msg = sprintf('Chain %d: degenerate (EE on chain axis).', i);
            return
        end

        ratio = C_i / denom;

        if abs(ratio) > 1 + 1e-9    % small tolerance for floating-point
            valid   = false;
            err_msg = sprintf('Chain %d: point unreachable (|ratio|=%.4f > 1).', ...
                              i, abs(ratio));
            return
        end

        ratio = max(-1, min(1, ratio));   % clamp to [-1,1] for acos

        % Closed-form solution (elbow-up branch)
        theta(i) = atan2(-z, a_i) + acos(ratio);
    end

    % --- Joint limit check -----------------------------------------------
    for i = 1:3
        if theta(i) < params.th_min - 1e-6 || theta(i) > params.th_max + 1e-6
            valid   = false;
            err_msg = sprintf('Chain %d: theta=%.2f deg violates limits [%.1f, %.1f] deg.', ...
                              i, rad2deg(theta(i)), ...
                              rad2deg(params.th_min), rad2deg(params.th_max));
            return
        end
    end
end
