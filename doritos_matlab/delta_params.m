function params = delta_params()
% DELTA_PARAMS  Returns the geometric and joint-limit parameters for a
%               3-DOF RRS Delta-type parallel robot.
%
% USAGE:
%   params = delta_params()
%
% OUTPUT:
%   params  struct with the following fields:
%     .Rb       Base platform circumradius [m]
%     .Rp       Moving platform circumradius [m]
%     .L1       Upper arm (proximal link) length [m]
%     .L2       Forearm (distal parallelogram link) length [m]
%     .phi      Base-joint azimuth angles [rad] (1x3 row vector)
%     .th_min   Joint angle lower limit [rad]
%     .th_max   Joint angle upper limit [rad]
%     .ws_lim   Cartesian workspace bounding box [3x2]:
%               [x_min x_max; y_min y_max; z_min z_max]  [m]
%
% -------------------------------------------------------------------------
% COORDINATE CONVENTION
% -------------------------------------------------------------------------
%   - Origin   : centre of the base (fixed) platform
%   - +Z axis  : pointing DOWNWARD (into the workspace)
%   - +X axis  : aligned with Chain 1 (phi_1 = 0 deg)
%   - Chain azimuths:
%       phi_1 =   0 deg  (along +X)
%       phi_2 = 120 deg
%       phi_3 = 240 deg
%
%   Joint angle theta_i is measured from the HORIZONTAL plane, positive
%   in the direction of increasing z (i.e. arm rotating downward).
%   At theta_i = 0  the upper arm is horizontal.
%   At theta_i > 0  the elbow dips below the base plane.
%
% -------------------------------------------------------------------------
% PHYSICAL MEANING OF PARAMETERS
% -------------------------------------------------------------------------
%   Rb  : circumradius of the equilateral triangle formed by the three
%         revolute (motorised) joint centres on the fixed base.
%   Rp  : circumradius of the equilateral triangle formed by the three
%         spherical joint attachment points on the moving platform.
%   L1  : length of each upper arm (rigid link driven by the motor).
%   L2  : length of each forearm (parallelogram assembly that keeps the
%         platform orientation fixed).
%
%   Note: the "side length" of the base triangle  f = Rb * sqrt(3)
%         the "side length" of the platform triangle e = Rp * sqrt(3)
%
% -------------------------------------------------------------------------
% ADJUSTING TO YOUR PHYSICAL ROBOT
% -------------------------------------------------------------------------
%   Replace the numerical values below with your measured / designed
%   values.  All lengths in metres.
%
%   Typical benchtop educational delta robot:
%     Rb = 0.150-0.250 m,  Rp = 0.040-0.080 m
%     L1 = 0.150-0.250 m,  L2 = 0.300-0.450 m
%
% -------------------------------------------------------------------------
% REFERENCES
% -------------------------------------------------------------------------
%   [1] Clavel, R. (1988). "Delta, a fast robot with parallel geometry."
%       Proc. Int. Symp. Industrial Robots (ISIR), pp. 91-100.
%   [2] Merlet, J.-P. (2006). Parallel Robots, 2nd ed. Springer, ch. 1-2.
%   [3] Tsai, L.-W. (1999). Robot Analysis: The Mechanics of Serial and
%       Parallel Manipulators. Wiley, ch. 7.

    % =====================================================================
    % GEOMETRY  -- edit these four numbers to match your robot
    % =====================================================================
    params.Rb = 0.200;          % [m]  base circumradius
    params.Rp = 0.060;          % [m]  moving-platform circumradius
    params.L1 = 0.200;          % [m]  upper-arm length
    params.L2 = 0.400;          % [m]  forearm length

    % =====================================================================
    % CHAIN AZIMUTHS  (do NOT change unless your robot is non-symmetric)
    % =====================================================================
    params.phi = [0, 2*pi/3, 4*pi/3];   % [rad]  phi_1, phi_2, phi_3

    % =====================================================================
    % JOINT LIMITS  -- set from your motor/servo hardware spec sheet
    % =====================================================================
    params.th_min = deg2rad(-20);   % [rad]  most retracted position
    params.th_max = deg2rad( 90);   % [rad]  most extended position

    % =====================================================================
    % CARTESIAN WORKSPACE BOUNDING BOX  -- conservative initial estimate
    % Refine this after running delta_workspace.m
    % =====================================================================
    params.ws_lim = [ -0.200,  0.200;   % x [m]
                      -0.200,  0.200;   % y [m]
                       0.100,  0.550 ]; % z [m]  (z > 0 = below base)

    % =====================================================================
    % DERIVED / CONVENIENCE FIELDS
    % =====================================================================
    params.f_base    = params.Rb * sqrt(3);   % [m] base triangle side
    params.e_platform = params.Rp * sqrt(3);  % [m] platform triangle side

    % Print summary to console
    fprintf('=== Delta Robot Parameters Loaded ===\n');
    fprintf('  Rb (base circumradius)     : %.4f m\n', params.Rb);
    fprintf('  Rp (platform circumradius) : %.4f m\n', params.Rp);
    fprintf('  L1 (upper arm)             : %.4f m\n', params.L1);
    fprintf('  L2 (forearm)               : %.4f m\n', params.L2);
    fprintf('  Joint limits               : [%.1f, %.1f] deg\n', ...
            rad2deg(params.th_min), rad2deg(params.th_max));
    fprintf('  Max reach (approx.)        : %.4f m\n', params.L1 + params.L2);
    fprintf('=====================================\n');
end
