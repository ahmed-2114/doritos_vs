function traj = generate_circle_signal_builder()
% GENERATE_CIRCLE_SIGNAL_BUILDER  Create a validated Signal Editor dataset
% that moves to max-Z down, max-Z up, centre-Z, then draws a circle.

params = delta_params();

home = params.home_pos(:)';
z_up = home(3);
z_down = -0.2250;
% Use a near-maximum XY circle plane validated against the current robot.
z_center = -0.1588;
circle_radius = 0.1300;
circle_turns = 1;

% Waypoints before the circle segment.
% Columns: time [s], x [m], y [m], z [m]
waypoints = [
     0.0, home(1), home(2), home(3);
     2.0, 0.0000, 0.0000, z_down;
     4.0, 0.0000, 0.0000, z_up;
     6.0, 0.0000, 0.0000, z_center;
     8.0, circle_radius, 0.0000, z_center
];

t_wp = waypoints(:, 1);
x_wp = waypoints(:, 2);
y_wp = waypoints(:, 3);
z_wp = waypoints(:, 4);

t_pre = linspace(t_wp(1), t_wp(end), 401)';
x_pre = interp1(t_wp, x_wp, t_pre, 'pchip');
y_pre = interp1(t_wp, y_wp, t_pre, 'pchip');
z_pre = interp1(t_wp, z_wp, t_pre, 'pchip');

% Circle segment at constant Z around the workspace centre.
t_circle = linspace(8.0, 16.0, 641)';
theta_circle = linspace(0, 2*pi*circle_turns, numel(t_circle))';
x_circle = circle_radius * cos(theta_circle);
y_circle = circle_radius * sin(theta_circle);
z_circle = z_center * ones(size(t_circle));

% Return to home.
t_return_wp = [16.0; 18.0; 20.0];
x_return_wp = [x_circle(end); 0.0; home(1)];
y_return_wp = [y_circle(end); 0.0; home(2)];
z_return_wp = [z_circle(end); z_center; home(3)];

t_return = linspace(t_return_wp(1), t_return_wp(end), 161)';
x_return = interp1(t_return_wp, x_return_wp, t_return, 'pchip');
y_return = interp1(t_return_wp, y_return_wp, t_return, 'pchip');
z_return = interp1(t_return_wp, z_return_wp, t_return, 'pchip');

t = [t_pre; t_circle(2:end); t_return(2:end)];
x = [x_pre; x_circle(2:end); x_return(2:end)];
y = [y_pre; y_circle(2:end); y_return(2:end)];
z = [z_pre; z_circle(2:end); z_return(2:end)];

worst_cond = 0;
for k = 1:numel(t)
    p = [x(k); y(k); z(k)];
    [theta, valid, err_msg] = delta_IK(params, p);
    if ~valid
        error('Circle trajectory invalid at t=%.3f s: %s for p=[%.4f %.4f %.4f].', ...
              t(k), err_msg, p(1), p(2), p(3));
    end

    if ~all(isfinite(theta))
        error('Circle trajectory returned non-finite joint angles at t=%.3f s.', t(k));
    end

    evalc('[~, ~, type1, type2, info] = delta_singularity(params, theta);');
    if type1 || type2
        error('Circle trajectory hits a singularity at t=%.3f s.', t(k));
    end

    worst_cond = max(worst_cond, info.cond_Jx);
end

traj = struct();
traj.name = 'circle_test';
traj.home = home;
traj.z_up = z_up;
traj.z_down = z_down;
traj.z_center = z_center;
traj.circle_radius = circle_radius;
traj.waypoints = waypoints;
traj.t = t;
traj.x = x;
traj.y = y;
traj.z = z;
traj.xyz = [x, y, z];
traj.x_ts = timeseries(x, t);
traj.y_ts = timeseries(y, t);
traj.z_ts = timeseries(z, t);
traj.worst_cond_Jx = worst_cond;

scenario = Simulink.SimulationData.Dataset;
scenario = scenario.addElement(traj.x_ts, 'x');
scenario = scenario.addElement(traj.y_ts, 'y');
scenario = scenario.addElement(traj.z_ts, 'z');

mat_file = fullfile(fileparts(mfilename('fullpath')), 'signal_builder_circle_test.mat');
save(mat_file, 'scenario', 'traj', 'waypoints', 't', 'x', 'y', 'z');

assignin('base', 'signal_builder_waypoints_circle', waypoints);
assignin('base', 'signal_builder_traj_circle', traj);
assignin('base', 'x_ref_ts_circle', traj.x_ts);
assignin('base', 'y_ref_ts_circle', traj.y_ts);
assignin('base', 'z_ref_ts_circle', traj.z_ts);

fprintf('Circle Signal Editor profile created.\n');
fprintf('Home position: [%.4f, %.4f, %.4f] m\n', home(1), home(2), home(3));
fprintf('Z up/down/center: [%.4f, %.4f, %.4f] m\n', z_up, z_down, z_center);
fprintf('Circle radius: %.4f m\n', circle_radius);
fprintf('Dense trajectory samples: %d\n', numel(t));
fprintf('X range: [%.4f, %.4f] m\n', min(x), max(x));
fprintf('Y range: [%.4f, %.4f] m\n', min(y), max(y));
fprintf('Z range: [%.4f, %.4f] m\n', min(z), max(z));
fprintf('Worst cond(Jx): %.3f\n', worst_cond);
fprintf('MAT file saved: %s\n', mat_file);
end