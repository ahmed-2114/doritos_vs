clear;
clc;

params = delta_params();

% A few centered test points inside the current workspace. The model uses
% +Z upward, so reachable points sit below the base at negative Z.
test_points = [
     params.home_pos.';
     0.000,  0.000, -0.1500;
     0.020,  0.000, -0.1600;
    -0.020,  0.020, -0.1800;
     0.030, -0.020, -0.1900
];

num_points = size(test_points, 1);
num_success = 0;
max_error_m = 0;

fprintf('\n=== FK/IK Consistency Test ===\n');

for i = 1:num_points
    p = test_points(i, :)';

    fprintf('\nTest point %d\n', i);
    fprintf('  p target           : [%.4f, %.4f, %.4f] m\n', p(1), p(2), p(3));

    [theta, ik_valid, ik_err] = delta_IK(params, p);

    if ik_valid
        theta_deg = rad2deg(theta);
        fprintf('  theta (deg)        : [%.2f, %.2f, %.2f]\n', ...
                theta_deg(1), theta_deg(2), theta_deg(3));
        fprintf('  IK valid           : true\n');

        [p_fk, fk_valid, fk_err] = delta_FK(params, theta);
        if fk_valid
            fk_valid_text = 'true';
        else
            fk_valid_text = 'false';
        end
        fprintf('  FK valid           : %s\n', fk_valid_text);

        if fk_valid
            err_m = norm(p - p_fk);
            err_mm = 1e3 * err_m;

            fprintf('  p reconstructed    : [%.4f, %.4f, %.4f] m\n', ...
                    p_fk(1), p_fk(2), p_fk(3));
            fprintf('  reconstruction err : %.3f mm\n', err_mm);

            num_success = num_success + 1;
            max_error_m = max(max_error_m, err_m);
        else
            fprintf('  FK error           : %s\n', fk_err);
        end
    else
        fprintf('  theta (deg)        : [n/a, n/a, n/a]\n');
        fprintf('  IK valid           : false\n');
        fprintf('  IK error           : %s\n', ik_err);
    end
end

fprintf('\n=== Summary ===\n');
fprintf('Successful points    : %d / %d\n', num_success, num_points);

if num_success > 0
    fprintf('Maximum recon. error : %.3f mm\n', 1e3 * max_error_m);
else
    fprintf('Maximum recon. error : n/a\n');
end
