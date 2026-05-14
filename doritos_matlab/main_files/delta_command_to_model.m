function theta_model_rad = delta_command_to_model(params, theta_command_deg)
% DELTA_COMMAND_TO_MODEL Convert hardware/user command degrees to model radians.

theta_command_deg = theta_command_deg(:);
theta_model_deg = params.command_zero_model_deg + params.command_direction * theta_command_deg;
theta_model_rad = deg2rad(theta_model_deg);
end
