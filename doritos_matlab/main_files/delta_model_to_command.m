function theta_command_deg = delta_model_to_command(params, theta_model_rad)
% DELTA_MODEL_TO_COMMAND Convert model radians to hardware/user command degrees.

theta_model_deg = rad2deg(theta_model_rad(:));
theta_command_deg = (theta_model_deg - params.command_zero_model_deg) ./ params.command_direction;
end
