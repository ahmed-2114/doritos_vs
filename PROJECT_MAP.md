# Project Map

Use this as the simple guide for what each folder is for.

## Learn First

- `doritos_matlab/main_files/` - core delta robot math.
  - `delta_params.m` stores geometry, limits, and the current homing convention.
  - `delta_IK.m` converts position to joint angles.
  - `delta_FK.m` converts joint angles to position.
  - `delta_command_to_model.m` and `delta_model_to_command.m` convert between the hardware command angles and the internal model angles.
- `doritos_matlab/test_fk_ik_consistency.m` - quick check that FK and IK still agree.
- `doritos_matlab/run_delta_analysis.m` - general workspace and singularity analysis runner.

## Use For The EE268 Setup

- `doritos_matlab/ee268_analysis/ee268_master_ui.m` - main UI.
- `doritos_matlab/ee268_analysis/run_ee268_analysis.m` - rebuilds the EE268 workspace results and plots.
- `doritos_matlab/ee268_analysis/configure_doritos_mk2_ui_bridge.m` - updates the Simulink model bridge and startup refs.
- `doritos_matlab/ee268_analysis/configure_doritos_mk2_pen_contact.m` - pen/contact model setup.

## Simulink And Robot Model

- `doritos_mk2/doritos_mk2.slx` - main Simulink/Simscape model.
- `doritos_mk2/urdf/` - exported robot URDF and CSV.
- `doritos_mk2/meshes/` - 3D mesh geometry used by the model/URDF.
- `doritos_mk2/config/` and `doritos_mk2/launch/` - ROS display and launch support.

## Trajectories And Drawing Data

- `writing.mat/` - writing scenarios used by the master UI, plus the raw generation files in `writing.mat/files/`.
- `doritos_matlab/generate_circle_signal_builder.m` - creates the circle trajectory dataset.
- `doritos_matlab/signal_builder_circle_test.mat` - reusable circle test scenario.

## Reference Docs

- `README.md` - quick start and project requirements.
- `MatlabModelDocumentation.pdf` - original model documentation.
- `Delta_LiveScript.mlx` - MATLAB live script notes/analysis.

## Can Ignore While Learning

- `slprj/`, `tmp_slx/`, `*.slxc`, logs, temporary reports, and backup models are generated files covered by `.gitignore`.
- They can be recreated by MATLAB/Simulink and are not where the project logic lives.
