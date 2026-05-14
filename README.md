# Doritos Delta Robot Analysis and Simulink Model

This repository contains the MATLAB analysis code, Simulink model, and ROS/URDF export assets for the Doritos MK2 delta robot project.

For a simpler learning-oriented folder guide, start with `PROJECT_MAP.md`.

## Contents

- `doritos_matlab/main_files/` - core delta robot kinematics and workspace functions:
  - `delta_params.m`
  - `delta_FK.m`
  - `delta_IK.m`
  - `delta_singularity.m`
  - `delta_workspace.m`
  - `delta_manipulability_space.m`
- `doritos_matlab/` - analysis and test entry points, including:
  - `run_delta_analysis.m`
  - `plot_workspace_surface.m`
  - `generate_circle_signal_builder.m`
  - `test_fk_ik_consistency.m`
  - `Delta_LiveScript.mlx`
- `doritos_matlab/ee268_analysis/` - EE268-specific analysis, UI, and Simulink bridge scripts.
- `doritos_mk2/` - Simulink model plus ROS package export files:
  - `doritos_mk2.slx`
  - `urdf/doritos_mk2.urdf`
  - `meshes/*.STL`
  - `launch/*.launch`
  - `config/joint_names_doritos_mk2.yaml`
- `writing.mat/` - Signal Editor scenario data and scripts for generating/converting signal files.
- `MatlabModelDocumentation.pdf` - project documentation.

## Requirements

- MATLAB with Simulink
- Simscape Multibody, if running the physical robot model
- Robotics System Toolbox or ROS tooling, if using the exported URDF package

## Quick Start

Open MATLAB at the repository root and add the project folders:

```matlab
addpath('doritos_matlab');
addpath('doritos_matlab/main_files');
addpath('doritos_matlab/ee268_analysis');
```

Run the main kinematics analysis:

```matlab
report = run_delta_analysis();
```

Run the FK/IK consistency check:

```matlab
test_fk_ik_consistency
```

Open the Simulink model:

```matlab
open_system('doritos_mk2/doritos_mk2.slx')
```

Run the EE268 analysis workflow:

```matlab
run_ee268_analysis
```

This script assigns `ee268_context` and `ee268_results` into the MATLAB base workspace.

Launch the interactive EE268 UI:

```matlab
app = ee268_master_ui();
```

## Generated Files

The repository intentionally ignores generated Simulink caches, unpacked model folders, autosaves, local logs, and temporary analysis outputs. These files are recreated by MATLAB/Simulink and should not be sent as source.

Important ignored examples:

- `slprj/`
- `tmp_slx/`
- `*.slxc`
- `*.slx.autosave`
- `*_output.txt`
- `doritos_matlab/ee268_analysis/*_results.mat`
- `doritos_mk2/doritos_mk2_[number].slx`

## Packaging Checklist

Before sending the project:

1. Open `doritos_mk2/doritos_mk2.slx` in MATLAB and confirm it loads.
2. Run `test_fk_ik_consistency`.
3. Run `run_delta_analysis` if fresh analysis outputs are needed.
4. Check `git status --short` and make sure only intentional source/model changes are staged.
