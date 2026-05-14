function app = ee268_master_ui(varargin)
% EE268_MASTER_UI  Interactive FK/IK and workspace viewer for the EE268 setup.
% Uses the generated ee268_analysis_results.mat as the workspace reference.

analysis_root = fileparts(mfilename('fullpath'));
project_root = fileparts(analysis_root);
addpath(project_root);
addpath(fullfile(project_root, 'main_files'));

results_file = fullfile(analysis_root, 'ee268_analysis_results.mat');
if ~isfile(results_file)
    error(['Missing ee268_analysis_results.mat. Run run_ee268_analysis.m first ', ...
           'from the ee268_analysis folder.']);
end

loaded = load(results_file, 'results');
results = loaded.results;
params = results.context.params;
workspace_root = fileparts(project_root);

visible = 'on';
if nargin >= 2
    for argIdx = 1:2:numel(varargin)
        if strcmpi(varargin{argIdx}, 'Visible')
            visible = varargin{argIdx + 1};
        end
    end
end

app = struct();
app.results = results;
app.params = params;
app.grid = build_reactive_grid(results);
app.homeThetaDeg = [0, 0, 0];
app.homeEE = results.context.ee_home(:).';
app.currentThetaDeg = app.homeThetaDeg;
app.currentEE = app.homeEE;
app.thetaLimitsDeg = [params.command_min_deg, params.command_max_deg];
app.thresholdLevels = [0.10, 0.20, 0.35, 0.50, 0.65, 0.80];
app.visual = buildVisualModel();
app.sim.modelPath = fullfile(project_root, '..', 'doritos_mk2', 'doritos_mk2.slx');
app.sim.holdDuration = 0.5;
app.sim.maxLoggedPoints = 2000;
app.sim.loggingDecimation = 20;
app.pickplace.outputFile = fullfile(analysis_root, 'ee268_pick_place_signal.mat');
app.pickplace.defaultPickCenterMm = [45, -45, -195];
app.pickplace.defaultPlaceCenterMm = [-45, 45, -195];
app.pickplace.defaultObjectDimsMm = [28, 28, 20];
app.pickplace.defaultClearanceMm = 35;
app.pickplace.defaultGripOpenDeg = 65;
app.pickplace.defaultGripClosedDeg = 18;
app.traj.preview = [];
app.trace.isRecording = false;
app.trace.points = zeros(0, 3);
app.scenarios = buildScenarioLibrary(project_root, workspace_root);
app.isUpdating = false;

app.fig = uifigure('Name', 'EE268 Master UI', ...
    'Position', [40 24 1780 1040], 'Visible', visible, 'Color', [0.98 0.98 0.99]);
main = uigridlayout(app.fig, [1 2]);
main.ColumnWidth = {'1.84x', 620};
main.RowHeight = {'1x'};

left = uigridlayout(main, [2 1]);
left.RowHeight = {'1x', 162};
left.ColumnWidth = {'1x'};
left.Padding = [8 8 8 8];
left.RowSpacing = 8;
left.Layout.Row = 1;
left.Layout.Column = 1;

app.ax = uiaxes(left);
app.ax.Layout.Row = 1;
title(app.ax, 'EE268 Relationship View');
xlabel(app.ax, 'X [mm]');
ylabel(app.ax, 'Y [mm]');
zlabel(app.ax, 'Z [mm]');
grid(app.ax, 'on');
axis(app.ax, 'equal');
view(app.ax, 45, 25);
app.ax.Box = 'on';

bottom = uigridlayout(left, [1 3]);
bottom.Layout.Row = 2;
bottom.ColumnWidth = {255, 165, '1x'};
bottom.Padding = [0 0 0 0];
bottom.ColumnSpacing = 6;

app.infoArea = uitextarea(bottom, 'Editable', 'off', ...
    'FontName', 'Consolas', 'FontSize', 10, ...
    'Value', {'Initializing...'});
app.infoArea.Layout.Row = 1;
app.infoArea.Layout.Column = 1;

togglePanel = uipanel(bottom, 'Title', 'Layer Visibility');
togglePanel.Layout.Row = 1;
togglePanel.Layout.Column = 2;
toggleGrid = uigridlayout(togglePanel, [7 1]);
toggleGrid.RowHeight = {19, 19, 19, 19, 19, 19, '1x'};
toggleGrid.Padding = [5 6 5 6];
toggleGrid.RowSpacing = 3;

app.chkReach = uicheckbox(toggleGrid, 'Text', 'Reachable cloud', 'Value', true, ...
    'ValueChangedFcn', @(~,~) refreshLayerVisibility());
app.chkManip = uicheckbox(toggleGrid, 'Text', 'Manipulability pass', 'Value', true, ...
    'ValueChangedFcn', @(~,~) refreshLayerVisibility());
app.chkClear = uicheckbox(toggleGrid, 'Text', 'Singularity-safe', 'Value', true, ...
    'ValueChangedFcn', @(~,~) refreshLayerVisibility());
app.chkBoth = uicheckbox(toggleGrid, 'Text', 'Passes both', 'Value', true, ...
    'ValueChangedFcn', @(~,~) refreshLayerVisibility());
app.chkRobot = uicheckbox(toggleGrid, 'Text', 'Robot overlay', 'Value', true, ...
    'ValueChangedFcn', @(~,~) refreshLayerVisibility());
app.chkBase = uicheckbox(toggleGrid, 'Text', 'Base / frame markers', 'Value', true, ...
    'ValueChangedFcn', @(~,~) refreshLayerVisibility());

right = uigridlayout(main, [3 1]);
right.ColumnWidth = {'1x'};
right.RowHeight = {292, 268, '1x'};
right.Padding = [8 8 8 8];
right.RowSpacing = 8;
right.Layout.Row = 1;
right.Layout.Column = 2;

xyzPanel = uipanel(right, 'Title', 'Desired EE Position');
xyzPanel.BackgroundColor = [1.00 0.97 0.97];
xyzPanel.Layout.Row = 1;
xyzGrid = uigridlayout(xyzPanel, [8 3]);
xyzGrid.RowHeight = {18, 34, 18, 34, 18, 34, 28, 18};
xyzGrid.ColumnWidth = {52, '1x', 76};
xyzGrid.Padding = [8 8 8 8];
xyzGrid.RowSpacing = 3;

app.lblX = uilabel(xyzGrid, 'Text', 'X', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.95 0.78 0.78], 'FontColor', [0.45 0.05 0.05]);
app.lblX.Layout.Row = 1; app.lblX.Layout.Column = 1;
app.sldX = uislider(xyzGrid, 'Limits', [0 1], 'ValueChangingFcn', @(src,evt) onXYZChanging('x', evt.Value), ...
    'ValueChangedFcn', @(src,evt) onXYZChanged('x', src.Value));
app.sldX.Layout.Row = 2; app.sldX.Layout.Column = 2;
app.xValue = uieditfield(xyzGrid, 'numeric', 'Editable', 'off', 'BackgroundColor', [1.00 0.94 0.94]);
app.xValue.Layout.Row = 2; app.xValue.Layout.Column = 3;

app.lblY = uilabel(xyzGrid, 'Text', 'Y', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.83 0.95 0.84], 'FontColor', [0.08 0.35 0.10]);
app.lblY.Layout.Row = 3; app.lblY.Layout.Column = 1;
app.sldY = uislider(xyzGrid, 'Limits', [0 1], 'ValueChangingFcn', @(src,evt) onXYZChanging('y', evt.Value), ...
    'ValueChangedFcn', @(src,evt) onXYZChanged('y', src.Value));
app.sldY.Layout.Row = 4; app.sldY.Layout.Column = 2;
app.yValue = uieditfield(xyzGrid, 'numeric', 'Editable', 'off', 'BackgroundColor', [0.95 1.00 0.95]);
app.yValue.Layout.Row = 4; app.yValue.Layout.Column = 3;

app.lblZ = uilabel(xyzGrid, 'Text', 'Z', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.80 0.89 0.98], 'FontColor', [0.06 0.22 0.55]);
app.lblZ.Layout.Row = 5; app.lblZ.Layout.Column = 1;
app.sldZ = uislider(xyzGrid, 'Limits', [0 1], 'ValueChangingFcn', @(src,evt) onXYZChanging('z', evt.Value), ...
    'ValueChangedFcn', @(src,evt) onXYZChanged('z', src.Value));
app.sldZ.Layout.Row = 6; app.sldZ.Layout.Column = 2;
app.zValue = uieditfield(xyzGrid, 'numeric', 'Editable', 'off', 'BackgroundColor', [0.93 0.97 1.00]);
app.zValue.Layout.Row = 6; app.zValue.Layout.Column = 3;

app.btnHome = uibutton(xyzGrid, 'push', 'Text', 'Go Home', ...
    'ButtonPushedFcn', @(~,~) goHome(), 'BackgroundColor', [0.90 0.15 0.15], ...
    'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnHome.Layout.Row = 7; app.btnHome.Layout.Column = [1 2];
app.homeLabel = uilabel(xyzGrid, 'Text', 'Start = home', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'BackgroundColor', [1.00 0.92 0.92]);
app.homeLabel.Layout.Row = 7; app.homeLabel.Layout.Column = 3;
app.xyzHint = uilabel(xyzGrid, 'Text', 'The red EE marker is the only EE point shown. Use Go Home anytime.', ...
    'FontAngle', 'italic');
app.xyzHint.Layout.Row = 8; app.xyzHint.Layout.Column = [1 3];

jointPanel = uipanel(right, 'Title', 'Joint Angles From FK / IK');
jointPanel.BackgroundColor = [0.96 0.98 1.00];
jointPanel.Layout.Row = 2;
jointGrid = uigridlayout(jointPanel, [6 3]);
jointGrid.RowHeight = {18, 38, 18, 38, 18, 38};
jointGrid.ColumnWidth = {54, '1x', 70};
jointGrid.Padding = [8 8 8 8];
jointGrid.RowSpacing = 4;

thLimitsDeg = app.thetaLimitsDeg;

app.lblT1 = uilabel(jointGrid, 'Text', 'th1', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'BackgroundColor', [1.00 0.88 0.78], 'FontColor', [0.50 0.22 0.00]);
app.lblT1.Layout.Row = 1; app.lblT1.Layout.Column = 1;
app.sldT1 = uislider(jointGrid, 'Limits', thLimitsDeg, 'ValueChangingFcn', @(src,evt) onThetaChanging(1, evt.Value), ...
    'ValueChangedFcn', @(src,evt) onThetaChanged(1, src.Value));
app.sldT1.Layout.Row = 2; app.sldT1.Layout.Column = 2;
app.t1Value = uieditfield(jointGrid, 'numeric', 'Editable', 'off', 'BackgroundColor', [1.00 0.96 0.90]); app.t1Value.Layout.Row = 2; app.t1Value.Layout.Column = 3;

app.lblT2 = uilabel(jointGrid, 'Text', 'th2', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.82 0.94 1.00], 'FontColor', [0.04 0.30 0.50]);
app.lblT2.Layout.Row = 3; app.lblT2.Layout.Column = 1;
app.sldT2 = uislider(jointGrid, 'Limits', thLimitsDeg, 'ValueChangingFcn', @(src,evt) onThetaChanging(2, evt.Value), ...
    'ValueChangedFcn', @(src,evt) onThetaChanged(2, src.Value));
app.sldT2.Layout.Row = 4; app.sldT2.Layout.Column = 2;
app.t2Value = uieditfield(jointGrid, 'numeric', 'Editable', 'off', 'BackgroundColor', [0.93 0.98 1.00]); app.t2Value.Layout.Row = 4; app.t2Value.Layout.Column = 3;

app.lblT3 = uilabel(jointGrid, 'Text', 'th3', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.90 0.84 0.99], 'FontColor', [0.34 0.05 0.58]);
app.lblT3.Layout.Row = 5; app.lblT3.Layout.Column = 1;
app.sldT3 = uislider(jointGrid, 'Limits', thLimitsDeg, 'ValueChangingFcn', @(src,evt) onThetaChanging(3, evt.Value), ...
    'ValueChangedFcn', @(src,evt) onThetaChanged(3, src.Value));
app.sldT3.Layout.Row = 6; app.sldT3.Layout.Column = 2;
app.t3Value = uieditfield(jointGrid, 'numeric', 'Editable', 'off', 'BackgroundColor', [0.98 0.94 1.00]); app.t3Value.Layout.Row = 6; app.t3Value.Layout.Column = 3;

controlPanel = uipanel(bottom, 'Title', 'Threshold Controls');
controlPanel.BackgroundColor = [0.99 0.99 0.95];
controlPanel.Layout.Row = 1;
controlPanel.Layout.Column = 3;
ctlGrid = uigridlayout(controlPanel, [4 2]);
ctlGrid.RowHeight = {18, 26, 18, 26};
ctlGrid.ColumnWidth = {'1x', 84};
ctlGrid.Padding = [8 8 8 8];
ctlGrid.RowSpacing = 4;
ctlGrid.ColumnSpacing = 8;

opsTabs = uitabgroup(right);
opsTabs.Layout.Row = 3;

trajTab = uitab(opsTabs, 'Title', 'Trajectory');
trajHost = uigridlayout(trajTab, [1 1]);
trajHost.Padding = [0 0 0 0];

manualTab = uitab(opsTabs, 'Title', 'Manual');
manualHost = uigridlayout(manualTab, [1 1]);
manualHost.Padding = [0 0 0 0];

traceTab = uitab(opsTabs, 'Title', 'Trace');
traceHost = uigridlayout(traceTab, [1 1]);
traceHost.Padding = [0 0 0 0];

pickPlaceTab = uitab(opsTabs, 'Title', 'Pick & Place');
pickPlaceHost = uigridlayout(pickPlaceTab, [1 1]);
pickPlaceHost.Padding = [0 0 0 0];

simTab = uitab(opsTabs, 'Title', 'Simulink');
simHost = uigridlayout(simTab, [1 1]);
simHost.Padding = [0 0 0 0];

uilabel(ctlGrid, 'Text', 'Manipulability threshold ratio');
app.manipThreshold = uieditfield(ctlGrid, 'numeric', 'Editable', 'off');
app.manipSlider = uislider(ctlGrid, 'Limits', [app.thresholdLevels(1) app.thresholdLevels(end)], ...
    'MajorTicks', app.thresholdLevels, ...
    'MajorTickLabels', compose('%.0f%%', 100 * app.thresholdLevels), ...
    'Value', nearestPreset(results.cfg.manip_threshold_ratio, app.thresholdLevels), ...
    'ValueChangingFcn', @(src,evt) onThresholdChanging('manip', evt.Value), ...
    'ValueChangedFcn', @(src,evt) onThresholdChanged('manip', src.Value));
app.manipSlider.Layout.Row = 2; app.manipSlider.Layout.Column = 1;
app.manipThreshold.Layout.Row = 2; app.manipThreshold.Layout.Column = 2;

uilabel(ctlGrid, 'Text', 'Singularity clearance ratio');
app.clearThreshold = uieditfield(ctlGrid, 'numeric', 'Editable', 'off');
app.clearSlider = uislider(ctlGrid, 'Limits', [app.thresholdLevels(1) app.thresholdLevels(end)], ...
    'MajorTicks', app.thresholdLevels, ...
    'MajorTickLabels', compose('%.0f%%', 100 * app.thresholdLevels), ...
    'Value', nearestPreset(results.cfg.clearance_threshold_ratio, app.thresholdLevels), ...
    'ValueChangingFcn', @(src,evt) onThresholdChanging('clear', evt.Value), ...
    'ValueChangedFcn', @(src,evt) onThresholdChanged('clear', src.Value));
app.clearSlider.Layout.Row = 4; app.clearSlider.Layout.Column = 1;
app.clearThreshold.Layout.Row = 4; app.clearThreshold.Layout.Column = 2;

trajPanel = uipanel(trajHost, 'Title', 'Trajectory Shape Library');
trajPanel.BackgroundColor = [0.96 0.99 1.00];
trajPanel.Layout.Row = 1;
trajGrid = uigridlayout(trajPanel, [7 3]);
trajGrid.RowHeight = {24, 30, 34, 34, 34, 34, '1x'};
trajGrid.ColumnWidth = {'1x', '1x', '1x'};
trajGrid.Padding = [8 8 8 8];

app.trajInfo = uilabel(trajGrid, 'Text', 'Build a validated EE path around the current pose, preview it on the axes, and publish it as workspace references.', ...
    'WordWrap', 'on');
app.trajInfo.Layout.Row = 1;
app.trajInfo.Layout.Column = [1 3];

app.trajShapeDrop = uidropdown(trajGrid, 'Items', {'circle', 'square', 'line-x', 'line-y', 'spiral'}, 'Value', 'circle');
app.trajShapeDrop.Layout.Row = 2;
app.trajShapeDrop.Layout.Column = 1;
app.trajShapeLabel = uilabel(trajGrid, 'Text', 'Shape', 'HorizontalAlignment', 'center');
app.trajShapeLabel.Layout.Row = 2;
app.trajShapeLabel.Layout.Column = 2;
app.trajShapeHint = uilabel(trajGrid, 'Text', 'Center = current EE pose', 'HorizontalAlignment', 'center');
app.trajShapeHint.Layout.Row = 2;
app.trajShapeHint.Layout.Column = 3;

app.trajSizeField = uieditfield(trajGrid, 'numeric', 'Limits', [5 150], 'Value', 40, 'RoundFractionalValues', false);
app.trajSizeField.Layout.Row = 3;
app.trajSizeField.Layout.Column = 1;
app.trajDurationField = uieditfield(trajGrid, 'numeric', 'Limits', [1 40], 'Value', 8, 'RoundFractionalValues', false);
app.trajDurationField.Layout.Row = 3;
app.trajDurationField.Layout.Column = 2;
app.trajTurnsField = uieditfield(trajGrid, 'numeric', 'Limits', [1 5], 'Value', 1, 'RoundFractionalValues', true);
app.trajTurnsField.Layout.Row = 3;
app.trajTurnsField.Layout.Column = 3;

app.trajSizeLabel = uilabel(trajGrid, 'Text', 'Size / radius [mm]', 'HorizontalAlignment', 'center');
app.trajSizeLabel.Layout.Row = 4;
app.trajSizeLabel.Layout.Column = 1;
app.trajDurationLabel = uilabel(trajGrid, 'Text', 'Duration [s]', 'HorizontalAlignment', 'center');
app.trajDurationLabel.Layout.Row = 4;
app.trajDurationLabel.Layout.Column = 2;
app.trajTurnsLabel = uilabel(trajGrid, 'Text', 'Turns / loops', 'HorizontalAlignment', 'center');
app.trajTurnsLabel.Layout.Row = 4;
app.trajTurnsLabel.Layout.Column = 3;

app.btnPreviewTraj = uibutton(trajGrid, 'push', 'Text', 'Preview Shape', ...
    'ButtonPushedFcn', @(~,~) previewTrajectoryShape(), ...
    'BackgroundColor', [0.12 0.48 0.84], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnPreviewTraj.Layout.Row = 5;
app.btnPreviewTraj.Layout.Column = 1;
app.btnPublishTraj = uibutton(trajGrid, 'push', 'Text', 'Publish Shape', ...
    'ButtonPushedFcn', @(~,~) publishTrajectoryShape(), ...
    'BackgroundColor', [0.10 0.58 0.30], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnPublishTraj.Layout.Row = 5;
app.btnPublishTraj.Layout.Column = 2;
app.btnClearTraj = uibutton(trajGrid, 'push', 'Text', 'Clear Preview', ...
    'ButtonPushedFcn', @(~,~) clearTrajectoryPreview(), ...
    'BackgroundColor', [0.72 0.24 0.24], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnClearTraj.Layout.Row = 5;
app.btnClearTraj.Layout.Column = 3;

app.scenarioDrop = uidropdown(trajGrid, 'Items', app.scenarios.labels, 'Value', app.scenarios.labels{1});
app.scenarioDrop.Layout.Row = 6;
app.scenarioDrop.Layout.Column = [1 2];
app.btnLoadScenario = uibutton(trajGrid, 'push', 'Text', 'Load Scenario', ...
    'ButtonPushedFcn', @(~,~) loadSelectedScenario(), ...
    'BackgroundColor', [0.48 0.32 0.76], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnLoadScenario.Layout.Row = 6;
app.btnLoadScenario.Layout.Column = 3;

app.trajStatus = uitextarea(trajGrid, 'Editable', 'off', 'FontName', 'Consolas', 'FontSize', 11, ...
    'Value', {'Trajectory library idle.'});
app.trajStatus.Layout.Row = 7;
app.trajStatus.Layout.Column = [1 3];

tracePanel = uipanel(traceHost, 'Title', 'Live Drawing Visualization');
tracePanel.BackgroundColor = [1.00 0.98 0.95];
tracePanel.Layout.Row = 1;
traceGrid = uigridlayout(tracePanel, [4 2]);
traceGrid.RowHeight = {24, 34, 34, '1x'};
traceGrid.ColumnWidth = {'1x', '1x'};
traceGrid.Padding = [8 8 8 8];

app.traceInfo = uilabel(traceGrid, 'Text', 'Record the EE path while jogging or moving the current target, then export it to the workspace.', ...
    'WordWrap', 'on');
app.traceInfo.Layout.Row = 1;
app.traceInfo.Layout.Column = [1 2];

app.btnTraceToggle = uibutton(traceGrid, 'push', 'Text', 'Start Trace', ...
    'ButtonPushedFcn', @(~,~) toggleTraceRecording(), ...
    'BackgroundColor', [0.12 0.55 0.82], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnTraceToggle.Layout.Row = 2;
app.btnTraceToggle.Layout.Column = 1;
app.btnTraceClear = uibutton(traceGrid, 'push', 'Text', 'Clear Trace', ...
    'ButtonPushedFcn', @(~,~) clearTrace(), ...
    'BackgroundColor', [0.72 0.24 0.24], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnTraceClear.Layout.Row = 2;
app.btnTraceClear.Layout.Column = 2;

app.btnTraceExport = uibutton(traceGrid, 'push', 'Text', 'Export Trace', ...
    'ButtonPushedFcn', @(~,~) exportTrace(), ...
    'BackgroundColor', [0.12 0.58 0.30], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnTraceExport.Layout.Row = 3;
app.btnTraceExport.Layout.Column = 1;
app.traceSamples = uilabel(traceGrid, 'Text', 'Samples: 0', 'HorizontalAlignment', 'center');
app.traceSamples.Layout.Row = 3;
app.traceSamples.Layout.Column = 2;

app.traceStatus = uitextarea(traceGrid, 'Editable', 'off', 'FontName', 'Consolas', 'FontSize', 11, ...
    'Value', {'Trace idle.'});
app.traceStatus.Layout.Row = 4;
app.traceStatus.Layout.Column = [1 2];

pickPlacePanel = uipanel(pickPlaceHost, 'Title', 'Pick And Place Generator');
pickPlacePanel.BackgroundColor = [0.97 0.97 1.00];
pickPlacePanel.Layout.Row = 1;
pickGrid = uigridlayout(pickPlacePanel, [9 3]);
pickGrid.RowHeight = {28, 18, 28, 18, 28, 18, 28, 30, 96};
pickGrid.ColumnWidth = {'1x', '1x', '1x'};
pickGrid.Padding = [8 8 8 8];
pickGrid.RowSpacing = 4;
pickGrid.ColumnSpacing = 8;

app.pickInfo = uilabel(pickGrid, 'Text', 'Create a MATLAB-only pick-and-place path with arm XYZ references, one shared gripper command, and a contact-hold signal. Positions are object centers in mm.', ...
    'WordWrap', 'on');
app.pickInfo.Layout.Row = 1;
app.pickInfo.Layout.Column = [1 3];

app.pickObjectLabel = uilabel(pickGrid, 'Text', 'Object center [X Y Z] mm', 'HorizontalAlignment', 'center');
app.pickObjectLabel.Layout.Row = 2;
app.pickObjectLabel.Layout.Column = [1 2];
app.pickObjectField = uieditfield(pickGrid, 'text', 'Value', vectorText(app.pickplace.defaultPickCenterMm));
app.pickObjectField.Layout.Row = 3;
app.pickObjectField.Layout.Column = [1 2];
app.pickDimsLabel = uilabel(pickGrid, 'Text', 'Object size [dX dY dZ] mm', 'HorizontalAlignment', 'center');
app.pickDimsLabel.Layout.Row = 2;
app.pickDimsLabel.Layout.Column = 3;
app.pickDimsField = uieditfield(pickGrid, 'text', 'Value', vectorText(app.pickplace.defaultObjectDimsMm));
app.pickDimsField.Layout.Row = 3;
app.pickDimsField.Layout.Column = 3;

app.placeObjectLabel = uilabel(pickGrid, 'Text', 'Place center [X Y Z] mm', 'HorizontalAlignment', 'center');
app.placeObjectLabel.Layout.Row = 4;
app.placeObjectLabel.Layout.Column = [1 2];
app.placeObjectField = uieditfield(pickGrid, 'text', 'Value', vectorText(app.pickplace.defaultPlaceCenterMm));
app.placeObjectField.Layout.Row = 5;
app.placeObjectField.Layout.Column = [1 2];
app.pickClearanceLabel = uilabel(pickGrid, 'Text', 'Approach clearance [mm]', 'HorizontalAlignment', 'center');
app.pickClearanceLabel.Layout.Row = 4;
app.pickClearanceLabel.Layout.Column = 3;
app.pickClearanceField = uieditfield(pickGrid, 'numeric', 'Limits', [5 120], 'Value', app.pickplace.defaultClearanceMm, 'RoundFractionalValues', false);
app.pickClearanceField.Layout.Row = 5;
app.pickClearanceField.Layout.Column = 3;

app.pickGripLabel = uilabel(pickGrid, 'Text', 'Gripper [open close] deg', 'HorizontalAlignment', 'center');
app.pickGripLabel.Layout.Row = 6;
app.pickGripLabel.Layout.Column = [1 2];
app.pickGripField = uieditfield(pickGrid, 'text', 'Value', vectorText([app.pickplace.defaultGripOpenDeg, app.pickplace.defaultGripClosedDeg]));
app.pickGripField.Layout.Row = 7;
app.pickGripField.Layout.Column = [1 2];
app.pickFileInfo = uilabel(pickGrid, 'Text', 'Output file: ee268_pick_place_signal.mat', 'HorizontalAlignment', 'center');
app.pickFileInfo.Layout.Row = 6;
app.pickFileInfo.Layout.Column = 3;
app.pickFileInfo2 = uilabel(pickGrid, 'Text', 'Contains arm, gripper, contact, and placement notes.', 'HorizontalAlignment', 'center', 'WordWrap', 'on');
app.pickFileInfo2.Layout.Row = 7;
app.pickFileInfo2.Layout.Column = 3;

app.btnPreviewPickPlace = uibutton(pickGrid, 'push', 'Text', 'Preview Pick/Place', ...
    'ButtonPushedFcn', @(~,~) previewPickPlaceMode(), ...
    'BackgroundColor', [0.12 0.48 0.84], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnPreviewPickPlace.Layout.Row = 8;
app.btnPreviewPickPlace.Layout.Column = 1;
app.btnPublishPickPlace = uibutton(pickGrid, 'push', 'Text', 'Publish Pick/Place', ...
    'ButtonPushedFcn', @(~,~) publishPickPlaceMode(), ...
    'BackgroundColor', [0.10 0.58 0.30], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnPublishPickPlace.Layout.Row = 8;
app.btnPublishPickPlace.Layout.Column = 2;
app.btnHomePickPlace = uibutton(pickGrid, 'push', 'Text', 'Use Current EE As Pick', ...
    'ButtonPushedFcn', @(~,~) seedPickPlaceFromCurrentEE(), ...
    'BackgroundColor', [0.60 0.44 0.16], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnHomePickPlace.Layout.Row = 8;
app.btnHomePickPlace.Layout.Column = 3;

app.pickStatus = uitextarea(pickGrid, 'Editable', 'off', 'FontName', 'Consolas', 'FontSize', 11, ...
    'Value', {'Pick/place generator idle.'});
app.pickStatus.Layout.Row = 9;
app.pickStatus.Layout.Column = [1 3];

simPanel = uipanel(simHost, 'Title', 'Simulink Bridge');
simPanel.BackgroundColor = [0.95 0.98 0.95];
simPanel.Layout.Row = 1;
simGrid = uigridlayout(simPanel, [6 2]);
simGrid.RowHeight = {24, 34, 34, 34, 34, '1x'};
simGrid.ColumnWidth = {'1x', '1x'};
simGrid.Padding = [8 8 8 8];

app.simInfo = uilabel(simGrid, 'Text', 'Push the current UI pose to the MATLAB base workspace or launch the MK2 model.', ...
    'WordWrap', 'on');
app.simInfo.Layout.Row = 1;
app.simInfo.Layout.Column = [1 2];

app.btnPublish = uibutton(simGrid, 'push', 'Text', 'Publish Pose', ...
    'ButtonPushedFcn', @(~,~) publishPoseAndRunHold(), ...
    'BackgroundColor', [0.14 0.56 0.26], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnPublish.Layout.Row = 2;
app.btnPublish.Layout.Column = 1;

app.btnOpenModel = uibutton(simGrid, 'push', 'Text', 'Open Model', ...
    'ButtonPushedFcn', @(~,~) openSimulinkModel(), ...
    'BackgroundColor', [0.20 0.36 0.72], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnOpenModel.Layout.Row = 2;
app.btnOpenModel.Layout.Column = 2;

app.btnRunHold = uibutton(simGrid, 'push', 'Text', 'Run Hold Test', ...
    'ButtonPushedFcn', @(~,~) runHoldSimulation(), ...
    'BackgroundColor', [0.78 0.48 0.08], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnRunHold.Layout.Row = 3;
app.btnRunHold.Layout.Column = 1;
app.btnRunCurrentTraj = uibutton(simGrid, 'push', 'Text', 'Run Current Path', ...
    'ButtonPushedFcn', @(~,~) runCurrentTrajectorySimulation(), ...
    'BackgroundColor', [0.52 0.24 0.72], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnRunCurrentTraj.Layout.Row = 3;
app.btnRunCurrentTraj.Layout.Column = 2;

app.chkAutoPublish = uicheckbox(simGrid, 'Text', 'Auto-publish on move', 'Value', false);
app.chkAutoPublish.Layout.Row = 5;
app.chkAutoPublish.Layout.Column = 1;

app.simDurationField = uieditfield(simGrid, 'numeric', 'Limits', [0.5 0.5], 'RoundFractionalValues', false, ...
    'Value', app.sim.holdDuration, 'LowerLimitInclusive', true, 'UpperLimitInclusive', true, 'Editable', 'off');
app.simDurationField.Layout.Row = 4;
app.simDurationField.Layout.Column = 1;
app.simDurationLabel = uilabel(simGrid, 'Text', 'Hold duration [s]', 'HorizontalAlignment', 'center');
app.simDurationLabel.Layout.Row = 4;
app.simDurationLabel.Layout.Column = 2;

app.simPathInfo = uilabel(simGrid, 'Text', 'Current path = latest preview or loaded scenario', 'HorizontalAlignment', 'center');
app.simPathInfo.Layout.Row = 5;
app.simPathInfo.Layout.Column = 2;

app.simStatus = uitextarea(simGrid, 'Editable', 'off', 'FontName', 'Consolas', 'FontSize', 11, ...
    'Value', {'Simulink bridge idle.'});
app.simStatus.Layout.Row = 6;
app.simStatus.Layout.Column = [1 2];

manualPanel = uipanel(manualHost, 'Title', 'Manual Command Mode');
manualPanel.BackgroundColor = [0.97 0.95 1.00];
manualPanel.Layout.Row = 1;
manualGrid = uigridlayout(manualPanel, [5 3]);
manualGrid.RowHeight = {24, 34, 42, 42, '1x'};
manualGrid.ColumnWidth = {'1x', '1x', '1x'};
manualGrid.Padding = [8 8 8 8];

app.manualInfo = uilabel(manualGrid, 'Text', 'Jog the current EE target in Cartesian space. Each command snaps to the nearest reachable EE sample.', ...
    'WordWrap', 'on');
app.manualInfo.Layout.Row = 1;
app.manualInfo.Layout.Column = [1 3];

app.manualStepField = uieditfield(manualGrid, 'numeric', 'Limits', [1 50], 'RoundFractionalValues', false, ...
    'Value', 5, 'LowerLimitInclusive', true, 'UpperLimitInclusive', true);
app.manualStepField.Layout.Row = 2;
app.manualStepField.Layout.Column = 1;
app.manualStepLabel = uilabel(manualGrid, 'Text', 'Jog step [mm]', 'HorizontalAlignment', 'center');
app.manualStepLabel.Layout.Row = 2;
app.manualStepLabel.Layout.Column = 2;
app.btnManualHome = uibutton(manualGrid, 'push', 'Text', 'Manual Home', ...
    'ButtonPushedFcn', @(~,~) manualGoHome(), ...
    'BackgroundColor', [0.70 0.16 0.16], 'FontColor', [1 1 1], 'FontWeight', 'bold');
app.btnManualHome.Layout.Row = 2;
app.btnManualHome.Layout.Column = 3;

app.btnXNeg = uibutton(manualGrid, 'push', 'Text', 'X-', ...
    'ButtonPushedFcn', @(~,~) manualJog('x', -1), 'BackgroundColor', [0.90 0.76 0.76], 'FontWeight', 'bold');
app.btnXNeg.Layout.Row = 3; app.btnXNeg.Layout.Column = 1;
app.btnYPos = uibutton(manualGrid, 'push', 'Text', 'Y+', ...
    'ButtonPushedFcn', @(~,~) manualJog('y', +1), 'BackgroundColor', [0.80 0.92 0.81], 'FontWeight', 'bold');
app.btnYPos.Layout.Row = 3; app.btnYPos.Layout.Column = 2;
app.btnXPos = uibutton(manualGrid, 'push', 'Text', 'X+', ...
    'ButtonPushedFcn', @(~,~) manualJog('x', +1), 'BackgroundColor', [0.90 0.76 0.76], 'FontWeight', 'bold');
app.btnXPos.Layout.Row = 3; app.btnXPos.Layout.Column = 3;

app.btnYNeg = uibutton(manualGrid, 'push', 'Text', 'Y-', ...
    'ButtonPushedFcn', @(~,~) manualJog('y', -1), 'BackgroundColor', [0.80 0.92 0.81], 'FontWeight', 'bold');
app.btnYNeg.Layout.Row = 4; app.btnYNeg.Layout.Column = 1;
app.btnZNeg = uibutton(manualGrid, 'push', 'Text', 'Z-', ...
    'ButtonPushedFcn', @(~,~) manualJog('z', -1), 'BackgroundColor', [0.81 0.88 0.98], 'FontWeight', 'bold');
app.btnZNeg.Layout.Row = 4; app.btnZNeg.Layout.Column = 2;
app.btnZPos = uibutton(manualGrid, 'push', 'Text', 'Z+', ...
    'ButtonPushedFcn', @(~,~) manualJog('z', +1), 'BackgroundColor', [0.81 0.88 0.98], 'FontWeight', 'bold');
app.btnZPos.Layout.Row = 4; app.btnZPos.Layout.Column = 3;

app.manualStatus = uitextarea(manualGrid, 'Editable', 'off', 'FontName', 'Consolas', 'FontSize', 11, ...
    'Value', {'Manual command mode idle.'});
app.manualStatus.Layout.Row = 5;
app.manualStatus.Layout.Column = [1 3];

initializePlotHandles();
recomputeThresholdMasks();
syncUIToTheta(app.currentThetaDeg, true);
publishSimulinkState(false);

    function initializePlotHandles()
        cla(app.ax);
        hold(app.ax, 'on');
        colors = struct('reach', [0.72 0.60 0.92], 'manip', [0.10 0.45 0.90], ...
            'clear', [0.95 0.50 0.10], 'both', [0.10 0.70 0.25]);

        app.hReach = scatter3(app.ax, nan, nan, nan, max(8, results.cfg.marker_size - 4), colors.reach, ...
            'filled', 'MarkerFaceAlpha', 0.30, 'MarkerEdgeAlpha', 0.18);
        app.hManip = scatter3(app.ax, nan, nan, nan, results.cfg.marker_size, colors.manip, ...
            'filled', 'MarkerFaceAlpha', 0.65, 'MarkerEdgeAlpha', 0.10);
        app.hClear = scatter3(app.ax, nan, nan, nan, results.cfg.marker_size, colors.clear, ...
            'filled', 'MarkerFaceAlpha', 0.45, 'MarkerEdgeAlpha', 0.10);
        app.hBoth = scatter3(app.ax, nan, nan, nan, results.cfg.marker_size + 4, colors.both, ...
            'filled', 'MarkerFaceAlpha', 0.90, 'MarkerEdgeAlpha', 0.15);
        app.hTrajPreview = plot3(app.ax, nan, nan, nan, '-', 'Color', [0.05 0.65 0.95], 'LineWidth', 2.4);
        app.hTrajStart = scatter3(app.ax, nan, nan, nan, 65, [0.05 0.65 0.95], 'filled', 's');
        app.hTrajEnd = scatter3(app.ax, nan, nan, nan, 65, [0.95 0.55 0.05], 'filled', 'h');
        app.hTrace = plot3(app.ax, nan, nan, nan, '-', ...
            'Color', [1.00 0.05 0.48], 'LineWidth', 3.6);

        app.hCurrent = scatter3(app.ax, nan, nan, nan, 120, [0.85 0.12 0.12], 'filled', 'o');
        app.hBase = scatter3(app.ax, 0, 0, 0, 95, 'k', 'filled', 'd');

        frameColor = [0.18 0.18 0.18];
        braceColor = [0.92 0.62 0.12];
        for postIdx = 1:3
            app.hFramePost(postIdx) = plot3(app.ax, nan, nan, nan, '-', 'Color', frameColor, 'LineWidth', 5.5); %#ok<AGROW>
            app.hFrameTop(postIdx) = plot3(app.ax, nan, nan, nan, '-', 'Color', frameColor, 'LineWidth', 4.2); %#ok<AGROW>
            app.hFrameFoot(postIdx) = scatter3(app.ax, nan, nan, nan, 80, braceColor, 'filled', 's'); %#ok<AGROW>
        end

        app.hBaseTri = plot3(app.ax, nan, nan, nan, '-', 'Color', [0.15 0.15 0.15], 'LineWidth', 2.4);
        app.hPlatformTri = plot3(app.ax, nan, nan, nan, '-', 'Color', [0.85 0.12 0.12], 'LineWidth', 2.2);
        app.hNeck = plot3(app.ax, nan, nan, nan, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 2.2);
        app.hTool = plot3(app.ax, nan, nan, nan, '-', 'Color', [0.55 0.00 0.00], 'LineWidth', 2.4);
        app.hTip = plot3(app.ax, nan, nan, nan, '-', 'Color', [0.05 0.05 0.05], 'LineWidth', 2.8);
        armColors = [0.90 0.45 0.15; 0.15 0.45 0.85; 0.55 0.20 0.80];
        for armIdx = 1:3
            app.hUpper(armIdx) = plot3(app.ax, nan, nan, nan, '-', 'Color', armColors(armIdx, :), 'LineWidth', 4.0); %#ok<AGROW>
            app.hFore(armIdx) = plot3(app.ax, nan, nan, nan, '-', 'Color', armColors(armIdx, :), 'LineWidth', 2.8, 'LineStyle', '--'); %#ok<AGROW>
        end
        app.hRobotJoints = scatter3(app.ax, nan, nan, nan, 36, [0.15 0.15 0.15], 'filled');
        app.hGripperTop = scatter3(app.ax, nan, nan, nan, 90, [0.10 0.95 0.15], 'filled', '^');
        app.hRotationPoint = scatter3(app.ax, nan, nan, nan, 75, [0.95 0.85 0.05], 'filled', 'hexagram');
        app.hTipPoint = scatter3(app.ax, nan, nan, nan, 80, [0.05 0.05 0.05], 'filled', 'v');
        app.hGuides = gobjects(1, 3);
        guideColors = [0.10 0.35 1.00; 0.10 0.80 0.10; 0.95 0.10 0.10];
        for guideIdx = 1:3
            app.hGuides(guideIdx) = plot3(app.ax, nan, nan, nan, ':', 'Color', guideColors(guideIdx, :), 'LineWidth', 1.5);
        end

        [xyLim, zLim] = computeAxisLimits();
        app.hX = plot3(app.ax, [xyLim(1), xyLim(2)], [0, 0], [0, 0], 'k--', 'LineWidth', 0.8);
        app.hY = plot3(app.ax, [0, 0], [xyLim(1), xyLim(2)], [0, 0], 'k--', 'LineWidth', 0.8);
        app.hZ = plot3(app.ax, [0, 0], [0, 0], [zLim(1), zLim(2)], 'k--', 'LineWidth', 0.8);

        xlim(app.ax, xyLim); ylim(app.ax, xyLim); zlim(app.ax, zLim);
           title(app.ax, {'EE268 Combined Workspace View', 'Measured stack: base->floor 383 mm | base->gripper top 180 mm | gripper top->pivot 32 mm | pivot->EE 55.18 mm | EE->tip 51.85 mm'});
           legend(app.ax, [app.hReach, app.hManip, app.hClear, app.hBoth, app.hTrajPreview, app.hTrace, app.hFramePost(1), app.hUpper(1), app.hFore(1), app.hGripperTop, app.hRotationPoint, app.hCurrent, app.hTipPoint, app.hX, app.hBase], ...
            {'Reachable cloud', 'Manipulability-passing', 'Singularity-safe', 'Passes both', ...
               'Trajectory preview', 'Live trace', 'Aluminium extrusions', 'Upper arms', 'Forearms / gripper top', 'Top of gripper', 'Tip rotation point', 'Current EE point', 'Tip end', 'Base-frame axes (X, Y, Z)', 'Base origin'}, 'Location', 'northeastoutside');
        hold(app.ax, 'off');
    end

    function [xyLim, zLim] = computeAxisLimits()
        ptsMm = results.reach_ee * 1e3;
        allPts = [ptsMm; 0, 0, 0; app.visual.frameVerticesMm; 0, 0, app.visual.floorZMm; 0, 0, min(ptsMm(:, 3)) - app.visual.tipOffsetMm];
        xyAbs = max(abs(allPts(:, 1:2)), [], 'all');
        xyAbs = max(xyAbs, 50);
        xyLim = [-1, 1] * (xyAbs + 0.08 * xyAbs);
        zMin = min(allPts(:, 3));
        zMax = max(allPts(:, 3));
        zMargin = max(20, 0.08 * (zMax - zMin + eps));
        zLim = [min(zMin - zMargin, -20), max(0, zMax + zMargin)];
    end

    function gridData = build_reactive_grid(resultsStruct)
        gridData = struct();
        gridData.allX = unique(resultsStruct.ee_points(:, 1));
        gridData.allY = unique(resultsStruct.ee_points(:, 2));
        gridData.allZ = unique(resultsStruct.ee_points(:, 3));
        gridData.reachX = unique(resultsStruct.reach_ee(:, 1));
        gridData.reachY = unique(resultsStruct.reach_ee(:, 2));
        gridData.reachZ = unique(resultsStruct.reach_ee(:, 3));
        gridData.tol = 1e-10;
    end

    function recomputeThresholdMasks()
        app.results.manip_threshold = app.manipSlider.Value * max(app.results.manip);
        app.results.clearance_threshold = app.clearSlider.Value * max(app.results.inv_cond);
        app.results.manip_keep = app.results.manip >= app.results.manip_threshold;
        app.results.clearance_keep = app.results.inv_cond >= app.results.clearance_threshold;
        app.results.both_keep = app.results.manip_keep & app.results.clearance_keep;

        set(app.hReach, 'XData', app.results.reach_ee(:, 1) * 1e3, 'YData', app.results.reach_ee(:, 2) * 1e3, 'ZData', app.results.reach_ee(:, 3) * 1e3);
        set(app.hManip, 'XData', app.results.reach_ee(app.results.manip_keep, 1) * 1e3, ...
            'YData', app.results.reach_ee(app.results.manip_keep, 2) * 1e3, ...
            'ZData', app.results.reach_ee(app.results.manip_keep, 3) * 1e3);
        set(app.hClear, 'XData', app.results.reach_ee(app.results.clearance_keep, 1) * 1e3, ...
            'YData', app.results.reach_ee(app.results.clearance_keep, 2) * 1e3, ...
            'ZData', app.results.reach_ee(app.results.clearance_keep, 3) * 1e3);
        set(app.hBoth, 'XData', app.results.reach_ee(app.results.both_keep, 1) * 1e3, ...
            'YData', app.results.reach_ee(app.results.both_keep, 2) * 1e3, ...
            'ZData', app.results.reach_ee(app.results.both_keep, 3) * 1e3);

        app.manipThreshold.Value = app.results.manip_threshold;
        app.clearThreshold.Value = app.results.clearance_threshold;
        refreshLayerVisibility();
    end

    function refreshLayerVisibility()
        app.hReach.Visible = onOff(app.chkReach.Value);
        app.hManip.Visible = onOff(app.chkManip.Value);
        app.hClear.Visible = onOff(app.chkClear.Value);
        app.hBoth.Visible = onOff(app.chkBoth.Value);
        app.hTrajPreview.Visible = 'on';
        app.hTrajStart.Visible = 'on';
        app.hTrajEnd.Visible = 'on';
        app.hTrace.Visible = 'on';
        robotState = onOff(app.chkRobot.Value);
        for postIdx = 1:numel(app.hFramePost)
            app.hFramePost(postIdx).Visible = robotState;
            app.hFrameTop(postIdx).Visible = robotState;
            app.hFrameFoot(postIdx).Visible = robotState;
        end
        app.hBaseTri.Visible = robotState;
        app.hPlatformTri.Visible = robotState;
        app.hNeck.Visible = robotState;
        app.hTool.Visible = robotState;
        app.hTip.Visible = robotState;
        app.hRobotJoints.Visible = robotState;
        app.hGripperTop.Visible = robotState;
        app.hRotationPoint.Visible = robotState;
        app.hTipPoint.Visible = robotState;
        for guideIdx = 1:numel(app.hGuides)
            app.hGuides(guideIdx).Visible = robotState;
        end
        for armIdx = 1:numel(app.hUpper)
            app.hUpper(armIdx).Visible = robotState;
            app.hFore(armIdx).Visible = robotState;
        end
        baseState = onOff(app.chkBase.Value);
        app.hBase.Visible = baseState;
        app.hX.Visible = baseState;
        app.hY.Visible = baseState;
        app.hZ.Visible = baseState;
    end

    function onThresholdChanging(whichOne, value)
        value = nearestPreset(value, app.thresholdLevels);
        if strcmp(whichOne, 'manip')
            app.manipThreshold.Value = value * max(app.results.manip);
        else
            app.clearThreshold.Value = value * max(app.results.inv_cond);
        end
    end

    function onThresholdChanged(whichOne, value)
        if app.isUpdating
            return
        end
        value = nearestPreset(value, app.thresholdLevels);
        if strcmp(whichOne, 'manip')
            app.manipSlider.Value = value;
        else
            app.clearSlider.Value = value;
        end
        recomputeThresholdMasks();
        updateInfoPanel();
    end

    function onXYZChanging(axisName, sliderValue)
        if app.isUpdating
            return
        end
        applyXYZSlider(axisName, sliderValue, false);
    end

    function onXYZChanged(axisName, sliderValue)
        if app.isUpdating
            return
        end
        applyXYZSlider(axisName, sliderValue, true);
    end

    function applyXYZSlider(axisName, sliderValue, commit)
        target = app.currentEE;
        switch axisName
            case 'x'
                target(1) = sliderValue;
            case 'y'
                target(2) = sliderValue;
            case 'z'
                target(3) = sliderValue;
        end
        snapToReachableEE(target, commit, [], axisName);
    end

    function onThetaChanging(thetaIdx, value)
        if app.isUpdating
            return
        end
        applyThetaSlider(thetaIdx, value, false);
    end

    function onThetaChanged(thetaIdx, value)
        if app.isUpdating
            return
        end
        applyThetaSlider(thetaIdx, value, true);
    end

    function applyThetaSlider(thetaIdx, value, commit)
        thetaCommandDeg = app.currentThetaDeg;
        thetaCommandDeg(thetaIdx) = value;
        thetaModelRad = delta_command_to_model(app.params, thetaCommandDeg(:));
        [platformPoint, fkOk, fkMsg] = delta_FK(app.params, thetaModelRad);
        if ~fkOk
            updateInfoPanel(['FK invalid: ' fkMsg]);
            return
        end

        eePoint = (platformPoint + app.results.context.tool_offset_base)';
        syncUIToTheta(thetaCommandDeg, commit, eePoint);
    end

    function syncUIToTheta(thetaCommandDeg, commit, eePoint)
        if nargin < 3
            thetaModelRad = delta_command_to_model(app.params, thetaCommandDeg(:));
            [platformPoint, fkOk, fkMsg] = delta_FK(app.params, thetaModelRad);
            if ~fkOk
                updateInfoPanel(['FK invalid: ' fkMsg]);
                return
            end
            eePoint = (platformPoint + app.results.context.tool_offset_base)';
        end

        if nargin < 2
            commit = true;
        end

        snapToReachableEE(eePoint, commit, thetaCommandDeg);
    end

    function snapToReachableEE(targetEE, commit, thetaCommandDegOverride, primaryAxis)
        if nargin < 3
            thetaCommandDegOverride = [];
        end
        if nargin < 4
            primaryAxis = '';
        end

        app.isUpdating = true;
        if isempty(thetaCommandDegOverride)
            if isempty(primaryAxis)
                targetEE = snapCoordinate(targetEE, 'x');
                targetEE = snapCoordinate(targetEE, 'y');
                targetEE = snapCoordinate(targetEE, 'z');
                [targetEE, nearestIdx] = findNearestReachable(targetEE);
            else
                [targetEE, nearestIdx] = findNearestReachableConstrained(targetEE, primaryAxis);
            end
        else
            [~, nearestIdx] = findNearestReachable(targetEE);
        end
        app.currentEE = targetEE;

        platformPoint = targetEE(:) - app.results.context.tool_offset_base;
        if ~isempty(thetaCommandDegOverride)
            thetaCommandDeg = thetaCommandDegOverride;
        else
            [thetaIK, ikOk] = delta_IK(app.params, platformPoint);
            if ikOk
                thetaCommandDeg = delta_model_to_command(app.params, thetaIK(:)).';
            else
                thetaCommandDeg = delta_model_to_command(app.params, app.results.reach_theta(nearestIdx, :).').';
            end
        end
        if isempty(thetaCommandDegOverride) && ~exist('ikOk', 'var')
            ikOk = true;
        end
        if exist('ikOk', 'var') && ~ikOk && isempty(thetaCommandDegOverride)
            thetaCommandDeg = delta_model_to_command(app.params, app.results.reach_theta(nearestIdx, :).').';
        end
        thetaCommandDeg = clampThetaDeg(thetaCommandDeg);
        app.currentThetaDeg = thetaCommandDeg;

        updateSliderLimits(targetEE);

        app.sldX.Value = targetEE(1); app.xValue.Value = targetEE(1) * 1e3;
        app.sldY.Value = targetEE(2); app.yValue.Value = targetEE(2) * 1e3;
        app.sldZ.Value = targetEE(3); app.zValue.Value = targetEE(3) * 1e3;
        app.sldT1.Value = thetaCommandDeg(1); app.t1Value.Value = thetaCommandDeg(1);
        app.sldT2.Value = thetaCommandDeg(2); app.t2Value.Value = thetaCommandDeg(2);
        app.sldT3.Value = thetaCommandDeg(3); app.t3Value.Value = thetaCommandDeg(3);

        set(app.hCurrent, 'XData', targetEE(1) * 1e3, 'YData', targetEE(2) * 1e3, 'ZData', targetEE(3) * 1e3);
        updateRobotOverlay(thetaCommandDeg);
        updateInfoPanel();
        maybeRecordTrace(targetEE, commit);
        maybeAutoPublishState();

        if commit
            drawnow limitrate;
        end
        app.isUpdating = false;
    end

    function pointOut = snapCoordinate(pointIn, primaryAxis)
        pointOut = pointIn;
        switch primaryAxis
            case 'x'
                pointOut(1) = nearestValue(pointIn(1), app.grid.reachX);
            case 'y'
                pointOut(2) = nearestValue(pointIn(2), app.grid.reachY);
            case 'z'
                pointOut(3) = nearestValue(pointIn(3), app.grid.reachZ);
        end
    end

    function [pointSnap, idxNearest] = findNearestReachable(pointIn)
        diffs = app.results.reach_ee - pointIn;
        dist2 = sum(diffs.^2, 2);
        [~, idxNearest] = min(dist2);
        pointSnap = app.results.reach_ee(idxNearest, :);
    end

    function pointSnap = findNearestReachableWithZFloor(pointIn, minZ)
        candidates = app.results.reach_ee;
        zMask = candidates(:, 3) >= (minZ - app.grid.tol);
        if any(zMask)
            candidates = candidates(zMask, :);
        end

        diffs = candidates - pointIn;
        dist2 = sum(diffs.^2, 2);
        [~, idxNearest] = min(dist2);
        pointSnap = candidates(idxNearest, :);
    end

    function [xProj, yProj, zProj] = projectTrajectoryToReachable(xIn, yIn, zIn)
        desiredPath = [xIn(:), yIn(:), zIn(:)];
        projectedPath = zeros(size(desiredPath));

        for sampleIdx = 1:size(desiredPath, 1)
            projectedPath(sampleIdx, :) = findNearestReachable(desiredPath(sampleIdx, :));
        end

        xProj = projectedPath(:, 1);
        yProj = projectedPath(:, 2);
        zProj = projectedPath(:, 3);
    end

    function [pointSnap, idxNearest] = findNearestReachableConstrained(pointIn, primaryAxis)
        candidates = app.results.reach_ee;
        targetSnap = pointIn;
        axisNames = 'xyz';
        for axisIdx = 1:numel(axisNames)
            axisName = axisNames(axisIdx);
            if axisName == primaryAxis
                continue
            end
            coordIdx = axisIndex(axisName);
            allowedVals = unique(candidates(:, coordIdx));
            targetSnap(coordIdx) = nearestValue(pointIn(coordIdx), allowedVals);
            axisMask = abs(candidates(:, coordIdx) - targetSnap(coordIdx)) <= app.grid.tol;
            if any(axisMask)
                candidates = candidates(axisMask, :);
            end
        end

        freeIdx = axisIndex(primaryAxis);
        allowedVals = unique(candidates(:, freeIdx));
        if ~isempty(allowedVals)
            targetSnap(freeIdx) = nearestValue(pointIn(freeIdx), allowedVals);
        end

        diffs = candidates - targetSnap;
        dist2 = sum(diffs.^2, 2);
        [~, localIdx] = min(dist2);
        pointSnap = candidates(localIdx, :);

        diffsAll = app.results.reach_ee - pointSnap;
        dist2All = sum(diffsAll.^2, 2);
        [~, idxNearest] = min(dist2All);
    end

    function updateSliderLimits(targetEE)
        xAllowed = getAllowedAxisValues(targetEE, 'x');
        yAllowed = getAllowedAxisValues(targetEE, 'y');
        zAllowed = getAllowedAxisValues(targetEE, 'z');

        xLimits = [min(xAllowed), max(xAllowed)];
        yLimits = [min(yAllowed), max(yAllowed)];
        zLimits = [min(zAllowed), max(zAllowed)];

        % Exact FK/home poses can fall between sampled workspace slices.
        % Keep the reactive limits, but always include the current target.
        xLimits = [min(xLimits(1), targetEE(1)), max(xLimits(2), targetEE(1))];
        yLimits = [min(yLimits(1), targetEE(2)), max(yLimits(2), targetEE(2))];
        zLimits = [min(zLimits(1), targetEE(3)), max(zLimits(2), targetEE(3))];

        app.sldX.Limits = xLimits;
        app.sldY.Limits = yLimits;
        app.sldZ.Limits = zLimits;
        updateSliderTicks(app.sldX);
        updateSliderTicks(app.sldY);
        updateSliderTicks(app.sldZ);
    end

    function allowedVals = getAllowedAxisValues(targetEE, axisName)
        candidates = app.results.reach_ee;
        axisNames = 'xyz';
        for axisIdx = 1:numel(axisNames)
            otherAxis = axisNames(axisIdx);
            if otherAxis == axisName
                continue
            end
            coordIdx = axisIndex(otherAxis);
            coordVals = unique(candidates(:, coordIdx));
            coordSnap = nearestValue(targetEE(coordIdx), coordVals);
            axisMask = abs(candidates(:, coordIdx) - coordSnap) <= app.grid.tol;
            if any(axisMask)
                candidates = candidates(axisMask, :);
            end
        end

        allowedVals = unique(candidates(:, axisIndex(axisName)));
        if isempty(allowedVals)
            allowedVals = app.grid.(['reach' upper(axisName)]);
        end
    end

    function idx = axisIndex(axisName)
        switch axisName
            case 'x'
                idx = 1;
            case 'y'
                idx = 2;
            case 'z'
                idx = 3;
            otherwise
                error('Unknown axis: %s', axisName);
        end
    end

    function updateInfoPanel(extraMessage)
        if nargin < 1
            extraMessage = '';
        end
        platformPoint = app.currentEE(:) - app.results.context.tool_offset_base;
        [thetaModel, ikOk, ikMsg] = delta_IK(app.params, platformPoint);
        if ikOk
            thetaDisplay = delta_model_to_command(app.params, thetaModel(:)).';
            thetaModelDisplay = rad2deg(thetaModel(:)).';
            [~, ~, type1, type2, info] = delta_singularity(app.params, thetaModel);
            manipVal = prod(svd((info.n_vecs'' * 0 + eye(3)))); %#ok<NASGU>
            % Use the nearest sampled point for stable displayed metrics.
            [~, idxNearest] = findNearestReachable(app.currentEE);
            manipVal = app.results.manip(idxNearest);
            invCondVal = app.results.inv_cond(idxNearest);
            jointMargin = app.results.joint_margin(idxNearest);
        else
            thetaDisplay = app.currentThetaDeg;
            thetaModelDisplay = rad2deg(delta_command_to_model(app.params, thetaDisplay(:))).';
            type1 = false;
            type2 = false;
            manipVal = NaN;
            invCondVal = NaN;
            jointMargin = NaN;
            ikMsg = ['invalid: ' ikMsg];
        end

        lines = {
            sprintf('EE XYZ [mm]     : [%7.1f, %7.1f, %7.1f]', app.currentEE(1) * 1e3, app.currentEE(2) * 1e3, app.currentEE(3) * 1e3)
            sprintf('Platform XYZ [mm]: [%7.1f, %7.1f, %7.1f]', platformPoint(1) * 1e3, platformPoint(2) * 1e3, platformPoint(3) * 1e3)
            sprintf('Gripper top Z    : %7.2f mm', app.currentEE(3) * 1e3 + app.visual.gripperTopOffsetMm)
            sprintf('Rotation point Z : %7.2f mm', app.currentEE(3) * 1e3 + app.visual.rotationPointOffsetMm)
            sprintf('Tip end Z        : %7.2f mm', app.currentEE(3) * 1e3 - app.visual.tipOffsetMm)
            sprintf('Theta cmd [deg]  : [%7.2f, %7.2f, %7.2f]', thetaDisplay(1), thetaDisplay(2), thetaDisplay(3))
            sprintf('Theta model [deg]: [%7.2f, %7.2f, %7.2f]', thetaModelDisplay(1), thetaModelDisplay(2), thetaModelDisplay(3))
            sprintf('Manipulability   : %.5f | threshold %.5f', manipVal, app.results.manip_threshold)
            sprintf('1 / cond(Jx)     : %.5f | threshold %.5f', invCondVal, app.results.clearance_threshold)
            sprintf('Joint margin     : %.5f', jointMargin)
            sprintf('Near singularity : type1=%s, type2=%s', mat2str(type1), mat2str(type2))
            sprintf('IK state         : %s', ternary(ikOk, 'valid', ikMsg))
            ' '
            'Relationship meaning:'
            '  Purple = everything IK can reach'
            '  Blue   = passes manipulability threshold'
            '  Orange = passes singularity clearance threshold'
            '  Green  = passes both thresholds'
        };
        if ~isempty(extraMessage)
            lines{end + 1} = ['Note: ' extraMessage];
        end
        app.infoArea.Value = lines;
    end

    function valueOut = nearestValue(valueIn, candidates)
        [~, idxMin] = min(abs(candidates - valueIn));
        valueOut = candidates(idxMin);
    end

    function valueOut = nearestPreset(valueIn, presets)
        [~, idxMin] = min(abs(presets - valueIn));
        valueOut = presets(idxMin);
    end

    function thetaDeg = clampThetaDeg(thetaDeg)
        thetaDeg = max(app.thetaLimitsDeg(1), min(app.thetaLimitsDeg(2), thetaDeg));
    end

    function goHome()
        syncUIToTheta(app.homeThetaDeg, true);
    end

    function maybeAutoPublishState()
        if app.chkAutoPublish.Value
            publishSimulinkState(false);
        end
    end

    function manualGoHome()
        goHome();
        updateManualStatus({ ...
            'Manual command executed: HOME', ...
            sprintf('EE [mm]: [%.1f %.1f %.1f]', app.currentEE(1) * 1e3, app.currentEE(2) * 1e3, app.currentEE(3) * 1e3) ...
            });
    end

    function manualJog(axisName, direction)
        stepMm = max(1, app.manualStepField.Value);
        stepM = stepMm / 1e3;
        target = app.currentEE;

        switch axisName
            case 'x'
                target(1) = target(1) + direction * stepM;
            case 'y'
                target(2) = target(2) + direction * stepM;
            case 'z'
                target(3) = target(3) + direction * stepM;
        end

        snapToReachableEE(target, true);
        updateManualStatus({ ...
            sprintf('Manual jog: %s %+0.1f mm', upper(axisName), direction * stepMm), ...
            sprintf('Reached EE [mm]: [%.1f %.1f %.1f]', app.currentEE(1) * 1e3, app.currentEE(2) * 1e3, app.currentEE(3) * 1e3), ...
            sprintf('Theta cmd [deg]: [%.2f %.2f %.2f]', app.currentThetaDeg(1), app.currentThetaDeg(2), app.currentThetaDeg(3)) ...
            });
    end

    function previewTrajectoryShape()
        try
            trajState = buildTrajectoryShapeState();
            applyTrajectoryPreview(trajState);
            updateTrajectoryStatus({ ...
                sprintf('Preview ready: %s', trajState.name), ...
                sprintf('Samples     : %d', numel(trajState.t)), ...
                sprintf('Center [mm] : [%.1f %.1f %.1f]%s', trajState.center(1) * 1e3, trajState.center(2) * 1e3, trajState.center(3) * 1e3, ternary(trajState.center_adjusted, '  (auto-shifted to safe point)', '')), ...
                sprintf('Size [mm]   : %.1f', trajState.size_mm), ...
                sprintf('Duration [s]: %.2f', trajState.duration_s), ...
                sprintf('Worst cond  : %.3f', trajState.worst_cond_Jx) ...
                });
        catch trajErr
            updateTrajectoryStatus({'Preview failed.', trajErr.message});
        end
    end

    function publishTrajectoryShape()
        try
            trajState = buildTrajectoryShapeState();
            applyTrajectoryPreview(trajState);
            publishTrajectorySignals(trajState, 'ee268_ui_traj', 'ee268_ui_traj_scenario');
            assignin('base', ['ee268_ui_' trajState.name '_traj'], trajState);
            updateTrajectoryStatus({ ...
                sprintf('Published trajectory: %s', trajState.name), ...
                sprintf('Center [mm]  : [%.1f %.1f %.1f]', trajState.center(1) * 1e3, trajState.center(2) * 1e3, trajState.center(3) * 1e3), ...
                ternary(trajState.center_adjusted, 'Center source : auto-shifted to safe point', 'Center source : current EE pose'), ...
                sprintf('Size [mm]    : %.1f', trajState.size_mm), ...
                sprintf('Duration [s] : %.2f', trajState.duration_s), ...
                'Workspace vars: ee268_ui_traj, x_ref_ts, y_ref_ts, z_ref_ts' ...
                });
        catch trajErr
            updateTrajectoryStatus({'Publish failed.', trajErr.message});
        end
    end

    function clearTrajectoryPreview()
        app.traj.preview = [];
        set(app.hTrajPreview, 'XData', nan, 'YData', nan, 'ZData', nan);
        set(app.hTrajStart, 'XData', nan, 'YData', nan, 'ZData', nan);
        set(app.hTrajEnd, 'XData', nan, 'YData', nan, 'ZData', nan);
        updateTrajectoryStatus({'Trajectory preview cleared.'});
    end

    function loadSelectedScenario()
        try
            label = app.scenarioDrop.Value;
            scenarioIdx = find(strcmp(app.scenarios.labels, label), 1, 'first');
            if isempty(scenarioIdx)
                error('Scenario label not found.');
            end

            scenarioPath = app.scenarios.paths{scenarioIdx};
            loadedScenario = loadScenarioFile(scenarioPath);
            applyTrajectoryPreview(loadedScenario);
            publishTrajectorySignals(loadedScenario, 'ee268_ui_loaded_scenario', 'ee268_ui_loaded_dataset');
            updateTrajectoryStatus({ ...
                sprintf('Loaded scenario: %s', label), ...
                sprintf('Samples       : %d', numel(loadedScenario.t)), ...
                sprintf('Source        : %s', scenarioPath), ...
                'Workspace vars: ee268_ui_loaded_scenario, x_ref_ts, y_ref_ts, z_ref_ts' ...
                });
        catch scenarioErr
            updateTrajectoryStatus({'Scenario load failed.', scenarioErr.message});
        end
    end

    function previewPickPlaceMode()
        try
            pickState = buildPickPlaceTrajectoryState();
            applyTrajectoryPreview(pickState);
            updatePickPlaceStatus(formatPickPlaceStatus(pickState, false));
        catch pickErr
            updatePickPlaceStatus({'Pick/place preview failed.', pickErr.message});
        end
    end

    function publishPickPlaceMode()
        try
            pickState = buildPickPlaceTrajectoryState();
            applyTrajectoryPreview(pickState);
            publishTrajectorySignals(pickState, 'ee268_ui_pick_place', 'ee268_ui_pick_place_dataset');
            assignin('base', 'ee268_ui_pick_place_notes', pickState.simulink_notes);
            assignin('base', 'ee268_ui_pick_place_transforms', pickState.rigid_transforms);
            outputFile = app.pickplace.outputFile;
            scenario = pickState.scenario; %#ok<NASGU>
            rigid_transforms = pickState.rigid_transforms; %#ok<NASGU>
            simulink_notes = pickState.simulink_notes; %#ok<NASGU>
            save(outputFile, 'pickState', 'scenario', 'rigid_transforms', 'simulink_notes');
            updatePickPlaceStatus(formatPickPlaceStatus(pickState, true));
        catch pickErr
            updatePickPlaceStatus({'Pick/place publish failed.', pickErr.message});
        end
    end

    function seedPickPlaceFromCurrentEE()
        currentMm = app.currentEE(:).' * 1e3;
        placeMm = currentMm + [-60, 60, 0];
        app.pickObjectField.Value = vectorText(currentMm);
        app.placeObjectField.Value = vectorText(placeMm);
        updatePickPlaceStatus({ ...
            'Pick/place fields seeded from current EE.', ...
            sprintf('Pick  [mm]: %s', vectorText(currentMm)), ...
            sprintf('Place [mm]: %s', vectorText(placeMm)) ...
            });
    end

    function toggleTraceRecording()
        app.trace.isRecording = ~app.trace.isRecording;
        if app.trace.isRecording
            app.btnTraceToggle.Text = 'Stop Trace';
            app.btnTraceToggle.BackgroundColor = [0.72 0.18 0.18];
            if isempty(app.trace.points)
                maybeRecordTrace(app.currentEE, true);
            end
            updateTraceStatus({'Trace recording ON.', sprintf('Start point [mm]: [%.1f %.1f %.1f]', app.currentEE(1) * 1e3, app.currentEE(2) * 1e3, app.currentEE(3) * 1e3)});
        else
            app.btnTraceToggle.Text = 'Start Trace';
            app.btnTraceToggle.BackgroundColor = [0.12 0.55 0.82];
            updateTraceStatus({'Trace recording OFF.', sprintf('Samples stored: %d', size(app.trace.points, 1))});
        end
    end

    function maybeRecordTrace(targetEE, commit)
        if ~commit || ~app.trace.isRecording
            return
        end

        point = targetEE(:).';
        if isempty(app.trace.points)
            app.trace.points = point;
        elseif norm(app.trace.points(end, :) - point) > 1e-6
            app.trace.points(end + 1, :) = point; %#ok<AGROW>
        end
        refreshTracePlot();
    end

    function refreshTracePlot()
        if isempty(app.trace.points)
            set(app.hTrace, 'XData', nan, 'YData', nan, 'ZData', nan);
            app.traceSamples.Text = 'Samples: 0';
            drawnow limitrate;
            return
        end
        set(app.hTrace, 'XData', app.trace.points(:, 1) * 1e3, 'YData', app.trace.points(:, 2) * 1e3, 'ZData', app.trace.points(:, 3) * 1e3);
        app.traceSamples.Text = sprintf('Samples: %d', size(app.trace.points, 1));
        uistack(app.hTrace, 'top');
        drawnow limitrate;
    end

    function clearTrace()
        app.trace.points = zeros(0, 3);
        refreshTracePlot();
        updateTraceStatus({'Trace cleared.'});
    end

    function exportTrace()
        if isempty(app.trace.points)
            updateTraceStatus({'Export failed.', 'No trace points recorded.'});
            return
        end

        points = app.trace.points;
        if size(points, 1) == 1
            points = [points; points];
        end
        t = linspace(0, max(1, size(points, 1) - 1), size(points, 1)).';
        traceState = struct();
        traceState.points = points;
        traceState.t = t;
        traceState.x_ts = timeseries(points(:, 1), t);
        traceState.y_ts = timeseries(points(:, 2), t);
        traceState.z_ts = timeseries(points(:, 3), t);
        scenario = Simulink.SimulationData.Dataset;
        scenario = scenario.addElement(traceState.x_ts, 'x');
        scenario = scenario.addElement(traceState.y_ts, 'y');
        scenario = scenario.addElement(traceState.z_ts, 'z');
        traceState.scenario = scenario;
        assignin('base', 'ee268_ui_trace', traceState);
        assignin('base', 'ee268_ui_trace_scenario', scenario);
        updateTraceStatus({ ...
            'Trace exported.', ...
            sprintf('Samples      : %d', size(points, 1)), ...
            'Workspace vars: ee268_ui_trace, ee268_ui_trace_scenario' ...
            });
    end

    function publishSimulinkState(showStatus)
        if nargin < 1
            showStatus = true;
        end

        simState = buildSimulinkBridgeState();
        assignin('base', 'ee268_ui_state', simState);
        assignin('base', 'ee268_ui_xyz', simState.ee_xyz_m);
        assignin('base', 'ee268_ui_theta_deg', simState.theta_deg);
        assignin('base', 'ee268_ui_theta_rad', simState.theta_rad);
        assignin('base', 'ee268_ui_theta_model_deg', simState.theta_model_deg);
        assignin('base', 'ee268_ui_theta_model_rad', simState.theta_model_rad);
        assignin('base', 'ee268_ui_x_ref_ts', simState.x_ref_ts);
        assignin('base', 'ee268_ui_y_ref_ts', simState.y_ref_ts);
        assignin('base', 'ee268_ui_z_ref_ts', simState.z_ref_ts);
        assignin('base', 'ee268_ui_hold_scenario', simState.scenario);

        % Also publish the generic reference names expected by many From Workspace blocks.
        assignin('base', 'x_ref_ts', simState.x_ref_ts);
        assignin('base', 'y_ref_ts', simState.y_ref_ts);
        assignin('base', 'z_ref_ts', simState.z_ref_ts);

        forceModelToUISource();

        if showStatus
            updateSimStatus({ ...
                'Published current UI pose to base workspace.', ...
                sprintf('EE [m]    : [%.4f %.4f %.4f]', simState.ee_xyz_m(1), simState.ee_xyz_m(2), simState.ee_xyz_m(3)), ...
                sprintf('Theta cmd [deg]: [%.2f %.2f %.2f]', simState.theta_deg(1), simState.theta_deg(2), simState.theta_deg(3)), ...
                sprintf('Hold refs  : x_ref_ts, y_ref_ts, z_ref_ts for %.2f s', simState.hold_duration_s) ...
                });
        end
    end

    function publishPoseAndRunHold()
        publishSimulinkState(false);
        runHoldSimulation();
    end

    function openSimulinkModel()
        if ~isfile(app.sim.modelPath)
            updateSimStatus({'Model file not found.', app.sim.modelPath});
            return
        end

        publishSimulinkState(false);
        open_system(app.sim.modelPath);
        forceModelToUISource();
        updateSimStatus({'Model opened and current UI pose published.', app.sim.modelPath});
    end

    function runHoldSimulation()
        if ~isfile(app.sim.modelPath)
            updateSimStatus({'Model file not found.', app.sim.modelPath});
            return
        end

        simState = buildSimulinkBridgeState();
        assignin('base', 'ee268_ui_state', simState);
        assignin('base', 'x_ref_ts', simState.x_ref_ts);
        assignin('base', 'y_ref_ts', simState.y_ref_ts);
        assignin('base', 'z_ref_ts', simState.z_ref_ts);

        try
            load_system(app.sim.modelPath);
            modelName = getModelName();
            forceModelToUISource();
            simGuard = configureLowMemorySimulation(modelName, simState.hold_duration_s); %#ok<NASGU>
            sim(modelName);
            updateSimStatus({ ...
                'Hold simulation completed.', ...
                sprintf('Model      : %s', modelName), ...
                sprintf('Stop time  : %.0f s', simState.hold_duration_s), ...
                'References : x_ref_ts, y_ref_ts, z_ref_ts' ...
                });
        catch simErr
            updateSimStatus({ ...
                'Hold simulation failed.', ...
                simErr.message, ...
                'The bridge still published x_ref_ts, y_ref_ts, z_ref_ts to the base workspace.' ...
                });
        end
    end

    function runCurrentTrajectorySimulation()
        if isempty(app.traj.preview)
            updateSimStatus({'Run current path failed.', 'No previewed or loaded trajectory is active.'});
            return
        end
        if ~isfile(app.sim.modelPath)
            updateSimStatus({'Model file not found.', app.sim.modelPath});
            return
        end

        trajState = app.traj.preview;
        publishTrajectorySignals(trajState, 'ee268_ui_traj', 'ee268_ui_traj_scenario');

        try
            load_system(app.sim.modelPath);
            modelName = getModelName();
            forceModelToUISource();
            simGuard = configureLowMemorySimulation(modelName, trajState.duration_s); %#ok<NASGU>
            sim(modelName);
            updateSimStatus({ ...
                'Current path simulation completed.', ...
                sprintf('Path       : %s', trajState.name), ...
                sprintf('Samples    : %d', numel(trajState.t)), ...
                sprintf('Stop time  : %.2f s', trajState.duration_s), ...
                'References : x_ref_ts, y_ref_ts, z_ref_ts' ...
                });
        catch simErr
            updateSimStatus({ ...
                'Current path simulation failed.', ...
                simErr.message, ...
                'The trajectory was still published to x_ref_ts, y_ref_ts, z_ref_ts.' ...
                });
        end
    end

    function simState = buildSimulinkBridgeState()
        holdDuration = 0.5;
        t = [0; holdDuration];
        ee = app.currentEE(:);
        thetaCommandDeg = app.currentThetaDeg(:).';
        thetaCommandRad = deg2rad(thetaCommandDeg);
        thetaModelRad = delta_command_to_model(app.params, thetaCommandDeg(:)).';
        thetaModelDeg = rad2deg(thetaModelRad);
        gripperTop = ee + [0; 0; app.visual.gripperTopOffsetMm / 1e3];
        rotationPoint = ee + [0; 0; app.visual.rotationPointOffsetMm / 1e3];
        tip = ee + [0; 0; -app.visual.tipOffsetMm / 1e3];

        simState = struct();
        simState.timestamp = datetime('now');
        simState.ee_xyz_m = ee.';
        simState.theta_deg = thetaCommandDeg;
        simState.theta_rad = thetaCommandRad;
        simState.theta_model_deg = thetaModelDeg;
        simState.theta_model_rad = thetaModelRad;
        simState.gripper_top_m = gripperTop.';
        simState.rotation_point_m = rotationPoint.';
        simState.tip_m = tip.';
        simState.hold_duration_s = holdDuration;
        simState.x_ref_ts = timeseries([ee(1); ee(1)], t);
        simState.y_ref_ts = timeseries([ee(2); ee(2)], t);
        simState.z_ref_ts = timeseries([ee(3); ee(3)], t);

        scenario = Simulink.SimulationData.Dataset;
        scenario = scenario.addElement(simState.x_ref_ts, 'x');
        scenario = scenario.addElement(simState.y_ref_ts, 'y');
        scenario = scenario.addElement(simState.z_ref_ts, 'z');
        simState.scenario = scenario;
    end

    function publishTrajectorySignals(trajState, structName, datasetName)
        assignin('base', structName, trajState);
        assignin('base', 'x_ref_ts', trajState.x_ts);
        assignin('base', 'y_ref_ts', trajState.y_ts);
        assignin('base', 'z_ref_ts', trajState.z_ts);
        assignin('base', datasetName, trajState.scenario);

        if isfield(trajState, 'gripper_ts') && isa(trajState.gripper_ts, 'timeseries')
            assignin('base', 'gripper_ref_ts', trajState.gripper_ts);
        end
        if isfield(trajState, 'contact_ts') && isa(trajState.contact_ts, 'timeseries')
            assignin('base', 'contact_pick_hold_ts', trajState.contact_ts);
        end
    end

    function forceModelToUISource()
        if ~isfile(app.sim.modelPath)
            return
        end

        try
            load_system(app.sim.modelPath);
            modelName = getModelName();
            switchBlocks = { ...
                [modelName '/Kinematics/X Source'], ...
                [modelName '/Kinematics/Y Source'], ...
                [modelName '/Kinematics/Z Source'] ...
                };

            for switchIdx = 1:numel(switchBlocks)
                if getSimulinkBlockHandle(switchBlocks{switchIdx}) > 0
                    set_param(switchBlocks{switchIdx}, 'sw', '0');
                end
            end
        catch
            % Leave the model untouched if the bridge blocks are missing.
        end
    end

    function cleanupObj = configureLowMemorySimulation(modelName, stopTime)
        cleanupState = struct();
        cleanupState.modelName = modelName;
        cleanupState.modelParams = captureParams(modelName, { ...
            'StopTime', ...
            'SignalLogging', ...
            'SaveOutput', ...
            'SaveTime', ...
            'SaveState', ...
            'ReturnWorkspaceOutputs', ...
            'DSMLogging', ...
            'LimitDataPoints', ...
            'MaxDataPoints', ...
            'Decimation' ...
            });

        setExistingParams(modelName, struct( ...
            'StopTime', num2str(stopTime), ...
            'SignalLogging', 'off', ...
            'SaveOutput', 'off', ...
            'SaveTime', 'off', ...
            'SaveState', 'off', ...
            'ReturnWorkspaceOutputs', 'off', ...
            'DSMLogging', 'off', ...
            'LimitDataPoints', 'on', ...
            'MaxDataPoints', num2str(app.sim.maxLoggedPoints), ...
            'Decimation', num2str(app.sim.loggingDecimation)));

        cleanupState.scopeBlocks = captureScopeLoggingState(modelName);
        for scopeIdx = 1:numel(cleanupState.scopeBlocks)
            scopePath = cleanupState.scopeBlocks(scopeIdx).path;
            setExistingParams(scopePath, struct( ...
                'DataLogging', 'off', ...
                'DataLoggingLimitDataPoints', 'on', ...
                'DataLoggingMaxPoints', num2str(app.sim.maxLoggedPoints), ...
                'DataLoggingDecimateData', 'on', ...
                'DataLoggingDecimation', num2str(app.sim.loggingDecimation)));
        end

        cleanupObj = onCleanup(@() restoreLowMemorySimulation(cleanupState));
    end

    function scopeState = captureScopeLoggingState(modelName)
        scopePaths = find_system(modelName, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Scope');
        scopeState = repmat(struct('path', '', 'params', struct()), 1, numel(scopePaths));
        for scopeIdx = 1:numel(scopePaths)
            scopeState(scopeIdx).path = scopePaths{scopeIdx};
            scopeState(scopeIdx).params = captureParams(scopePaths{scopeIdx}, { ...
                'DataLogging', ...
                'DataLoggingLimitDataPoints', ...
                'DataLoggingMaxPoints', ...
                'DataLoggingDecimateData', ...
                'DataLoggingDecimation' ...
                });
        end
    end

    function params = captureParams(targetPath, paramNames)
        params = struct();
        for paramIdx = 1:numel(paramNames)
            paramName = paramNames{paramIdx};
            try
                params.(paramName) = get_param(targetPath, paramName);
            catch
                % Skip parameters not supported by this model or block.
            end
        end
    end

    function setExistingParams(targetPath, paramMap)
        paramNames = fieldnames(paramMap);
        for paramIdx = 1:numel(paramNames)
            paramName = paramNames{paramIdx};
            try
                set_param(targetPath, paramName, paramMap.(paramName));
            catch
                % Leave unsupported parameters unchanged.
            end
        end
    end

    function restoreLowMemorySimulation(cleanupState)
        if ~bdIsLoaded(cleanupState.modelName)
            return
        end

        for scopeIdx = 1:numel(cleanupState.scopeBlocks)
            scopePath = cleanupState.scopeBlocks(scopeIdx).path;
            if getSimulinkBlockHandle(scopePath) > 0
                setExistingParams(scopePath, cleanupState.scopeBlocks(scopeIdx).params);
            end
        end

        setExistingParams(cleanupState.modelName, cleanupState.modelParams);
    end

    function modelName = getModelName()
        [~, modelName, ~] = fileparts(app.sim.modelPath);
    end

    function updateSimStatus(lines)
        if ischar(lines) || isstring(lines)
            lines = cellstr(lines);
        end
        app.simStatus.Value = lines;
    end

    function updateManualStatus(lines)
        if ischar(lines) || isstring(lines)
            lines = cellstr(lines);
        end
        app.manualStatus.Value = lines;
    end

    function updateTrajectoryStatus(lines)
        if ischar(lines) || isstring(lines)
            lines = cellstr(lines);
        end
        app.trajStatus.Value = lines;
    end

    function updateTraceStatus(lines)
        if ischar(lines) || isstring(lines)
            lines = cellstr(lines);
        end
        app.traceStatus.Value = lines;
    end

    function trajState = buildTrajectoryShapeState()
        shapeName = app.trajShapeDrop.Value;
        requestedSizeMm = app.trajSizeField.Value;
        durationS = app.trajDurationField.Value;
        turns = max(1, round(app.trajTurnsField.Value));
        requestedCenter = app.currentEE(:);
        [center, centerAdjusted] = chooseTrajectoryCenter(shapeName, requestedCenter, requestedSizeMm, durationS, turns);

        [sizeMm, t, x, y, z, worstCond] = fitTrajectoryToWorkspace(shapeName, center, requestedSizeMm, durationS, turns);

        trajState = struct();
        trajState.name = shapeName;
        trajState.requested_center = requestedCenter.';
        trajState.center = center.';
        trajState.center_adjusted = centerAdjusted;
        trajState.requested_size_mm = requestedSizeMm;
        trajState.size_mm = sizeMm;
        trajState.duration_s = durationS;
        trajState.turns = turns;
        trajState.t = t;
        trajState.x = x;
        trajState.y = y;
        trajState.z = z;
        trajState.xyz = [x, y, z];
        trajState.x_ts = timeseries(x, t);
        trajState.y_ts = timeseries(y, t);
        trajState.z_ts = timeseries(z, t);
        trajState.worst_cond_Jx = worstCond;
        scenario = Simulink.SimulationData.Dataset;
        scenario = scenario.addElement(trajState.x_ts, 'x');
        scenario = scenario.addElement(trajState.y_ts, 'y');
        scenario = scenario.addElement(trajState.z_ts, 'z');
        trajState.scenario = scenario;
    end

    function pickState = buildPickPlaceTrajectoryState()
        pickCenterMm = parseVectorField(app.pickObjectField.Value, 3, 'object center [x y z]');
        placeCenterMm = parseVectorField(app.placeObjectField.Value, 3, 'place center [x y z]');
        dimsMm = parseVectorField(app.pickDimsField.Value, 3, 'object size [dx dy dz]');
        gripDeg = parseVectorField(app.pickGripField.Value, 2, 'gripper [open close]');

        if any(dimsMm <= 0)
            error('Object dimensions must be strictly positive.');
        end
        if any(gripDeg < 0) || any(gripDeg > 90)
            error('Gripper command angles must stay within [0 90] deg.');
        end
        if gripDeg(2) >= gripDeg(1)
            error('Gripper close angle must be smaller than the open angle.');
        end

        clearanceMm = app.pickClearanceField.Value;
        pickCenter = pickCenterMm(:).' / 1e3;
        placeCenter = placeCenterMm(:).' / 1e3;
        dimsM = dimsMm(:).' / 1e3;
        topOffset = [0, 0, dimsM(3) / 2];

        desiredPickGrip = pickCenter + topOffset;
        desiredPlaceGrip = placeCenter + topOffset;
        [pickGrip, ~] = findNearestReachable(desiredPickGrip);
        [placeGrip, ~] = findNearestReachable(desiredPlaceGrip);
        effectivePickCenter = pickGrip - topOffset;
        effectivePlaceCenter = placeGrip - topOffset;

        clearanceM = clearanceMm / 1e3;
        abovePick = findNearestReachableWithZFloor(pickGrip, pickGrip(3) + clearanceM);
        abovePlace = findNearestReachableWithZFloor(placeGrip, placeGrip(3) + clearanceM);

        tWaypoint = [0; 1.0; 1.6; 2.3; 2.6; 3.0; 3.8; 5.1; 5.8; 6.1; 6.5; 7.3];
        xyzWaypoints = [ ...
            app.currentEE; ...
            abovePick; ...
            abovePick; ...
            pickGrip; ...
            pickGrip; ...
            pickGrip; ...
            abovePick; ...
            abovePlace; ...
            placeGrip; ...
            placeGrip; ...
            placeGrip; ...
            abovePlace ...
            ];

        t = linspace(0, tWaypoint(end), 360).';
        x = interp1(tWaypoint, xyzWaypoints(:, 1), t, 'linear');
        y = interp1(tWaypoint, xyzWaypoints(:, 2), t, 'linear');
        z = interp1(tWaypoint, xyzWaypoints(:, 3), t, 'linear');
        [x, y, z] = projectTrajectoryToReachable(x, y, z);
        worstCond = validateEETrajectory(x, y, z);

        gripOpen = gripDeg(1);
        gripClosed = gripDeg(2);
        tCloseStart = tWaypoint(5);
        tCloseDone = tWaypoint(6);
        tOpenStart = tWaypoint(10);
        tOpenDone = tWaypoint(11);

        gripWaypoint = [0; tCloseStart; tCloseDone; tOpenStart; tOpenDone; tWaypoint(end)];
        gripValues = [gripOpen; gripOpen; gripClosed; gripClosed; gripOpen; gripOpen];
        gripSignal = interp1(gripWaypoint, gripValues, t, 'linear');
        contactSignal = double(t >= tCloseDone & t < tOpenStart);

        xTs = timeseries(x, t, 'Name', 'x');
        yTs = timeseries(y, t, 'Name', 'y');
        zTs = timeseries(z, t, 'Name', 'z');
        gripperTs = timeseries(gripSignal, t, 'Name', 'gripper');
        contactTs = timeseries(contactSignal, t, 'Name', 'contact');

        scenario = Simulink.SimulationData.Dataset;
        scenario = scenario.addElement(xTs, 'x');
        scenario = scenario.addElement(yTs, 'y');
        scenario = scenario.addElement(zTs, 'z');
        scenario = scenario.addElement(gripperTs, 'gripper');
        scenario = scenario.addElement(contactTs, 'contact');

        pickState = struct();
        pickState.name = 'pick_place';
        pickState.t = t;
        pickState.x = x;
        pickState.y = y;
        pickState.z = z;
        pickState.xyz = [x, y, z];
        pickState.x_ts = xTs;
        pickState.y_ts = yTs;
        pickState.z_ts = zTs;
        pickState.gripper_ts = gripperTs;
        pickState.contact_ts = contactTs;
        pickState.gripper_open_deg = gripOpen;
        pickState.gripper_closed_deg = gripClosed;
        pickState.contact_high_window_s = [tCloseDone, tOpenStart];
        pickState.close_done_s = tCloseDone;
        pickState.open_start_s = tOpenStart;
        pickState.duration_s = t(end);
        pickState.turns = 1;
        pickState.center = mean([pickGrip; placeGrip], 1);
        pickState.size_mm = max(range([x, y, z], 1)) * 1e3;
        pickState.object_dims_mm = dimsMm;
        pickState.requested_pick_center_mm = pickCenterMm;
        pickState.requested_place_center_mm = placeCenterMm;
        pickState.pick_center_mm = effectivePickCenter * 1e3;
        pickState.place_center_mm = effectivePlaceCenter * 1e3;
        pickState.pick_grip_point_mm = pickGrip * 1e3;
        pickState.place_grip_point_mm = placeGrip * 1e3;
        pickState.above_pick_mm = abovePick * 1e3;
        pickState.above_place_mm = abovePlace * 1e3;
        pickState.approach_clearance_mm = clearanceMm;
        pickState.worst_cond_Jx = worstCond;
        pickState.scenario = scenario;
        pickState.signals = {'x_ref_ts', 'y_ref_ts', 'z_ref_ts', 'gripper_ref_ts', 'contact_pick_hold_ts'};
        pickState.rigid_transforms = buildPickPlaceRigidTransforms(effectivePickCenter * 1e3, effectivePlaceCenter * 1e3, dimsMm);
        pickState.simulink_notes = buildPickPlaceSimulinkNotes(pickState, dimsMm);
    end

    function rigidTransforms = buildPickPlaceRigidTransforms(pickCenterMm, placeCenterMm, dimsMm)
        rigidTransforms = struct();
        rigidTransforms.pick_frame_name = 'PickProductFrame';
        rigidTransforms.place_frame_name = 'PlaceTargetFrame';
        rigidTransforms.pick_translation_mm = pickCenterMm;
        rigidTransforms.place_translation_mm = placeCenterMm;
        rigidTransforms.rotation = 'None';
        rigidTransforms.translation_method = 'Cartesian';
        rigidTransforms.object_dims_mm = dimsMm;
    end

    function notes = buildPickPlaceSimulinkNotes(pickState, dimsMm)
        notes = { ...
            'Add two Rigid Transform blocks from Base Link Visual R Frame with RotationMethod=None and TranslationMethod=Cartesian.', ...
            sprintf('PickProductFrame translation [mm]: [%0.1f %0.1f %0.1f]', pickState.rigid_transforms.pick_translation_mm(1), pickState.rigid_transforms.pick_translation_mm(2), pickState.rigid_transforms.pick_translation_mm(3)), ...
            sprintf('PlaceTargetFrame translation [mm]: [%0.1f %0.1f %0.1f]', pickState.rigid_transforms.place_translation_mm(1), pickState.rigid_transforms.place_translation_mm(2), pickState.rigid_transforms.place_translation_mm(3)), ...
            sprintf('If the product body is a Brick Solid, set BrickDimensions [mm] = [%0.1f %0.1f %0.1f].', dimsMm(1), dimsMm(2), dimsMm(3)), ...
            'Attach the product reference frame to PickProductFrame for the initial pose. Use PlaceTargetFrame as the release marker.', ...
            'Use From Workspace blocks like the existing UI X/UI Y/UI Z bridge blocks.', ...
            'Workspace variables for this motion are x_ref_ts, y_ref_ts, z_ref_ts, gripper_ref_ts, contact_pick_hold_ts.', ...
            'Feed gripper_ref_ts to one finger, and pass it through a Gain = -1 block for the mirrored finger.', ...
            sprintf('Set contact_pick_hold_ts HIGH at t = %.2f s after the close action finishes.', pickState.close_done_s), ...
            sprintf('Set contact_pick_hold_ts LOW at t = %.2f s when the open action begins at the place pose.', pickState.open_start_s) ...
            };
    end

    function lines = formatPickPlaceStatus(pickState, wasPublished)
        outputName = 'ee268_pick_place_signal.mat';
        if wasPublished
            header = 'Pick/place motion published.';
        else
            header = 'Pick/place preview ready.';
        end
        lines = { ...
            header, ...
            sprintf('Pick center [mm]   : [%.1f %.1f %.1f]', pickState.pick_center_mm(1), pickState.pick_center_mm(2), pickState.pick_center_mm(3)), ...
            sprintf('Place center [mm]  : [%.1f %.1f %.1f]', pickState.place_center_mm(1), pickState.place_center_mm(2), pickState.place_center_mm(3)), ...
            sprintf('Object dims [mm]   : [%.1f %.1f %.1f]', pickState.object_dims_mm(1), pickState.object_dims_mm(2), pickState.object_dims_mm(3)), ...
            sprintf('Gripper open/close : [%.1f %.1f] deg', pickState.gripper_open_deg, pickState.gripper_closed_deg), ...
            sprintf('Contact HIGH       : %.2f s to %.2f s', pickState.contact_high_window_s(1), pickState.contact_high_window_s(2)), ...
            sprintf('Signals            : %s', strjoin(pickState.signals, ', ')), ...
            sprintf('Rigid pick frame   : [%0.1f %0.1f %0.1f] mm', pickState.rigid_transforms.pick_translation_mm(1), pickState.rigid_transforms.pick_translation_mm(2), pickState.rigid_transforms.pick_translation_mm(3)), ...
            sprintf('Rigid place frame  : [%0.1f %0.1f %0.1f] mm', pickState.rigid_transforms.place_translation_mm(1), pickState.rigid_transforms.place_translation_mm(2), pickState.rigid_transforms.place_translation_mm(3)), ...
            sprintf('MAT file           : %s', outputName) ...
            };
    end

    function values = parseVectorField(textValue, expectedLen, fieldName)
        if isstring(textValue)
            textValue = char(textValue);
        end
        numericParts = sscanf(strrep(textValue, ',', ' '), '%f');
        if numel(numericParts) ~= expectedLen
            error('Field %s must contain exactly %d numeric values.', fieldName, expectedLen);
        end
        values = double(numericParts(:)).';
    end

    function updatePickPlaceStatus(lines)
        if ischar(lines) || isstring(lines)
            lines = cellstr(lines);
        end
        app.pickStatus.Value = lines;
    end

    function [center, centerAdjusted] = chooseTrajectoryCenter(shapeName, requestedCenter, requestedSizeMm, durationS, turns)
        center = requestedCenter;
        centerAdjusted = false;

        try
            [~, ~, ~, ~, ~, ~] = fitTrajectoryToWorkspace(shapeName, center, requestedSizeMm, durationS, turns);
            return
        catch
        end

        candidateSets = {};
        if isfield(app.results, 'both_keep') && any(app.results.both_keep)
            candidateSets{end + 1} = app.results.reach_ee(app.results.both_keep, :); %#ok<AGROW>
        end
        if isfield(app.results, 'manip_keep') && any(app.results.manip_keep)
            candidateSets{end + 1} = app.results.reach_ee(app.results.manip_keep, :); %#ok<AGROW>
        end
        if isfield(app.results, 'clearance_keep') && any(app.results.clearance_keep)
            candidateSets{end + 1} = app.results.reach_ee(app.results.clearance_keep, :); %#ok<AGROW>
        end
        candidateSets{end + 1} = app.results.reach_ee; %#ok<AGROW>

        for setIdx = 1:numel(candidateSets)
            candidates = unique(candidateSets{setIdx}, 'rows', 'stable');
            diffs = candidates - requestedCenter.';
            dist2 = sum(diffs.^2, 2);
            [~, order] = sort(dist2, 'ascend');
            maxChecks = min(60, numel(order));
            for candidateIdx = 1:maxChecks
                trialCenter = candidates(order(candidateIdx), :).';
                try
                    [~, ~, ~, ~, ~, ~] = fitTrajectoryToWorkspace(shapeName, trialCenter, requestedSizeMm, durationS, turns);
                    center = trialCenter;
                    centerAdjusted = norm(center - requestedCenter) > 1e-9;
                    return
                catch
                end
            end
        end
    end

    function [sizeMm, t, x, y, z, worstCond] = fitTrajectoryToWorkspace(shapeName, center, requestedSizeMm, durationS, turns)
        minSizeMm = app.trajSizeField.Limits(1);
        candidateSizeMm = requestedSizeMm;
        lastErr = '';

        while candidateSizeMm >= minSizeMm - 1e-9
            [t, x, y, z] = generateTrajectorySamples(shapeName, center, candidateSizeMm / 1e3, durationS, turns);
            try
                worstCond = validateEETrajectory(x, y, z);
                sizeMm = candidateSizeMm;
                return
            catch trajErr
                lastErr = trajErr.message;
                nextSizeMm = max(minSizeMm, floor(candidateSizeMm * 0.85));
                if nextSizeMm >= candidateSizeMm
                    break
                end
                candidateSizeMm = nextSizeMm;
            end
        end

        error('Requested %s trajectory is not reachable from the current pose. Last validation error: %s', shapeName, lastErr);
    end

    function [t, x, y, z] = generateTrajectorySamples(shapeName, center, sizeM, durationS, turns)
        switch shapeName
            case 'circle'
                [t, x, y, z] = makeCircleTrajectory(center, sizeM, durationS, turns);
            case 'square'
                [t, x, y, z] = makeSquareTrajectory(center, sizeM, durationS, turns);
            case 'line-x'
                [t, x, y, z] = makeLineTrajectory(center, sizeM, durationS, 'x');
            case 'line-y'
                [t, x, y, z] = makeLineTrajectory(center, sizeM, durationS, 'y');
            case 'spiral'
                [t, x, y, z] = makeSpiralTrajectory(center, sizeM, durationS, turns);
            otherwise
                error('Unsupported shape: %s', shapeName);
        end
    end

    function worstCond = validateEETrajectory(x, y, z)
        worstCond = 0;
        for sampleIdx = 1:numel(x)
            eePoint = [x(sampleIdx); y(sampleIdx); z(sampleIdx)];
            platformPoint = eePoint - app.results.context.tool_offset_base;
            [theta, valid, errMsg] = delta_IK(app.params, platformPoint);
            if ~valid
                error('Trajectory invalid at sample %d: %s', sampleIdx, errMsg);
            end
            [~, Jq, type1, type2, info] = delta_singularity(app.params, theta);
            if type1 || type2 || any(~isfinite(diag(Jq)))
                error('Trajectory hits a singularity at sample %d.', sampleIdx);
            end
            worstCond = max(worstCond, info.cond_Jx);
        end
    end

    function applyTrajectoryPreview(trajState)
        app.traj.preview = trajState;
        set(app.hTrajPreview, 'XData', trajState.x * 1e3, 'YData', trajState.y * 1e3, 'ZData', trajState.z * 1e3);
        set(app.hTrajStart, 'XData', trajState.x(1) * 1e3, 'YData', trajState.y(1) * 1e3, 'ZData', trajState.z(1) * 1e3);
        set(app.hTrajEnd, 'XData', trajState.x(end) * 1e3, 'YData', trajState.y(end) * 1e3, 'ZData', trajState.z(end) * 1e3);
    end

    function loadedScenario = loadScenarioFile(scenarioPath)
        loaded = load(scenarioPath);

        if isfield(loaded, 'traj') && isfield(loaded.traj, 'x_ts') && isfield(loaded.traj, 'y_ts') && isfield(loaded.traj, 'z_ts')
            loadedScenario = loaded.traj;
            if ~isfield(loadedScenario, 'scenario') && isfield(loaded, 'scenario')
                loadedScenario.scenario = loaded.scenario;
            end
        elseif isfield(loaded, 'scenario')
            loadedScenario = scenarioDatasetToTrajectory(loaded.scenario, scenarioPath);
        else
            error('MAT file does not contain a supported traj or scenario variable.');
        end

        if ~isfield(loadedScenario, 'scenario')
            scenario = Simulink.SimulationData.Dataset;
            scenario = scenario.addElement(loadedScenario.x_ts, 'x');
            scenario = scenario.addElement(loadedScenario.y_ts, 'y');
            scenario = scenario.addElement(loadedScenario.z_ts, 'z');
            loadedScenario.scenario = scenario;
        end
    end

    function trajState = scenarioDatasetToTrajectory(datasetObj, scenarioPath)
        xTs = extractDatasetSignal(datasetObj, 'x');
        yTs = extractDatasetSignal(datasetObj, 'y');
        zTs = extractDatasetSignal(datasetObj, 'z');
        [t, x, y, z] = alignScenarioSignals(xTs, yTs, zTs);
        [x, y, z, importNotes] = normalizeScenarioPathToWorkspace(x, y, z, scenarioPath);
        xTs = timeseries(x, t, 'Name', 'x');
        yTs = timeseries(y, t, 'Name', 'y');
        zTs = timeseries(z, t, 'Name', 'z');

        scenario = Simulink.SimulationData.Dataset;
        scenario = scenario.addElement(xTs, 'x');
        scenario = scenario.addElement(yTs, 'y');
        scenario = scenario.addElement(zTs, 'z');

        trajState = struct();
        [~, scenarioName, ~] = fileparts(scenarioPath);
        trajState.name = normalizeScenarioLabel(scenarioName);
        trajState.center = [mean(x), mean(y), mean(z)];
        trajState.size_mm = max(range(x), range(y)) * 1e3;
        trajState.duration_s = t(end) - t(1);
        trajState.turns = 1;
        trajState.t = t;
        trajState.x = x;
        trajState.y = y;
        trajState.z = z;
        trajState.xyz = [x, y, z];
        trajState.x_ts = xTs;
        trajState.y_ts = yTs;
        trajState.z_ts = zTs;
        trajState.worst_cond_Jx = NaN;
        trajState.import_notes = importNotes;
        trajState.scenario = scenario;
    end

    function [xFit, yFit, zFit, notes] = normalizeScenarioPathToWorkspace(xIn, yIn, zIn, scenarioPath)
        xRaw = xIn(:);
        yRaw = yIn(:);
        zRaw = zIn(:);
        notes = {};

        if all(zRaw >= 0)
            zRaw = -zRaw;
            notes{end + 1} = 'Converted imported scenario Z from positive-down to robot negative-up convention.';
        end

        rawPath = [xRaw, yRaw, zRaw];
        rawCenter = mean(rawPath, 1);
        centeredPath = rawPath - rawCenter;
        [fitPath, scaleUsed, centerUsed] = fitScenarioOffsetsToWorkspace(centeredPath);

        xFit = fitPath(:, 1);
        yFit = fitPath(:, 2);
        zFit = fitPath(:, 3);

        notes{end + 1} = sprintf('Imported scenario was auto-fit to reachable EE workspace with scale %.3f.', scaleUsed);
        notes{end + 1} = sprintf('Applied center [mm] = [%.1f %.1f %.1f].', centerUsed(1) * 1e3, centerUsed(2) * 1e3, centerUsed(3) * 1e3);
        notes{end + 1} = sprintf('Source file: %s', scenarioPath);
    end

    function [fitPath, scaleUsed, centerUsed] = fitScenarioOffsetsToWorkspace(centeredPath)
        candidateCenters = getScenarioFitCenters();
        scaleCandidates = [1.0, 0.92 .^ (1:24)];
        lastErr = 'No candidate fit attempted.';

        for centerIdx = 1:size(candidateCenters, 1)
            center = candidateCenters(centerIdx, :);
            for scaleIdx = 1:numel(scaleCandidates)
                candidateScale = scaleCandidates(scaleIdx);
                candidatePath = centeredPath * candidateScale + center;
                try
                    validateEETrajectory(candidatePath(:, 1), candidatePath(:, 2), candidatePath(:, 3));
                    fitPath = candidatePath;
                    scaleUsed = candidateScale;
                    centerUsed = center;
                    return
                catch fitErr
                    lastErr = fitErr.message;
                end
            end
        end

        error('Imported scenario is not reachable after convention conversion and workspace fitting. Last validation error: %s', lastErr);
    end

    function candidateCenters = getScenarioFitCenters()
        seeds = [
            app.currentEE;
            app.homeEE;
            mean(app.results.reach_ee, 1);
            [0, 0, mean(app.results.reach_ee(:, 3))]
            ];

        snappedCenters = zeros(size(seeds));
        for seedIdx = 1:size(seeds, 1)
            snappedCenters(seedIdx, :) = findNearestReachable(seeds(seedIdx, :));
        end

        candidateCenters = unique(round(snappedCenters, 9), 'rows', 'stable');
    end

    function ts = extractDatasetSignal(datasetObj, signalName)
        try
            element = getElement(datasetObj, signalName);
            ts = datasetElementToTimeseries(element);
        catch
            try
                signalNames = arrayfun(@(idx) string(getElement(datasetObj, idx).Name), 1:numElements(datasetObj));
                matchIdx = find(strcmpi(signalNames, signalName), 1, 'first');
                if isempty(matchIdx)
                    error('Signal %s not found.', signalName);
                end
                element = getElement(datasetObj, matchIdx);
                ts = datasetElementToTimeseries(element);
            catch innerErr
                error('Could not extract signal ''%s'' from scenario dataset: %s', signalName, innerErr.message);
            end
        end
    end

    function ts = datasetElementToTimeseries(element)
        if isa(element, 'timeseries')
            ts = element;
        elseif isa(element, 'Simulink.SimulationData.Signal')
            ts = element.Values;
        else
            error('Unsupported dataset element type: %s', class(element));
        end
    end

    function [t, x, y, z] = alignScenarioSignals(xTs, yTs, zTs)
        xTime = xTs.Time(:);
        yTime = yTs.Time(:);
        zTime = zTs.Time(:);
        commonLen = min([numel(xTime), numel(yTime), numel(zTime), numel(xTs.Data(:)), numel(yTs.Data(:)), numel(zTs.Data(:))]);

        t = xTime(1:commonLen);
        x = xTs.Data(1:commonLen);

        if numel(yTime) == commonLen && max(abs(yTime(1:commonLen) - t)) < 1e-9
            y = yTs.Data(1:commonLen);
        else
            y = interp1(yTime, yTs.Data(:), t, 'linear', 'extrap');
        end

        if numel(zTime) == commonLen && max(abs(zTime(1:commonLen) - t)) < 1e-9
            z = zTs.Data(1:commonLen);
        else
            z = interp1(zTime, zTs.Data(:), t, 'linear', 'extrap');
        end

        x = x(:);
        y = y(:);
        z = z(:);
    end

    function label = normalizeScenarioLabel(rawName)
        label = regexprep(rawName, '_scenario$', '', 'ignorecase');
        label = regexprep(label, '^signal_builder_', '', 'ignorecase');
    end

    function textOut = vectorText(values)
        textOut = strtrim(sprintf('%g ', values));
    end

    function [t, x, y, z] = makeCircleTrajectory(center, radiusM, durationS, turns)
        t = linspace(0, durationS, 300).';
        theta = linspace(0, 2 * pi * turns, numel(t)).';
        x = center(1) + radiusM * cos(theta);
        y = center(2) + radiusM * sin(theta);
        z = center(3) * ones(size(t));
    end

    function [t, x, y, z] = makeSquareTrajectory(center, halfSpanM, durationS, turns)
        corners = [ ...
            center(1) - halfSpanM, center(2) - halfSpanM; ...
            center(1) + halfSpanM, center(2) - halfSpanM; ...
            center(1) + halfSpanM, center(2) + halfSpanM; ...
            center(1) - halfSpanM, center(2) + halfSpanM; ...
            center(1) - halfSpanM, center(2) - halfSpanM];
        corners = repmat(corners, turns, 1);
        numSeg = size(corners, 1) - 1;
        samplesPerSeg = 50;
        x = [];
        y = [];
        for segIdx = 1:numSeg
            alpha = linspace(0, 1, samplesPerSeg + 1).';
            xs = corners(segIdx, 1) + alpha * (corners(segIdx + 1, 1) - corners(segIdx, 1));
            ys = corners(segIdx, 2) + alpha * (corners(segIdx + 1, 2) - corners(segIdx, 2));
            if segIdx > 1
                xs = xs(2:end);
                ys = ys(2:end);
            end
            x = [x; xs]; %#ok<AGROW>
            y = [y; ys]; %#ok<AGROW>
        end
        t = linspace(0, durationS, numel(x)).';
        z = center(3) * ones(size(t));
    end

    function [t, x, y, z] = makeLineTrajectory(center, halfSpanM, durationS, axisName)
        t = linspace(0, durationS, 220).';
        sweep = halfSpanM * sin(linspace(-pi/2, 3*pi/2, numel(t)).');
        x = center(1) * ones(size(t));
        y = center(2) * ones(size(t));
        if strcmp(axisName, 'x')
            x = center(1) + sweep;
        else
            y = center(2) + sweep;
        end
        z = center(3) * ones(size(t));
    end

    function [t, x, y, z] = makeSpiralTrajectory(center, radiusM, durationS, turns)
        t = linspace(0, durationS, 320).';
        theta = linspace(0, 2 * pi * turns, numel(t)).';
        r = linspace(0.15 * radiusM, radiusM, numel(t)).';
        x = center(1) + r .* cos(theta);
        y = center(2) + r .* sin(theta);
        z = center(3) * ones(size(t));
    end

    function scenarios = buildScenarioLibrary(projectRoot, workspaceRoot)
        scenarioPaths = { ...
            fullfile(projectRoot, 'signal_builder_circle_test.mat'), ...
            fullfile(workspaceRoot, 'writing.mat', 'AHMED_scenario.mat'), ...
            fullfile(workspaceRoot, 'writing.mat', 'BOODY_scenario.mat'), ...
            fullfile(workspaceRoot, 'writing.mat', 'HAMZA_scenario.mat'), ...
            fullfile(workspaceRoot, 'writing.mat', 'NADA_scenario.mat'), ...
            fullfile(workspaceRoot, 'writing.mat', 'POWER_scenario.mat'), ...
            fullfile(workspaceRoot, 'writing.mat', 'SH7S_scenario.mat') ...
            };
        labels = {};
        paths = {};
        for scenarioIdx = 1:numel(scenarioPaths)
            if isfile(scenarioPaths{scenarioIdx})
                [~, label, ~] = fileparts(scenarioPaths{scenarioIdx});
                labels{end + 1} = normalizeScenarioLabel(label); %#ok<AGROW>
                paths{end + 1} = scenarioPaths{scenarioIdx}; %#ok<AGROW>
            end
        end
        if isempty(labels)
            labels = {'no_scenarios_found'};
            paths = {''};
        end
        scenarios = struct('labels', {labels}, 'paths', {paths});
    end

    function updateRobotOverlay(thetaCommandDeg)
        thetaRad = delta_command_to_model(app.params, thetaCommandDeg(:));
        [platformPoint, fkOk] = delta_FK(app.params, thetaRad);
        if ~fkOk
            return
        end

        phi = app.visual.baseAnglesRad;
        cp = cos(phi);
        sp = sin(phi);

        frameTop = app.visual.frameVerticesMm;
        frameBottom = frameTop;
        frameBottom(:, 3) = app.visual.floorZMm;
        triClose = [1 2 3 1];
        for postIdx = 1:3
            nextIdx = mod(postIdx, 3) + 1;
            set(app.hFramePost(postIdx), 'XData', [frameTop(postIdx, 1), frameBottom(postIdx, 1)], ...
                'YData', [frameTop(postIdx, 2), frameBottom(postIdx, 2)], ...
                'ZData', [frameTop(postIdx, 3), frameBottom(postIdx, 3)]);
            set(app.hFrameTop(postIdx), 'XData', [frameTop(postIdx, 1), frameTop(nextIdx, 1)], ...
                'YData', [frameTop(postIdx, 2), frameTop(nextIdx, 2)], ...
                'ZData', [frameTop(postIdx, 3), frameTop(nextIdx, 3)]);
            set(app.hFrameFoot(postIdx), 'XData', frameBottom(postIdx, 1), 'YData', frameBottom(postIdx, 2), 'ZData', frameBottom(postIdx, 3));
        end

        shoulderPts = [app.params.Rb * cp(:), app.params.Rb * sp(:), zeros(3, 1)] * 1e3;
        elbowPts = zeros(3, 3);
        platformPts = zeros(3, 3);
        for armIdx = 1:3
            elbow = [(app.params.Rb + app.params.L1 * cos(thetaRad(armIdx))) * cp(armIdx); ...
                     (app.params.Rb + app.params.L1 * cos(thetaRad(armIdx))) * sp(armIdx); ...
                     -app.params.L1 * sin(thetaRad(armIdx))];
            plat = platformPoint + app.params.Rp * [cp(armIdx); sp(armIdx); 0];
            elbowPts(armIdx, :) = elbow.' * 1e3;
            platformPts(armIdx, :) = plat.' * 1e3;

            set(app.hUpper(armIdx), 'XData', [shoulderPts(armIdx, 1), elbowPts(armIdx, 1)], ...
                'YData', [shoulderPts(armIdx, 2), elbowPts(armIdx, 2)], ...
                'ZData', [shoulderPts(armIdx, 3), elbowPts(armIdx, 3)]);
            set(app.hFore(armIdx), 'XData', [elbowPts(armIdx, 1), platformPts(armIdx, 1)], ...
                'YData', [elbowPts(armIdx, 2), platformPts(armIdx, 2)], ...
                'ZData', [elbowPts(armIdx, 3), platformPts(armIdx, 3)]);
        end

        frameCenter = mean(frameTop, 1);
        baseCenter = mean(shoulderPts, 1);
        visualOffset = frameCenter - baseCenter;
        shoulderPts = shoulderPts + visualOffset;
        elbowPts = elbowPts + visualOffset;
        platformPts = platformPts + visualOffset;

        eeMm = app.currentEE * 1e3;
        platformMm = platformPoint.' * 1e3;
        eeMm = eeMm + visualOffset;
        platformMm = platformMm + visualOffset;
        gripperTopMm = eeMm + [0, 0, app.visual.gripperTopOffsetMm];
        rotationMm = eeMm + [0, 0, app.visual.rotationPointOffsetMm];
        tipMm = eeMm + [0, 0, -app.visual.tipOffsetMm];
        gripperBaseMm = eeMm + [0, 0, app.visual.gripperBaseMidOffsetMm];
        gripperTopTriPts = platformPts + repmat(gripperTopMm - platformMm, 3, 1);

        set(app.hBaseTri, 'XData', shoulderPts(triClose, 1), 'YData', shoulderPts(triClose, 2), 'ZData', shoulderPts(triClose, 3));
        set(app.hPlatformTri, 'XData', gripperTopTriPts(triClose, 1), 'YData', gripperTopTriPts(triClose, 2), 'ZData', gripperTopTriPts(triClose, 3));

        for armIdx = 1:3
            set(app.hFore(armIdx), 'XData', [elbowPts(armIdx, 1), gripperTopTriPts(armIdx, 1)], ...
                'YData', [elbowPts(armIdx, 2), gripperTopTriPts(armIdx, 2)], ...
                'ZData', [elbowPts(armIdx, 3), gripperTopTriPts(armIdx, 3)]);
        end

        set(app.hNeck, 'XData', [gripperTopMm(1), rotationMm(1)], 'YData', [gripperTopMm(2), rotationMm(2)], 'ZData', [gripperTopMm(3), rotationMm(3)]);
        set(app.hTool, 'XData', [rotationMm(1), eeMm(1)], 'YData', [rotationMm(2), eeMm(2)], 'ZData', [rotationMm(3), eeMm(3)]);
        set(app.hTip, 'XData', [eeMm(1), tipMm(1)], 'YData', [eeMm(2), tipMm(2)], 'ZData', [eeMm(3), tipMm(3)]);
        set(app.hGripperTop, 'XData', gripperTopMm(1), 'YData', gripperTopMm(2), 'ZData', gripperTopMm(3));
        set(app.hRotationPoint, 'XData', rotationMm(1), 'YData', rotationMm(2), 'ZData', rotationMm(3));
        set(app.hTipPoint, 'XData', tipMm(1), 'YData', tipMm(2), 'ZData', tipMm(3));
        set(app.hGuides(1), 'XData', [0, 0], 'YData', [0, 0], 'ZData', [0, app.visual.floorZMm]);
        set(app.hGuides(2), 'XData', [eeMm(1), gripperTopMm(1)], 'YData', [eeMm(2), gripperTopMm(2)], 'ZData', [eeMm(3), gripperTopMm(3)]);
        set(app.hGuides(3), 'XData', [eeMm(1), tipMm(1)], 'YData', [eeMm(2), tipMm(2)], 'ZData', [eeMm(3), tipMm(3)]);

        jointPts = [shoulderPts; elbowPts; gripperTopTriPts; gripperTopMm; rotationMm; gripperBaseMm; eeMm; tipMm];
        set(app.hRobotJoints, 'XData', jointPts(:, 1), 'YData', jointPts(:, 2), 'ZData', jointPts(:, 3));
    end

    function updateSliderTicks(sliderHandle)
        limits = sliderHandle.Limits;
        ticks = linspace(limits(1), limits(2), 5);
        sliderHandle.MajorTicks = ticks;
        sliderHandle.MajorTickLabels = compose('%.0f', ticks * 1e3);
    end

    function visual = buildVisualModel()
        visual = struct();
        visual.floorZMm = -383;
        visual.frameRadiusMm = 272.5;
        visual.gripperTopToRotationMm = 32.0;
        visual.rotationToEEMm = 55.18;
        visual.rotationPointOffsetMm = visual.rotationToEEMm;
        visual.gripperTopOffsetMm = visual.gripperTopToRotationMm + visual.rotationToEEMm;
        visual.tipOffsetMm = 51.85;
        visual.gripperBaseMidOffsetMm = 43.575;
        visual.frameAnglesRad = deg2rad([-90, 30, 150]);
        visual.baseAnglesRad = visual.frameAnglesRad;
        visual.frameVerticesMm = [visual.frameRadiusMm * cos(visual.frameAnglesRad(:)), ...
                      visual.frameRadiusMm * sin(visual.frameAnglesRad(:)), ...
                                  zeros(3, 1)];
    end

    function out = ternary(condValue, trueText, falseText)
        if condValue
            out = trueText;
        else
            out = falseText;
        end
    end

    function state = onOff(flag)
        if flag
            state = 'on';
        else
            state = 'off';
        end
    end
end
