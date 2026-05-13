function results = test_ee268_master_ui_options()
analysisRoot = fileparts(mfilename('fullpath'));
addpath(analysisRoot);

results = struct('passes', {{}}, 'failures', {{}});
app = ee268_master_ui('Visible', 'off');
cleanupObj = onCleanup(@() cleanupApp(app)); %#ok<NASGU>

recordPass('ui_launch');
assert(abs(app.simDurationField.Value - 0.5) < 1e-9, 'Sim duration is not fixed at 0.5 s.');
recordPass('sim_duration_fixed_0_5s');

shapeItems = cellstr(app.trajShapeDrop.Items);
for shapeIdx = 1:numel(shapeItems)
	shapeName = shapeItems{shapeIdx};
	app.trajShapeDrop.Value = shapeName;
	runAction(['preview_shape_' shapeName], @() pushButton(app.btnPreviewTraj));
	runAction(['publish_shape_' shapeName], @() pushButton(app.btnPublishTraj));
end
runAction('clear_preview', @() pushButton(app.btnClearTraj));

scenarioItems = cellstr(app.scenarioDrop.Items);
for scenarioIdx = 1:numel(scenarioItems)
	app.scenarioDrop.Value = scenarioItems{scenarioIdx};
	runAction(['load_scenario_' matlab.lang.makeValidName(scenarioItems{scenarioIdx})], @() pushButton(app.btnLoadScenario));
end

runAction('manual_home', @() pushButton(app.btnManualHome));
runAction('manual_x_neg', @() pushButton(app.btnXNeg));
runAction('manual_x_pos', @() pushButton(app.btnXPos));
runAction('manual_y_pos', @() pushButton(app.btnYPos));
runAction('manual_y_neg', @() pushButton(app.btnYNeg));
runAction('manual_z_neg', @() pushButton(app.btnZNeg));
runAction('manual_z_pos', @() pushButton(app.btnZPos));

runAction('trace_start', @() pushButton(app.btnTraceToggle));
runAction('trace_export', @() pushButton(app.btnTraceExport));
runAction('trace_stop', @() pushButton(app.btnTraceToggle));
runAction('trace_clear', @() pushButton(app.btnTraceClear));

runAction('pick_place_preview', @() pushButton(app.btnPreviewPickPlace));
runAction('pick_place_publish', @() pushButton(app.btnPublishPickPlace));
if isfield(app, 'pickplace')
	assert(isfile(app.pickplace.outputFile), 'Pick/place output MAT file was not created.');
	recordPass('pick_place_output_file_created');
end

runAction('publish_pose', @() pushButton(app.btnPublish));
runAction('open_model', @() pushButton(app.btnOpenModel));
runAction('run_hold', @() pushButton(app.btnRunHold));
runAction('run_current_path', @() pushButton(app.btnRunCurrentTraj));

disp('EE268_UI_OPTION_TEST_BEGIN');
fprintf('PASS_COUNT=%d\n', numel(results.passes));
for idx = 1:numel(results.passes)
	fprintf('PASS:%s\n', results.passes{idx});
end
fprintf('FAIL_COUNT=%d\n', numel(results.failures));
for idx = 1:numel(results.failures)
	fprintf('FAIL:%s\n', results.failures{idx});
end
disp('EE268_UI_OPTION_TEST_END');

	function runAction(name, fn)
		try
			fn();
			drawnow;
			recordPass(name);
		catch actionErr
			results.failures{end + 1} = sprintf('%s :: %s', name, actionErr.message); %#ok<AGROW>
		end
	end

	function recordPass(name)
		results.passes{end + 1} = name; %#ok<AGROW>
	end
end

function pushButton(buttonHandle)
callback = buttonHandle.ButtonPushedFcn;
if isa(callback, 'function_handle')
	callback(buttonHandle, struct());
elseif iscell(callback)
	feval(callback{:}, buttonHandle, struct());
else
	error('Unsupported button callback type.');
end
end

function cleanupApp(app)
try
	if isstruct(app) && isfield(app, 'fig') && isvalid(app.fig)
		delete(app.fig);
	end
catch
end
end
