modelPath = 'c:/Users/ahmed medhat/Desktop/Advanced Robotics Analysis Control MCT 442/Doritos/doritos_vs/doritos_mk2/doritos_mk2.slx';
analysisRoot = fileparts(mfilename('fullpath'));
projectRoot = fileparts(analysisRoot);
workspaceRoot = fileparts(projectRoot);
resultsFile = fullfile(analysisRoot, 'ee268_analysis_results.mat');
reportFile = fullfile(analysisRoot, 'doritos_mk2_ui_bridge_report.txt');

loaded = load(resultsFile, 'results');
toolOffset = loaded.results.context.tool_offset_base(:);

addpath(projectRoot);
addpath(fullfile(projectRoot, 'main_files'));
addpath(analysisRoot);

load_system(modelPath);
[~, modelName, ~] = fileparts(modelPath);
subsystemPath = [modelName '/Kinematics'];
signalBlock = [subsystemPath '/Signal Editor'];
fcnBlock = [subsystemPath '/MATLAB Function'];

set_model_callbacks(modelName, projectRoot, analysisRoot);
add_ui_source_path(subsystemPath, signalBlock, fcnBlock);
update_ik_function_block(fcnBlock, toolOffset);

save_system(modelName);
write_report(reportFile, modelName, subsystemPath);
bdclose(modelName);

function set_model_callbacks(modelName, projectRoot, analysisRoot)
callbackText = sprintf([ ...
    'projectRoot = ''%s'';\n' ...
    'analysisRoot = ''%s'';\n' ...
    'if exist(projectRoot, ''dir''), addpath(projectRoot); end\n' ...
    'if exist(fullfile(projectRoot, ''main_files''), ''dir''), addpath(fullfile(projectRoot, ''main_files'')); end\n' ...
    'if exist(analysisRoot, ''dir''), addpath(analysisRoot); end\n'], ...
    escape_quotes(projectRoot), escape_quotes(analysisRoot));
set_param(modelName, 'PreLoadFcn', callbackText);
set_param(modelName, 'InitFcn', callbackText);
end

function add_ui_source_path(subsystemPath, signalBlock, fcnBlock)
signalPos = get_param(signalBlock, 'Position');
fcnPos = get_param(fcnBlock, 'Position');

uiBlockLeft = signalPos(3) + 45;
uiBlockWidth = 90;
uiBlockHeight = 24;
switchLeft = uiBlockLeft + 125;
switchWidth = 30;
switchHeight = 40;

rowCenters = [signalPos(2) + 34, signalPos(2) + 82, signalPos(2) + 130];
workspaceVars = {'x_ref_ts', 'y_ref_ts', 'z_ref_ts'};
uiBlockNames = {'UI X', 'UI Y', 'UI Z'};
switchNames = {'X Source', 'Y Source', 'Z Source'};
outportNames = {'θ1', 'θ2', 'θ3'};

shiftAmount = max(0, (switchLeft + switchWidth + 50) - fcnPos(1));
shiftedFcnPos = fcnPos;
if shiftAmount > 0
    shiftedFcnPos = fcnPos + [shiftAmount, 0, shiftAmount, 0];
    set_param(fcnBlock, 'Position', shiftedFcnPos);
end

outLeft = shiftedFcnPos(3) + 25;
outWidth = 30;
outHeight = 14;
for idx = 1:3
    outTop = shiftedFcnPos(2) + 2 + (idx - 1) * 20;
    set_param([subsystemPath '/' outportNames{idx}], 'Position', [outLeft, outTop, outLeft + outWidth, outTop + outHeight]);
end

phSignal = get_param(signalBlock, 'PortHandles');
phFcn = get_param(fcnBlock, 'PortHandles');

for idx = 1:3
    uiBlock = [subsystemPath '/' uiBlockNames{idx}];
    switchBlock = [subsystemPath '/' switchNames{idx}];
    uiTop = rowCenters(idx) - uiBlockHeight / 2;
    switchTop = rowCenters(idx) - switchHeight / 2;

    ensure_block('simulink/Sources/From Workspace', uiBlock, [uiBlockLeft, uiTop, uiBlockLeft + uiBlockWidth, uiTop + uiBlockHeight]);
    set_param(uiBlock, 'VariableName', workspaceVars{idx});
    set_param(uiBlock, 'Interpolate', 'on');

    ensure_block('simulink/Signal Routing/Manual Switch', switchBlock, [switchLeft, switchTop, switchLeft + switchWidth, switchTop + switchHeight]);
    set_param(switchBlock, 'Orientation', 'right');

    delete_incoming_line(phFcn.Inport(idx));
    delete_outgoing_line(phSignal.Outport(idx));

    phUi = get_param(uiBlock, 'PortHandles');
    phSwitch = get_param(switchBlock, 'PortHandles');

    safe_add_line(subsystemPath, phSignal.Outport(idx), phSwitch.Inport(1));
    safe_add_line(subsystemPath, phUi.Outport, phSwitch.Inport(2));
    safe_add_line(subsystemPath, phSwitch.Outport, phFcn.Inport(idx));
end
end

function update_ik_function_block(fcnBlock, toolOffset)
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', fcnBlock);
if isempty(chart)
    error('Could not find MATLAB Function chart at %s', fcnBlock);
end

script = sprintf([ ...
        'function [theta1, theta2, theta3] = fcn(x, y, z)\n' ...
        '%% x,y,z are desired EE coordinates in meters.\n' ...
        'persistent lastTheta\n' ...
        'if isempty(lastTheta)\n' ...
        '    lastTheta = zeros(3,1);\n' ...
        'end\n\n' ...
        'params = delta_params();\n' ...
        'tool_offset_base = [%.9f; %.9f; %.9f];\n' ...
        'platform_p = [x; y; z] - tool_offset_base;\n\n' ...
        '[theta, valid, ~] = delta_IK(params, platform_p);\n\n' ...
        'if valid\n' ...
        '    lastTheta = theta(:);\n' ...
        'end\n\n' ...
        'theta1 = lastTheta(1);\n' ...
        'theta2 = lastTheta(2);\n' ...
        'theta3 = lastTheta(3);\n'], ...
        toolOffset(1), toolOffset(2), toolOffset(3));

chart.Script = script;
end

function write_report(reportFile, modelName, subsystemPath)
blocks = find_system(subsystemPath, 'SearchDepth', 1, 'Type', 'Block');
lines = find_system(subsystemPath, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');

fid = fopen(reportFile, 'w');
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'MODEL:%s\n', modelName);
fprintf(fid, 'SUBSYSTEM:%s\n', subsystemPath);
fprintf(fid, 'BLOCK_COUNT:%d\n', numel(blocks));
for idx = 1:numel(blocks)
    fprintf(fid, 'BLOCK:%s | TYPE:%s | POS:%s\n', blocks{idx}, get_param(blocks{idx}, 'BlockType'), mat2str(get_param(blocks{idx}, 'Position')));
end
fprintf(fid, '\nLINES\n');
for idx = 1:numel(lines)
    src = get_param(lines(idx), 'SrcBlockHandle');
    dst = get_param(lines(idx), 'DstBlockHandle');
    if src == -1 || isempty(dst)
        continue
    end
    fprintf(fid, 'LINE:%s -> ', getfullname(src));
    for dstIdx = 1:numel(dst)
        fprintf(fid, '%s;', getfullname(dst(dstIdx)));
    end
    fprintf(fid, '\n');
end
end

function ensure_block(libPath, blockPath, position)
blockHandle = getSimulinkBlockHandle(blockPath);
if blockHandle > 0
    set_param(blockPath, 'Position', position);
else
    add_block(libPath, blockPath, 'Position', position);
end
end

function delete_incoming_line(inportHandle)
lineHandle = get_param(inportHandle, 'Line');
if lineHandle ~= -1
    delete_line(lineHandle);
end
end

function delete_outgoing_line(outportHandle)
lineHandle = get_param(outportHandle, 'Line');
if lineHandle ~= -1
    delete_line(lineHandle);
end
end

function safe_add_line(subsystemPath, srcHandle, dstHandle)
try
    add_line(subsystemPath, srcHandle, dstHandle, 'autorouting', 'on');
catch
    % Line already present or ports already connected.
end
end

function out = escape_quotes(in)
out = strrep(in, '''', '''''');
end