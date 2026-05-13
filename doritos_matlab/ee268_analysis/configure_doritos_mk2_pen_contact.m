modelPath = 'c:/Users/ahmed medhat/Desktop/Advanced Robotics Analysis Control MCT 442/Doritos/doritos_vs/doritos_mk2/doritos_mk2.slx';
reportFile = fullfile(fileparts(mfilename('fullpath')), 'doritos_mk2_pen_contact_report.txt');

paperTransformName = 'Rigid Transform';
paperExportName = 'PaperContactFrame';
paperOuterPortName = 'Paper';
paperInputName = 'PaperFrameIn';

penBodyTransformName = 'PenBodyTransform';
penBodyName = 'PenBody';
penTipFrameName = 'PenTipFrame';
inkTipVisualName = 'InkTip';
inkTipVisualTransformName = 'InkTipVisualTransform';
paperInnerInputName = 'PaperFrameInput';

trailSensorName = 'PenTipTrailSensor';
trailConverterName = 'PenTipTrailPS2SL';
trailWorkspaceName = 'PenTipTrailToWorkspace';
trailVariableName = 'pen_tip_trail_xyz_m';
trailMaxDataPoints = 5000;
trailDecimation = 5;

penLengthMm = 115;
penBodyDimsMm = [8 8 110];
inkTipDimsMm = [4 4 2];
paperDimsMm = [500 500 1];

load_system(modelPath);
[~, modelName, ~] = fileparts(modelPath);

cleanupObj = onCleanup(@() bdclose(modelName)); %#ok<NASGU>
fid = fopen(reportFile, 'w');
cleanupReport = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, 'MODEL:%s\n', modelName);

baseOuter = [modelName '/Base'];
baseLink = [baseOuter '/base_link'];
gripperOuter = [modelName '/Gripper'];
gripperLink = [gripperOuter '/gripper_link'];

paperTransform = [baseLink '/' paperTransformName];
paperBrick = [baseLink '/Brick Solid'];
gripperRef = [gripperLink '/ReferenceFrame'];

assertBlockExists(paperTransform);
assertBlockExists(paperBrick);
assertBlockExists(gripperRef);

paperExportInner = addPhysicalPort(baseLink, paperExportName, 'Right');
paperExportOuter = addPhysicalPort(baseOuter, paperOuterPortName, 'Right');
ensurePhysicalConnection(baseLink, getRConnHandle(paperTransform), getRConnHandle(paperExportInner));
ensurePhysicalConnection(baseOuter, getRConnHandle([baseOuter '/base_link'], portIndexOfPortBlock(baseLink, paperExportInner)), getRConnHandle(paperExportOuter));

set_param(paperBrick, 'BrickDimensions', mat2str(paperDimsMm), 'BrickDimensionsUnits', 'mm');
set_param(paperBrick, 'GraphicDiffuseColor', '[1 1 1]', 'GraphicOpacity', '0.92');

deleteIfExists([gripperLink '/' penBodyTransformName]);
deleteIfExists([gripperLink '/' penBodyName]);
deleteIfExists([gripperLink '/' penTipFrameName]);
deleteIfExists([gripperLink '/' inkTipVisualName]);
deleteIfExists([gripperLink '/' inkTipVisualTransformName]);
deleteIfExists([gripperOuter '/' paperInputName]);
deleteIfExists([gripperLink '/' paperInnerInputName]);
deleteIfExists([gripperLink '/' trailSensorName]);
deleteIfExists([gripperLink '/' trailConverterName]);
deleteIfExists([gripperLink '/' trailWorkspaceName]);

penBodyTransform = add_block('sm_lib/Frames and Transforms/Rigid Transform', [gripperLink '/' penBodyTransformName], ...
    'MakeNameUnique', 'off', 'Position', nextBlockPosition(gripperLink, [430 860 470 900]));
set_param(penBodyTransform, ...
    'RotationMethod', 'None', ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', sprintf('[0 0 %.6f]', -penLengthMm / 2), ...
    'TranslationCartesianOffsetUnits', 'mm');

penBody = add_block('sm_lib/Body Elements/Brick Solid', [gripperLink '/' penBodyName], ...
    'MakeNameUnique', 'off', 'Position', nextBlockPosition(gripperLink, [520 860 560 900]));
set_param(penBody, ...
    'BrickDimensions', mat2str(penBodyDimsMm), ...
    'BrickDimensionsUnits', 'mm', ...
    'Density', '250', ...
    'DensityUnits', 'kg/m^3', ...
    'GraphicDiffuseColor', '[0.10 0.35 0.85]', ...
    'GraphicOpacity', '1', ...
    'DoExposeReferenceFrame', 'on');

penTipFrame = add_block('sm_lib/Frames and Transforms/Rigid Transform', [gripperLink '/' penTipFrameName], ...
    'MakeNameUnique', 'off', 'Position', nextBlockPosition(gripperLink, [430 930 470 970]));
set_param(penTipFrame, ...
    'RotationMethod', 'None', ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', sprintf('[0 0 %.6f]', -penLengthMm), ...
    'TranslationCartesianOffsetUnits', 'mm');

inkTipVisualTransform = add_block('sm_lib/Frames and Transforms/Rigid Transform', [gripperLink '/' inkTipVisualTransformName], ...
    'MakeNameUnique', 'off', 'Position', nextBlockPosition(gripperLink, [520 930 560 970]));
set_param(inkTipVisualTransform, ...
    'RotationMethod', 'None', ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', '[0 0 1]', ...
    'TranslationCartesianOffsetUnits', 'mm');

inkTipVisual = add_block('sm_lib/Body Elements/Brick Solid', [gripperLink '/' inkTipVisualName], ...
    'MakeNameUnique', 'off', 'Position', nextBlockPosition(gripperLink, [610 930 650 970]));
set_param(inkTipVisual, ...
    'BrickDimensions', mat2str(inkTipDimsMm), ...
    'BrickDimensionsUnits', 'mm', ...
    'Density', '500', ...
    'DensityUnits', 'kg/m^3', ...
    'GraphicDiffuseColor', '[0 0 0]', ...
    'GraphicOpacity', '1', ...
    'DoExposeReferenceFrame', 'on');

paperInputOuter = addPhysicalPort(gripperOuter, paperInputName, 'Left');
paperInputInner = addPhysicalPort(gripperLink, paperInnerInputName, 'Left');

ensurePhysicalConnection(gripperLink, getRConnHandle(gripperRef), getLConnHandle(penBodyTransform));
ensurePhysicalConnection(gripperLink, getRConnHandle(penBodyTransform), getRConnHandle(penBody));
ensurePhysicalConnection(gripperLink, getRConnHandle(gripperRef), getLConnHandle(penTipFrame));
ensurePhysicalConnection(gripperLink, getRConnHandle(penTipFrame), getLConnHandle(inkTipVisualTransform));
ensurePhysicalConnection(gripperLink, getRConnHandle(inkTipVisualTransform), getRConnHandle(inkTipVisual));

trailSensor = add_block('sm_lib/Frames and Transforms/Transform Sensor', [gripperLink '/' trailSensorName], ...
    'MakeNameUnique', 'off', 'Position', nextBlockPosition(gripperLink, [700 930 780 1010]));
set_param(trailSensor, 'MeasurementFrame', 'Base', 'SenseXYZ', 'on');

trailConverter = add_block('nesl_utility/PS-Simulink Converter', [gripperLink '/' trailConverterName], ...
    'MakeNameUnique', 'off', 'Position', nextBlockPosition(gripperLink, [835 930 915 1010]));

trailWorkspace = add_block('simulink/Sinks/To Workspace', [gripperLink '/' trailWorkspaceName], ...
    'MakeNameUnique', 'off', 'Position', nextBlockPosition(gripperLink, [965 930 1075 1010]));
set_param(trailWorkspace, ...
    'VariableName', trailVariableName, ...
    'SaveFormat', 'Array', ...
    'MaxDataPoints', num2str(trailMaxDataPoints), ...
    'Decimation', num2str(trailDecimation));

basePortHandle = getRConnHandle(baseOuter, portIndexOfPortBlock(baseOuter, paperExportOuter));
gripperPaperHandle = getLConnHandle(gripperOuter, portIndexOfPortBlock(gripperOuter, paperInputOuter));

ensurePhysicalConnection(modelName, basePortHandle, gripperPaperHandle);
ensurePhysicalConnection(gripperOuter, getRConnHandle(paperInputOuter), getLConnHandle([gripperOuter '/gripper_link'], portIndexOfPortBlock(gripperLink, paperInputInner)));
ensurePhysicalConnection(gripperLink, getRConnHandle(paperInputInner), getLConnHandle(trailSensor));
ensurePhysicalConnection(gripperLink, getRConnHandle(penTipFrame), getRConnHandle(trailSensor, 1));
ensurePhysicalConnection(gripperLink, getRConnHandle(trailSensor, 2), getLConnHandle(trailConverter));
ensureSignalConnection(gripperLink, getOutportHandle(trailConverter), getInportHandle(trailWorkspace));

save_system(modelPath);

fprintf(fid, 'PAPER_FRAME:%s\n', paperExportInner);
fprintf(fid, 'PAPER_INPUT:%s\n', paperInputInner);
fprintf(fid, 'PEN_TIP_FRAME:%s\n', penTipFrame);
fprintf(fid, 'TRAIL_SENSOR:%s\n', trailSensor);
fprintf(fid, 'TRAIL_WORKSPACE:%s\n', trailVariableName);
fprintf(fid, 'TRAIL_MAX_DATA_POINTS:%d\n', trailMaxDataPoints);
fprintf(fid, 'TRAIL_DECIMATION:%d\n', trailDecimation);
fprintf(fid, 'PEN_LENGTH_MM:%.3f\n', penLengthMm);
fprintf(fid, 'STATUS:OK\n');

function assertBlockExists(blockPath)
if getSimulinkBlockHandle(blockPath) <= 0
    error('Block not found: %s', blockPath);
end
end

function deleteIfExists(blockPath)
if getSimulinkBlockHandle(blockPath) > 0
    delete_block(blockPath);
end
end

function blockPath = addPhysicalPort(systemPath, portName, side)
existing = [systemPath '/' portName];
if getSimulinkBlockHandle(existing) > 0
    blockPath = existing;
    return
end

portNum = nextPortNumber(systemPath, side);
position = nextPortPosition(systemPath, side);
blockHandle = add_block('built-in/PMIOPort', existing, 'MakeNameUnique', 'off', 'Position', position);
blockPath = getfullname(blockHandle);
set_param(blockPath, 'Side', side, 'Port', num2str(portNum));
end

function portNum = nextPortNumber(systemPath, side)
blocks = find_system(systemPath, 'SearchDepth', 1, 'BlockType', 'PMIOPort');
portNums = [];
for idx = 1:numel(blocks)
    if strcmpi(get_param(blocks{idx}, 'Side'), side)
        rawPort = strtrim(get_param(blocks{idx}, 'Port'));
        if isempty(rawPort)
            portNums(end + 1) = 1; %#ok<AGROW>
        else
            portNums(end + 1) = str2double(rawPort); %#ok<AGROW>
        end
    end
end
if isempty(portNums)
    portNum = 1;
else
    portNum = max(portNums) + 1;
end
end

function position = nextPortPosition(systemPath, side)
blocks = find_system(systemPath, 'SearchDepth', 1, 'BlockType', 'PMIOPort');
positions = [];
for idx = 1:numel(blocks)
    if strcmpi(get_param(blocks{idx}, 'Side'), side)
        positions(end + 1, :) = get_param(blocks{idx}, 'Position'); %#ok<AGROW>
    end
end

if isempty(positions)
    position = [300 60 330 74];
    return
end

switch lower(side)
    case 'right'
        x0 = max(positions(:, 1));
        x1 = max(positions(:, 3));
        y0 = max(positions(:, 2)) + 18;
        position = [x0 y0 x1 y0 + 14];
    case 'left'
        x0 = min(positions(:, 1));
        x1 = min(positions(:, 3));
        y0 = max(positions(:, 2)) + 18;
        position = [x0 y0 x1 y0 + 14];
    otherwise
        x0 = max(positions(:, 1)) + 18;
        x1 = max(positions(:, 3)) + 18;
        y0 = max(positions(:, 2));
        y1 = max(positions(:, 4));
        position = [x0 y0 x1 y1];
end
end

function idx = portIndexOfPortBlock(systemPath, blockPath)
blockPath = normalizeBlockPath(blockPath);
blocks = find_system(systemPath, 'SearchDepth', 1, 'BlockType', 'PMIOPort');
sides = cell(size(blocks));
positions = zeros(numel(blocks), 4);
for idxBlock = 1:numel(blocks)
    sides{idxBlock} = get_param(blocks{idxBlock}, 'Side');
    positions(idxBlock, :) = get_param(blocks{idxBlock}, 'Position');
end
targetSide = get_param(blockPath, 'Side');
targetPosition = get_param(blockPath, 'Position');
sameSide = find(strcmpi(sides, targetSide));
if any(strcmpi(targetSide, {'Right', 'Left'}))
    [~, order] = sort(positions(sameSide, 2), 'ascend');
else
    [~, order] = sort(positions(sameSide, 1), 'ascend');
end
orderedBlocks = blocks(sameSide(order));
idx = find(strcmp(orderedBlocks, blockPath), 1, 'first');
if isempty(idx)
    error('Could not resolve connector index for %s', blockPath);
end
if ~isequal(targetPosition, get_param(blockPath, 'Position'))
    error('Connector position changed unexpectedly for %s', blockPath);
end
end

function pos = nextBlockPosition(systemPath, seedPosition)
blocks = find_system(systemPath, 'SearchDepth', 1, 'Type', 'Block');
maxY = seedPosition(2);
for idx = 1:numel(blocks)
    pos = get_param(blocks{idx}, 'Position');
    maxY = max(maxY, pos(4));
end
height = seedPosition(4) - seedPosition(2);
width = seedPosition(3) - seedPosition(1);
pos = [seedPosition(1), maxY + 35, seedPosition(1) + width, maxY + 35 + height];
end

function ensurePhysicalConnection(systemPath, srcHandle, dstHandle)
if hasPhysicalConnection(srcHandle, dstHandle) || hasAnyConnection(srcHandle) || hasAnyConnection(dstHandle)
    return
end
add_line(systemPath, srcHandle, dstHandle, 'autorouting', 'on');
end

function ensureSignalConnection(systemPath, srcHandle, dstHandle)
if hasSignalConnection(srcHandle, dstHandle) || hasAnyConnection(srcHandle) || hasAnyConnection(dstHandle)
    return
end
add_line(systemPath, srcHandle, dstHandle, 'autorouting', 'on');
end

function tf = hasAnyConnection(portHandle)
lineHandle = get_param(portHandle, 'Line');
tf = ~(isempty(lineHandle) || any(lineHandle == -1));
end

function tf = hasPhysicalConnection(srcHandle, dstHandle)
tf = false;
lineHandles = get_param(srcHandle, 'Line');
if isempty(lineHandles) || any(lineHandles == -1)
    return
end
if ~iscell(lineHandles)
    lineHandles = num2cell(lineHandles);
end
for idx = 1:numel(lineHandles)
    lh = lineHandles{idx};
    if lh == -1
        continue
    end
    dstHandles = get_param(lh, 'DstPortHandle');
    if any(dstHandles == dstHandle)
        tf = true;
        return
    end
end
end

function tf = hasSignalConnection(srcHandle, dstHandle)
tf = false;
lineHandle = get_param(srcHandle, 'Line');
if isempty(lineHandle) || lineHandle == -1
    return
end
dstHandles = get_param(lineHandle, 'DstPortHandle');
tf = any(dstHandles == dstHandle);
end

function handle = getLConnHandle(blockPath, index)
if nargin < 2
    index = 1;
end
ports = get_param(blockPath, 'PortHandles');
handle = ports.LConn(index);
end

function handle = getRConnHandle(blockPath, index)
if nargin < 2
    index = 1;
end
blockPath = normalizeBlockPath(blockPath);
ports = get_param(blockPath, 'PortHandles');
handle = ports.RConn(index);
end

function handle = getInportHandle(blockPath, index)
if nargin < 2
    index = 1;
end
blockPath = normalizeBlockPath(blockPath);
ports = get_param(blockPath, 'PortHandles');
handle = ports.Inport(index);
end

function handle = getOutportHandle(blockPath, index)
if nargin < 2
    index = 1;
end
blockPath = normalizeBlockPath(blockPath);
ports = get_param(blockPath, 'PortHandles');
handle = ports.Outport(index);
end

function blockPath = normalizeBlockPath(blockPath)
if isnumeric(blockPath)
    blockPath = getfullname(blockPath);
end
end