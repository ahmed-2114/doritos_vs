modelPath = 'c:/Users/ahmed medhat/Desktop/Advanced Robotics Analysis Control MCT 442/Doritos/doritos_vs/doritos_mk2/doritos_mk2.slx';
outFile = fullfile(fileparts(mfilename('fullpath')), 'simulink_top_blocks.txt');

load_system(modelPath);
[~, modelName, ~] = fileparts(modelPath);

blocks = find_system(modelName, 'SearchDepth', 1, 'Type', 'Block');
lines = find_system(modelName, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');
kinBlocks = find_system([modelName '/Kinematics'], 'SearchDepth', 1, 'Type', 'Block');
kinLines = find_system([modelName '/Kinematics'], 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');

fid = fopen(outFile, 'w');
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, 'MODEL:%s\n', modelName);
fprintf(fid, 'BLOCK_COUNT:%d\n', numel(blocks));
for idx = 1:numel(blocks)
    fprintf(fid, 'BLOCK:%s | TYPE:%s\n', blocks{idx}, get_param(blocks{idx}, 'BlockType'));
end

fprintf(fid, '\nLINE_COUNT:%d\n', numel(lines));
for idx = 1:numel(lines)
    src = get_param(lines(idx), 'SrcBlockHandle');
    dst = get_param(lines(idx), 'DstBlockHandle');
    if src == -1 || isempty(dst)
        continue
    end
    srcName = getfullname(src);
    dstNames = cell(1, numel(dst));
    for dstIdx = 1:numel(dst)
        dstNames{dstIdx} = getfullname(dst(dstIdx));
    end
    fprintf(fid, 'LINE:%s -> %s\n', srcName, strjoin(dstNames, ', '));
end

fprintf(fid, '\nKINEMATICS_BLOCK_COUNT:%d\n', numel(kinBlocks));
for idx = 1:numel(kinBlocks)
    fprintf(fid, 'KIN_BLOCK:%s | TYPE:%s\n', kinBlocks{idx}, get_param(kinBlocks{idx}, 'BlockType'));
end

fprintf(fid, '\nKINEMATICS_LINE_COUNT:%d\n', numel(kinLines));
for idx = 1:numel(kinLines)
    src = get_param(kinLines(idx), 'SrcBlockHandle');
    dst = get_param(kinLines(idx), 'DstBlockHandle');
    if src == -1 || isempty(dst)
        continue
    end
    srcName = getfullname(src);
    dstNames = cell(1, numel(dst));
    for dstIdx = 1:numel(dst)
        dstNames{dstIdx} = getfullname(dst(dstIdx));
    end
    fprintf(fid, 'KIN_LINE:%s -> %s\n', srcName, strjoin(dstNames, ', '));
end

bdclose(modelName);