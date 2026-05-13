modelPath = 'c:/Users/ahmed medhat/Desktop/Advanced Robotics Analysis Control MCT 442/Doritos/doritos_vs/doritos_mk2/doritos_mk2.slx';
outFile = fullfile(fileparts(mfilename('fullpath')), 'kinematics_blocks.txt');

load_system(modelPath);
[~, modelName, ~] = fileparts(modelPath);
subsystemPath = [modelName '/Kinematics'];

blocks = find_system(subsystemPath, 'SearchDepth', 1, 'Type', 'Block');

fid = fopen(outFile, 'w');
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, 'SUBSYSTEM:%s\n', subsystemPath);
fprintf(fid, 'BLOCK_COUNT:%d\n', numel(blocks));
for idx = 1:numel(blocks)
    fprintf(fid, 'BLOCK:%s | TYPE:%s\n', blocks{idx}, get_param(blocks{idx}, 'BlockType'));
end

bdclose(modelName);