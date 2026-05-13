modelPath = 'c:/Users/ahmed medhat/Desktop/Advanced Robotics Analysis Control MCT 442/Doritos/doritos_vs/doritos_mk2/doritos_mk2.slx';
outFile = fullfile(fileparts(mfilename('fullpath')), 'kinematics_layout.txt');

load_system(modelPath);
[~, modelName, ~] = fileparts(modelPath);
subsystemPath = [modelName '/Kinematics'];

blockNames = { ...
    [subsystemPath '/Signal Editor'], ...
    [subsystemPath '/MATLAB Function'], ...
    [subsystemPath '/θ1'], ...
    [subsystemPath '/θ2'], ...
    [subsystemPath '/θ3'] ...
    };

lines = find_system(subsystemPath, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');

fid = fopen(outFile, 'w');
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

for idx = 1:numel(blockNames)
    fprintf(fid, 'BLOCK:%s\n', blockNames{idx});
    fprintf(fid, 'POSITION:%s\n', mat2str(get_param(blockNames{idx}, 'Position')));
    ph = get_param(blockNames{idx}, 'PortHandles');
    fprintf(fid, 'INPORTS:%d | OUTPORTS:%d\n', numel(ph.Inport), numel(ph.Outport));
end

fprintf(fid, '\nLINES\n');
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

bdclose(modelName);