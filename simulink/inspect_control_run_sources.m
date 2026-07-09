% Print source block paths for logged dataset elements.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
S = load(fullfile(thisDir, 'control_run.mat'));
data = S.data;

for idx = 1:data.numElements
    elem = data{idx};
    fprintf('%2d name="%s" propagated="%s"', idx, elem.Name, elem.PropagatedName);
    try
        bp = elem.BlockPath;
        fprintf(' path="%s"', blockPathToText(bp));
    catch err
        fprintf(' path_error="%s"', err.message);
    end
    try
        fprintf(' port=%d', elem.PortIndex);
    catch
    end
    fprintf('\n');
end

function txt = blockPathToText(bp)
txt = '';
try
    c = bp.convertToCell;
    if iscell(c)
        txt = strjoin(string(c), ' | ');
        return;
    end
catch
end
try
    txt = char(bp.getBlock(1));
    return;
catch
end
try
    txt = char(string(bp));
catch
    txt = '<unprintable>';
end
end
