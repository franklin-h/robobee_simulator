% Inspect logsout source paths from a fresh short simulation.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
target_driver_setup_2022_Ctrl_V_2_4;

mdl = 'updated_target_driver_2026_withVariants';
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', '0.02');
out = sim(in);
data = out.logsout;

fprintf('logsout elements: %d\n', data.numElements);
for idx = 1:data.numElements
    elem = data.getElement(idx);
    fprintf('%2d name="%s" path="%s" port=%d size=%s\n', ...
        idx, elem.Name, blockPathToText(elem.BlockPath), elem.PortIndex, mat2str(size(elem.Values.Data)));
end

function txt = blockPathToText(bp)
try
    c = bp.convertToCell;
    txt = strjoin(string(c), ' | ');
catch
    txt = char(string(bp));
end
end
