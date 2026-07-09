% Print the Geometric Controller MATLAB Function code from the model.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
mdl = 'updated_target_driver_2026_withVariants';
load_system(mdl);

rt = sfroot;
charts = rt.find('-isa', 'Stateflow.EMChart');
for i = 1:numel(charts)
    if strcmp(charts(i).Path, [mdl '/Geometric Controller/Geometric Controller'])
        lines = splitlines(string(charts(i).Script));
        for j = 1:numel(lines)
            fprintf('%3d: %s\n', j, lines(j));
        end
    end
end
