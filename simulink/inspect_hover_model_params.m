% Inspect relevant model blocks and parameters without editing the model.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
target_driver_setup_2022_Ctrl_V_2_4;

mdl = 'updated_target_driver_2026_withVariants';
load_system(mdl);

patterns = {'F_tau_limit','Thurst_limit','Torque_roll_limit','Torque_pitch_limit', ...
    'Torque_yaw_limit','control_gain','k_R','k_Omega','k_Rx','k_z','k_vz', ...
    'force2voltage','max_drv_bias','drv_bias','satur','drake2controller'};

blocks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'Block');
fprintf('\nRelevant blocks/params in %s:\n', mdl);
for i = 1:numel(blocks)
    b = blocks{i};
    try
        dlg = get_param(b, 'DialogParameters');
    catch
        dlg = struct();
    end
    if ~isstruct(dlg)
        dlg = struct();
    end
    hit = false;
    details = {};
    if contains(lower(get_param(b, 'Name')), 'satur') || contains(lower(get_param(b, 'Name')), 'force') || contains(lower(get_param(b, 'Name')), 'voltage')
        hit = true;
    end
    f = fieldnames(dlg);
    for j = 1:numel(f)
        try
            val = get_param(b, f{j});
        catch
            continue;
        end
        if ischar(val) || isstring(val)
            for p = 1:numel(patterns)
                if contains(string(val), patterns{p}, 'IgnoreCase', true)
                    hit = true;
                    details{end+1} = sprintf('%s=%s', f{j}, char(string(val))); %#ok<AGROW>
                    break;
                end
            end
        end
    end
    if hit
        fprintf('  %s [%s]\n', b, get_param(b, 'BlockType'));
        for j = 1:numel(details)
            fprintf('      %s\n', details{j});
        end
    end
end

fprintf('\nLimits currently in workspace:\n');
fprintf('  F_tau_limit = %s\n', mat2str(F_tau_limit, 10));
fprintf('  body weight = %.10g N\n', m*g);
fprintf('  max_drv_bias = %.10g V\n', max_drv_bias);

fprintf('\nForce_to_Voltage1 subsystem blocks:\n');
forceBlocks = find_system([mdl '/Force_to_Voltage1'], 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'Block');
for i = 1:numel(forceBlocks)
    b = forceBlocks{i};
    fprintf('  %s [%s]\n', b, get_param(b, 'BlockType'));
    printParamIfPresent(b, 'Value');
    printParamIfPresent(b, 'UpperLimit');
    printParamIfPresent(b, 'LowerLimit');
    printParamIfPresent(b, 'Gain');
    printParamIfPresent(b, 'FunctionName');
end

fprintf('\nAll Saturate block limits:\n');
satBlocks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Saturate');
for i = 1:numel(satBlocks)
    b = satBlocks{i};
    fprintf('  %s\n', b);
    printParamIfPresent(b, 'UpperLimit');
    printParamIfPresent(b, 'LowerLimit');
end

fprintf('\nStateflow MATLAB Function snippets mentioning force/voltage/torque:\n');
rt = sfroot;
charts = rt.find('-isa', 'Stateflow.EMChart');
for i = 1:numel(charts)
    script = charts(i).Script;
    if contains(script, 'force', 'IgnoreCase', true) || contains(script, 'voltage', 'IgnoreCase', true) || contains(script, 'torque', 'IgnoreCase', true)
        fprintf('  %s\n', charts(i).Path);
        lines = splitlines(string(script));
        for j = 1:min(numel(lines), 80)
            fprintf('      %s\n', lines(j));
        end
    end
end

function printParamIfPresent(blockPath, paramName)
try
    val = get_param(blockPath, paramName);
    fprintf('      %s = %s\n', paramName, char(string(val)));
catch
end
end
