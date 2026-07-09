% Test whether the pitch-torque-to-pitch-bias mapping sign is destabilizing.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
target_driver_setup_2022_Ctrl_V_2_4;

mdl = 'updated_target_driver_2026_withVariants';
pvBase = params_vec;

cases = {
    'current_delta3', pvBase;
    'flipped_delta3', setIndex(pvBase, 3, -pvBase(3));
    };

fprintf('\nClosed-loop pitch mapping sign test\n');
fprintf('%-18s %10s %10s %10s %10s %10s %10s\n', ...
    'case', 'flip_s', 'maxTilt', 'endTilt', 'endZ', 'minPitchV', 'maxPitchV');

for i = 1:size(cases, 1)
    clear mex;
    pause(0.1);
    in = Simulink.SimulationInput(mdl);
    in = in.setModelParameter('StopTime', '0.12');
    in = in.setVariable('params_vec', cases{i, 2});
    in = in.setVariable('control_gain', control_gain);
    in = in.setVariable('closedloop_flag', 1);
    out = sim(in);
    t = out.logsout.getElement(1).Values.Time(:);
    R = out.logsout.getElement(1).Values.Data;
    pos = out.logsout.getElement(3).Values.Data;
    pitchLeft = out.logsout.getElement(15).Values.Data;
    pitchRight = out.logsout.getElement(16).Values.Data;
    tilt = acosd(max(min(R(:,9), 1), -1));
    flipIdx = find(tilt > 90, 1, 'first');
    if isempty(flipIdx)
        flipText = 'never';
    else
        flipText = sprintf('%.4f', t(flipIdx));
    end
    fprintf('%-18s %10s %10.2f %10.2f %10.4f %10.3f %10.3f\n', ...
        cases{i, 1}, flipText, max(tilt), tilt(end), pos(end,3), ...
        min([pitchLeft; pitchRight]), max([pitchLeft; pitchRight]));
end

function v = setIndex(v, idx, value)
v(idx) = value;
end
