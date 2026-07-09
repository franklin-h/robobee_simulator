% Compare hover behavior with selected feedback channels enabled/disabled.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
target_driver_setup_2022_Ctrl_V_2_4;

mdl = 'updated_target_driver_2026_withVariants';
baseGain = control_gain;

scenarios = {
    'open_loop_zero_all', zeros(size(baseGain));
    'current_all', baseGain;
    'attitude_only', [0 0 baseGain(3) baseGain(4) 0 0 baseGain(7)];
    'position_alt_only', [baseGain(1) baseGain(2) 0 0 baseGain(5) baseGain(6) 0];
    'no_lateral_pos', [0 0 baseGain(3) baseGain(4) baseGain(5) baseGain(6) baseGain(7)];
    'half_attitude', [baseGain(1) baseGain(2) 0.5*baseGain(3) 0.5*baseGain(4) baseGain(5) baseGain(6) 0.5*baseGain(7)];
    'quarter_attitude', [baseGain(1) baseGain(2) 0.25*baseGain(3) 0.25*baseGain(4) baseGain(5) baseGain(6) 0.25*baseGain(7)];
    'omega_damping_only', [0 0 0 baseGain(4) 0 0 0];
    'negative_omega_damping', [0 0 0 -baseGain(4) 0 0 0];
    'negative_attitude_sign', [0 0 -baseGain(3) -baseGain(4) 0 0 -baseGain(7)];
    };

fprintf('\nFeedback mode sweep, eta=%.5f, target z=%.4f\n', params_vec(7), r_desired(3));
fprintf('%-24s %8s %8s %10s %10s %10s %10s %10s\n', ...
    'scenario', 'flip_s', 'maxTilt', 'endTilt', 'endZ', 'maxAmpV', 'maxRollV', 'endRollV');

for i = 1:size(scenarios, 1)
    name = scenarios{i, 1};
    gain = scenarios{i, 2};
    clear mex;
    pause(0.1);
    in = Simulink.SimulationInput(mdl);
    in = in.setModelParameter('StopTime', '0.3');
    in = in.setVariable('control_gain', gain);
    in = in.setVariable('params_vec', params_vec);
    out = sim(in);
    data = out.logsout;
    t = data.getElement(1).Values.Time(:);
    R = data.getElement(1).Values.Data;
    pos = data.getElement(3).Values.Data;
    ampApplied = data.getElement(13).Values.Data;
    rollApplied = data.getElement(17).Values.Data;
    tilt = acosd(max(min(R(:,9), 1), -1));
    flipIdx = find(tilt > 90, 1, 'first');
    if isempty(flipIdx)
        flipText = 'never';
    else
        flipText = sprintf('%.4f', t(flipIdx));
    end
    fprintf('%-24s %8s %8.2f %10.2f %10.4f %10.3f %10.3f %10.3f\n', ...
        name, flipText, max(tilt), tilt(end), pos(end,3), ...
        max(abs(ampApplied)), max(abs(rollApplied)), rollApplied(end));
    fprintf('    applied roll min/max=% .3f/% .3f, gain=%s\n', min(rollApplied), max(rollApplied), mat2str(gain, 4));
end
