% True open-loop roll-bias response sweep.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
target_driver_setup_2022_Ctrl_V_2_4;

mdl = 'updated_target_driver_2026_withVariants';
rollValues = [-4, -2, -1, -0.5, -0.1, 0, 0.5, 1, 2, 4];

fprintf('\nTrue open-loop roll-bias sweep, eta=%.5f, amp=%.3f, pitch=%.3f\n', ...
    params_vec(7), drv_amp, drv_pitch_left);
fprintf('%10s %10s %10s %10s %10s %10s\n', ...
    'drv_roll', 'tilt20ms', 'tilt50ms', 'z50ms', 'zb_x50ms', 'zb_y50ms');

for i = 1:numel(rollValues)
    clear mex;
    pause(0.1);
    in = Simulink.SimulationInput(mdl);
    in = in.setModelParameter('StopTime', '0.05');
    in = in.setVariable('closedloop_flag', 0);
    in = in.setVariable('drv_roll', rollValues(i));
    in = in.setVariable('drv_amp', drv_amp);
    in = in.setVariable('drv_pitch_left', drv_pitch_left);
    in = in.setVariable('drv_pitch_right', drv_pitch_right);
    in = in.setVariable('a2_openloop', a2_openloop);
    out = sim(in);
    t = out.logsout.getElement(1).Values.Time(:);
    R = out.logsout.getElement(1).Values.Data;
    pos = out.logsout.getElement(3).Values.Data;
    tilt = acosd(max(min(R(:,9), 1), -1));
    i20 = find(t >= 0.02, 1);
    i50 = find(t >= 0.05, 1);
    fprintf('%10.3f %10.2f %10.2f %10.4f %10.4f %10.4f\n', ...
        rollValues(i), tilt(i20), tilt(i50), pos(i50,3), R(i50,3), R(i50,6));
end
