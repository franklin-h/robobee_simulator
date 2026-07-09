% True open-loop pitch-bias response sweep.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
target_driver_setup_2022_Ctrl_V_2_4;

mdl = 'updated_target_driver_2026_withVariants';
pitchValues = [-12, -8, -6.95, -4, 0, 4, 6.95, 8, 12];

fprintf('\nTrue open-loop pitch-bias sweep, eta=%.5f, amp=%.3f, roll=%.3f\n', ...
    params_vec(7), drv_amp, drv_roll);
fprintf('%10s %10s %10s %10s %10s %10s\n', ...
    'pitch', 'tilt20ms', 'tilt50ms', 'z50ms', 'zb_x50ms', 'zb_y50ms');

for i = 1:numel(pitchValues)
    clear mex;
    pause(0.1);
    in = Simulink.SimulationInput(mdl);
    in = in.setModelParameter('StopTime', '0.05');
    in = in.setVariable('closedloop_flag', 0);
    in = in.setVariable('drv_roll', drv_roll);
    in = in.setVariable('drv_amp', drv_amp);
    in = in.setVariable('drv_pitch_left', pitchValues(i));
    in = in.setVariable('drv_pitch_right', pitchValues(i));
    in = in.setVariable('a2_openloop', a2_openloop);
    out = sim(in);
    t = out.logsout.getElement(1).Values.Time(:);
    R = out.logsout.getElement(1).Values.Data;
    pos = out.logsout.getElement(3).Values.Data;
    tilt = acosd(max(min(R(:,9), 1), -1));
    i20 = find(t >= 0.02, 1);
    i50 = find(t >= 0.05, 1);
    fprintf('%10.3f %10.2f %10.2f %10.4f %10.4f %10.4f\n', ...
        pitchValues(i), tilt(i20), tilt(i50), pos(i50,3), R(i50,3), R(i50,6));
end
