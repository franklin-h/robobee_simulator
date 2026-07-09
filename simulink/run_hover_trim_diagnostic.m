% Run the existing setup and trim diagnostic for the current hover model.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);

target_driver_setup_2022_Ctrl_V_2_4;

fprintf('\nKey setup values after target_driver_setup_2022_Ctrl_V_2_4:\n');
fprintf('  m*g = %.6g N, hover target = [%g %g %g] m\n', m*g, r_desired);
fprintf('  drive bias = %.3f V, open-loop drv_amp = %.3f, pitch = %.3f / %.3f, a2 = %.3f\n', ...
    drv_bias, drv_amp, drv_pitch_left, drv_pitch_right, a2_openloop);
fprintf('  params_vec = %s\n', mat2str(params_vec.', 6));
fprintf('  control_gain = %s\n', mat2str(control_gain, 6));

robobee_trim_test(0.1, 0.3);
