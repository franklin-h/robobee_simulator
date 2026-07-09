clear
clc 
data = load('RoboBee_optimal_fitting_parameter_155Hz_2022_BBee_v2'); 
params_opt = data.params_opt; 
params_opt.gamma_2_1 = 0.011; 

% for 155 hz 
% params_opt.nu = -5; % More negative nu results in more positive pitch
% torque. 
% params_opt.nu = 0; % Results in -1.969e-6 N*m of pitch torque
% params_opt.nu = -1; % results in -1.62e-6 N*m of pitch torque 
% params_opt.nu = -5; % Results in -5e-7 N*m of pitch torque; 
% params_opt.nu = -7; % Results in -1e-7 N*m of pitch torque. 
% params_opt.nu = -7.7; % results in +0.4e-7 N*m of pitch torque 
% params_opt.nu = -9; % Results in +4.0e-7 


% For 180 Hz, nu sweep. 
% -7.5  gives  1.700e-07 
% -7.0  gives  0.198e-07 
%  6.95 gives  0.005e-07 FINAL 
% -6.9  gives -0.102e-07
% -6.0  gives -2.875e-07 

% I find that -0.1 = drv_roll trims it a lot better. 
% Solving for eta, we get 1.0043. 
params_opt.nu = -6.95; 
params_opt.eta = 1.0; % basically fully balanced  
params_opt.mu = 0.0; 
params_vec = [params_opt.delta_1, params_opt.delta_2, params_opt.delta_3, ...
    params_opt.gamma_1, params_opt.gamma_2_1, params_opt.gamma_3, ...
    params_opt.eta, params_opt.nu, params_opt.mu]';

% 0.14 seconds 
Thrust_desired = 1.2e-3; 
Roll_torque_desired = -1e-6; 
Pitch_torque_desired = 0; 
Yaw_torque_desired = 0; 
[drv_amp, drv_roll, drv_pitch_left, drv_pitch_right, a2_coeff] = ...
force2voltage(Thrust_desired, Roll_torque_desired, Pitch_torque_desired, ...
Yaw_torque_desired, params_vec)
